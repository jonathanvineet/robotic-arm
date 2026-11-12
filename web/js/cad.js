// cad.js — loads web/models/arm-model.json (made from Final_assembly.STEP by
// tools/step-to-model.mjs) and builds the real arm for the digital twin.
//
// How: build the joint chain from the measured kinematics, pose it exactly as
// the arm was modelled in SolidWorks, attach every part where it sits in the
// CAD (three's attach() keeps world placement), then return to the zero pose.
// From then on each part turns around its real joint axis.

import * as THREE from 'three';

const D2R = Math.PI / 180;

function bytes(b64) {
  const bin = atob(b64);
  const u = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) u[i] = bin.charCodeAt(i);
  return u;
}

export async function loadCad(url) {
  const res = await fetch(url);
  if (!res.ok) throw new Error(`${url}: HTTP ${res.status}`);
  const model = await res.json();
  model.geos = model.meshes.map((m) => {
    const q = new Uint16Array(bytes(m.pos).buffer);
    const pos = new Float32Array(q.length);
    for (let i = 0; i < q.length; i += 3) for (let k = 0; k < 3; k++) pos[i + k] = m.min[k] + q[i + k] * m.scale[k];
    const ib = bytes(m.idx).buffer;
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    g.setIndex(new THREE.BufferAttribute(m.idx32 ? new Uint32Array(ib) : new Uint16Array(ib), 1));
    g.computeVertexNormals();
    g.computeBoundingSphere();
    return g;
  });
  return model;
}

/** @param matFor (color string) => THREE.Material */
export function buildCadArm(cad, matFor) {
  const K = cad.kin;
  const L = K.links;
  const root = new THREE.Group();
  const yaw = new THREE.Group();
  root.add(yaw);
  const shoulder = new THREE.Group();
  shoulder.position.set(L.fwd, L.base, -L.side);
  yaw.add(shoulder);
  const elbow = new THREE.Group();
  elbow.position.set(0, L.upper, 0);
  shoulder.add(elbow);
  const roll = new THREE.Group(); // wrist roll comes before pitch on this arm
  elbow.add(roll);
  const pitch = new THREE.Group();
  pitch.position.set(0, L.fore, 0);
  roll.add(pitch);

  const pose = (y, q1, q2, r, q3) => {
    yaw.rotation.y = y;
    shoulder.rotation.z = -q1;
    elbow.rotation.z = -q2;
    roll.rotation.y = r;
    pitch.rotation.z = -q3;
    root.updateMatrixWorld(true);
  };
  pose(K.yaw0 * D2R, K.q1_0 * D2R, K.q2_0 * D2R, K.roll0 * D2R, K.q3_0 * D2R);

  // claw pivots (the two meshing gears turn equal and opposite)
  const fingers = cad.gripper.pivots.map((pv) => {
    const g = new THREE.Group();
    pitch.add(g);
    g.position.copy(pitch.worldToLocal(new THREE.Vector3(...pv.centre)));
    const inv = pitch.getWorldQuaternion(new THREE.Quaternion()).invert();
    const ax = new THREE.Vector3(...pv.axis).applyQuaternion(inv).normalize();
    g.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), ax);
    const spin = new THREE.Group();
    spin.userData = { side: pv.side, sign: pv.sign, close: cad.gripper.closeAngle * D2R };
    g.add(spin);
    g.updateMatrixWorld(true);
    return spin;
  });

  const groups = { base: root, yaw, upper: shoulder, fore: elbow, roll, pitch };
  const M = new THREE.Matrix4();
  for (const part of cad.parts) {
    const mesh = new THREE.Mesh(cad.geos[part.mesh], matFor(part.color));
    M.fromArray(part.matrix).decompose(mesh.position, mesh.quaternion, mesh.scale);
    mesh.castShadow = mesh.receiveShadow = true;
    mesh.name = part.name;
    root.add(mesh);
    mesh.updateMatrixWorld(true);
    const target = part.link.startsWith('finger')
      ? fingers.find((f) => f.userData.side === +part.link.slice(6))
      : groups[part.link] || pitch;
    target.attach(mesh);
  }
  pose(0, 0, 0, 0, 0);
  return { root, yaw, shoulder, elbow, roll, wrist: pitch, hand: pitch, fingers, cad: true };
}
