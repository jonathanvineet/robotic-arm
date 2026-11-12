// npm start — runs the whole thing:
//   * Plastic-Segregation-AI/vision_bridge.py (camera + YOLO + ArUco markers), which
//     also serves the web app (controller, digital twin, simulator) on one port
//   * opens the browser
// No camera / no YOLO? The bridge still runs (markers-only / retries the camera) and the
// simulator works. No Python at all? Falls back to the plain web server (simulator only).
//
//   npm start                         # camera 0
//   npm start -- --camera 1           # any vision_bridge.py option after --
//   npm start -- --device 0           # NVIDIA GPU
//   npm start -- --no-open            # don't open the browser
import { spawn, spawnSync } from 'node:child_process';
import { existsSync } from 'node:fs';
import { join, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const ai = join(root, 'Plastic-Segregation-AI');
const win = process.platform === 'win32';
const argv = process.argv.slice(2);
const noOpen = argv.includes('--no-open') || process.env.NO_OPEN;
const bridgeArgs = argv.filter((a) => a !== '--no-open');
const portIdx = bridgeArgs.indexOf('--port');
const port = portIdx >= 0 ? +bridgeArgs[portIdx + 1] : +process.env.PORT || 8765;
if (portIdx < 0) bridgeArgs.push('--port', String(port));

const say = (m) => console.log(`\x1b[36m[start]\x1b[0m ${m}`);

// ---------------------------------------------------------------- find Python with OpenCV
function findPython() {
  const venvs = ['.venv', 'venv', 'ai_env'].map((v) => [join(ai, v, win ? 'Scripts\\python.exe' : 'bin/python')]);
  const sys = win ? [['py', '-3'], ['python'], ['python3']] : [['python3'], ['python']];
  let noCv = null;
  for (const cmd of [...venvs.filter(([p]) => existsSync(p)), ...sys]) {
    const r = spawnSync(cmd[0], [...cmd.slice(1), '-c', 'import cv2, sys; print(cv2.__version__, sys.version.split()[0])'], { encoding: 'utf8' });
    if (r.status === 0) return { cmd, info: r.stdout.trim() };
    const v = spawnSync(cmd[0], [...cmd.slice(1), '--version'], { encoding: 'utf8' });
    if (v.status === 0 && !noCv) noCv = cmd;
  }
  return { cmd: null, noCv };
}

function openBrowser(url) {
  if (noOpen) return;
  const [c, a] = win ? ['cmd', ['/c', 'start', '""', url]] : process.platform === 'darwin' ? ['open', [url]] : ['xdg-open', [url]];
  spawn(c, a, { stdio: 'ignore', detached: true }).on('error', () => {}).unref();
}

async function waitFor(url, ms = 30000) {
  const t0 = Date.now();
  while (Date.now() - t0 < ms) {
    try {
      const r = await fetch(url);
      if (r.ok) return true;
    } catch {}
    await new Promise((r) => setTimeout(r, 300));
  }
  return false;
}

function fallbackWeb(reason) {
  say(`${reason}`);
  say('Starting the web app only (simulator works; no real camera). Fix the above and run npm start again.');
  const webPort = 8080;
  const child = spawn(process.execPath, [join(root, 'tools', 'serve.mjs')], { stdio: 'inherit', env: { ...process.env, PORT: String(webPort) } });
  waitFor(`http://localhost:${webPort}/`).then((ok) => ok && openBrowser(`http://localhost:${webPort}/`));
  child.on('exit', (c) => process.exit(c ?? 0));
}

const py = findPython();
if (!py.cmd) {
  fallbackWeb(
    py.noCv
      ? `Python found but OpenCV is missing. Install the vision deps:\n    cd Plastic-Segregation-AI\n    ${py.noCv.join(' ')} -m pip install -r requirements.txt`
      : 'Python 3 not found (install it from python.org, tick "Add to PATH" on Windows).',
  );
} else {
  say(`Python: ${py.cmd.join(' ')} (OpenCV/Python ${py.info})`);
  say(`Starting vision bridge + web app on http://localhost:${port}  (Ctrl+C to stop)`);
  const child = spawn(py.cmd[0], [...py.cmd.slice(1), '-u', join(ai, 'vision_bridge.py'), ...bridgeArgs], {
    stdio: 'inherit',
    cwd: ai,
    env: { ...process.env, PYTHONUNBUFFERED: '1' },
  });
  child.on('error', (e) => fallbackWeb(`Could not start the vision bridge: ${e.message}`));
  child.on('exit', (code) => {
    if (code) say(`vision bridge exited with code ${code}`);
    process.exit(code ?? 0);
  });
  for (const sig of ['SIGINT', 'SIGTERM']) process.on(sig, () => child.kill(sig));
  waitFor(`http://localhost:${port}/api/info`).then((ok) => {
    if (ok) {
      say(`Ready → http://localhost:${port}`);
      openBrowser(`http://localhost:${port}/`);
    } else say('bridge did not answer in 30 s — check the messages above');
  });
}
