// npm run bridge — OPTIONAL alternative to in-browser detection: runs
// Plastic-Segregation-AI/vision_bridge.py (camera + YOLO via Python/PyTorch), which also
// serves the web app on http://localhost:8765, and opens the browser.
//
//   npm run bridge -- --camera 1 --device 0     # any vision_bridge.py option after --
import { spawn, spawnSync } from 'node:child_process';
import { existsSync } from 'node:fs';
import { join, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const ai = join(root, 'Plastic-Segregation-AI');
const win = process.platform === 'win32';
const argv = process.argv.slice(2);
const noOpen = argv.includes('--no-open') || process.env.NO_OPEN;
const args = argv.filter((a) => a !== '--no-open');
const pi = args.indexOf('--port');
const port = pi >= 0 ? +args[pi + 1] : 8765;
if (pi < 0) args.push('--port', String(port));
const say = (m) => console.log(`\x1b[36m[bridge]\x1b[0m ${m}`);

function findPython() {
  const venvs = ['.venv', 'venv', 'ai_env'].map((v) => join(ai, v, win ? 'Scripts\\python.exe' : 'bin/python')).filter(existsSync).map((p) => [p]);
  for (const cmd of [...venvs, ...(win ? [['py', '-3'], ['python']] : [['python3'], ['python']])]) {
    const r = spawnSync(cmd[0], [...cmd.slice(1), '-c', 'import cv2'], { encoding: 'utf8' });
    if (r.status === 0) return cmd;
  }
  return null;
}

const py = findPython();
if (!py) {
  say('No Python with OpenCV found. Use "npm start" instead (detection runs in the browser), or set up');
  say('Plastic-Segregation-AI/.venv:  python -m venv .venv  then  .venv/bin/pip install -r requirements.txt');
  process.exit(1);
}
say(`${py.join(' ')} vision_bridge.py → http://localhost:${port}`);
const child = spawn(py[0], [...py.slice(1), '-u', join(ai, 'vision_bridge.py'), ...args], { stdio: 'inherit', cwd: ai, env: { ...process.env, PYTHONUNBUFFERED: '1' } });
child.on('exit', (c) => process.exit(c ?? 0));
for (const sig of ['SIGINT', 'SIGTERM']) process.on(sig, () => child.kill(sig));
const url = `http://localhost:${port}/`;
const t0 = Date.now();
(async function wait() {
  try {
    if ((await fetch(url + 'api/info')).ok) {
      say(`Ready → ${url}`);
      if (!noOpen) {
        const [c, a] = win ? ['cmd', ['/c', 'start', '""', url]] : process.platform === 'darwin' ? ['open', [url]] : ['xdg-open', [url]];
        spawn(c, a, { stdio: 'ignore', detached: true }).on('error', () => {}).unref();
      }
      return;
    }
  } catch {}
  if (Date.now() - t0 < 30000) setTimeout(wait, 300);
})();
