// npm start — serves the RoboArm web app on http://localhost:8080 and opens the browser.
// Everything runs in the page: controller, digital twin, simulator, and the Vision tab's
// webcam + YOLO plastic detection + ArUco markers (no Python needed at run time).
//
//   npm start                 # web app, opens the browser
//   npm start -- --no-open
//   npm run bridge            # optional: the Python vision bridge instead (camera handled by Python)
import { spawn } from 'node:child_process';
import { existsSync } from 'node:fs';
import { join, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const port = +process.env.PORT || 8080;
const say = (m) => console.log(`\x1b[36m[start]\x1b[0m ${m}`);

if (!existsSync(join(root, 'web', 'models', 'plastic.onnx')))
  say('No web/models/plastic.onnx yet: the Vision tab will find markers but not plastic. Convert the model once: cd Plastic-Segregation-AI && python export_onnx.py');

process.env.PORT = String(port);
await import('./serve.mjs');
const url = `http://localhost:${port}/`;
say(`Open ${url}  (Ctrl+C to stop)`);
if (!process.argv.includes('--no-open') && !process.env.NO_OPEN) {
  const win = process.platform === 'win32';
  const [c, a] = win ? ['cmd', ['/c', 'start', '""', url]] : process.platform === 'darwin' ? ['open', [url]] : ['xdg-open', [url]];
  spawn(c, a, { stdio: 'ignore', detached: true }).on('error', () => {}).unref();
}
