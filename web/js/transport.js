// transport.js — three interchangeable links to "the arm":
//   SimTransport    in-browser VirtualArm (no hardware)
//   WsTransport     ESP over Wi-Fi (WebSocket, port 81)
//   SerialTransport ESP over USB cable (Web Serial, Chrome/Edge)
// All expose: connect(), close(), send(obj), and the callbacks
// onmessage(obj), onstatus('connecting'|'online'|'offline', detail), onlog(text).

import { VirtualArm } from './sim.js';

class Base {
  constructor() {
    this.onmessage = () => {};
    this.onstatus = () => {};
    this.onlog = () => {};
    this.status = 'offline';
  }
  setStatus(s, detail = '') {
    this.status = s;
    this.onstatus(s, detail);
  }
}

// ------------------------------------------------------------------ simulator
export class SimTransport extends Base {
  constructor({ latency = 0, jitter = 0, flashKey = 'roboarm.simflash' } = {}) {
    super();
    this.label = 'Simulator';
    this.latency = latency;
    this.jitter = jitter;
    this.flashKey = flashKey;
  }

  delay(fn) {
    const ms = this.latency + Math.random() * this.jitter;
    if (ms <= 0) fn();
    else setTimeout(fn, ms);
  }

  async connect() {
    const key = this.flashKey;
    const flash = {
      load: () => {
        try {
          return JSON.parse(localStorage.getItem(key)) || null;
        } catch {
          return null;
        }
      },
      save: (cfg) => {
        try {
          localStorage.setItem(key, JSON.stringify(cfg));
        } catch {}
      },
    };
    this.arm = new VirtualArm({ emit: (m) => this.delay(() => this.onmessage(m)), flash });
    this.arm.boot(performance.now());
    this.timer = setInterval(() => this.arm.loop(performance.now()), 5);
    this.setStatus('online', 'Simulator');
    this.arm.sendAll();
  }

  send(obj) {
    if (!this.arm) return;
    this.delay(() => this.arm?.handle(JSON.parse(JSON.stringify(obj))));
  }

  close() {
    clearInterval(this.timer);
    this.arm?.disconnect();
    this.arm = null;
    this.setStatus('offline');
  }
}

// ------------------------------------------------------------------ Wi-Fi
export class WsTransport extends Base {
  constructor(host) {
    super();
    this.host = host.replace(/^\w+:\/\//, '').replace(/\/.*$/, '');
    this.label = this.host;
    this.wanted = false;
    this.retry = 500;
  }

  url() {
    // accept "192.168.4.1", "roboarm.local", "host:port"
    return /:\d+$/.test(this.host) ? `ws://${this.host}/` : `ws://${this.host}:81/`;
  }

  async connect() {
    this.wanted = true;
    this.open();
  }

  open() {
    this.setStatus('connecting', this.url());
    let ws;
    try {
      ws = new WebSocket(this.url());
    } catch (e) {
      this.setStatus('offline', String(e.message || e));
      return;
    }
    this.ws = ws;
    ws.onopen = () => {
      this.retry = 500;
      this.setStatus('online', this.host);
      this.send({ c: 'get' });
    };
    ws.onmessage = (ev) => {
      try {
        this.onmessage(JSON.parse(ev.data));
      } catch {
        this.onlog(String(ev.data));
      }
    };
    ws.onclose = () => {
      if (this.ws !== ws) return;
      this.ws = null;
      if (this.wanted) {
        this.setStatus('connecting', `reconnecting in ${(this.retry / 1000).toFixed(1)}s`);
        clearTimeout(this.reconnectTimer);
        this.reconnectTimer = setTimeout(() => this.wanted && this.open(), this.retry);
        this.retry = Math.min(this.retry * 1.7, 5000);
      } else {
        this.setStatus('offline');
      }
    };
    ws.onerror = () => {};
  }

  send(obj) {
    if (this.ws?.readyState === WebSocket.OPEN) this.ws.send(JSON.stringify(obj));
  }

  close() {
    this.wanted = false;
    clearTimeout(this.reconnectTimer);
    const ws = this.ws;
    this.ws = null;
    ws?.close();
    this.setStatus('offline');
  }
}

// ------------------------------------------------------------------ USB serial
export class SerialTransport extends Base {
  static supported() {
    return 'serial' in navigator;
  }

  constructor(baud = 115200) {
    super();
    this.baud = baud;
    this.label = 'USB';
  }

  async connect() {
    if (!SerialTransport.supported()) {
      this.setStatus('offline', 'Web Serial needs Chrome or Edge');
      return;
    }
    this.setStatus('connecting', 'choose the port');
    try {
      this.port = await navigator.serial.requestPort();
      await this.port.open({ baudRate: this.baud });
    } catch (e) {
      this.setStatus('offline', e.message || String(e));
      return;
    }
    this.encoder = new TextEncoder();
    this.writer = this.port.writable.getWriter();
    this.alive = true;
    this.setStatus('online', 'USB serial');
    this.readLoop();
    // opening the port can reset the board — ask again once it has booted
    this.send({ c: 'get' });
    setTimeout(() => this.alive && this.send({ c: 'get' }), 1800);
  }

  async readLoop() {
    const decoder = new TextDecoder();
    let buf = '';
    while (this.alive && this.port?.readable) {
      this.reader = this.port.readable.getReader();
      try {
        for (;;) {
          const { value, done } = await this.reader.read();
          if (done) break;
          buf += decoder.decode(value, { stream: true });
          let nl;
          while ((nl = buf.indexOf('\n')) >= 0) {
            const line = buf.slice(0, nl).replace(/\r$/, '');
            buf = buf.slice(nl + 1);
            if (!line) continue;
            if (line[0] === '{') {
              try {
                this.onmessage(JSON.parse(line));
                continue;
              } catch {}
            }
            this.onlog(line);
          }
        }
      } catch (e) {
        this.onlog(`serial: ${e.message || e}`);
      } finally {
        this.reader.releaseLock();
      }
      if (!this.alive) break;
    }
    if (this.alive) this.close();
  }

  async send(obj) {
    if (!this.writer) return;
    try {
      await this.writer.write(this.encoder.encode(JSON.stringify(obj) + '\n'));
    } catch (e) {
      this.onlog(`serial write: ${e.message || e}`);
    }
  }

  async close() {
    this.alive = false;
    try {
      await this.reader?.cancel();
    } catch {}
    try {
      this.writer?.releaseLock();
    } catch {}
    try {
      await this.port?.close();
    } catch {}
    this.writer = null;
    this.port = null;
    this.setStatus('offline');
  }
}
