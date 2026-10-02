// Transport + protocol layer. See Main/Tuning.ino for the firmware side of the protocol.
//
//   Link  = anything with { open(), send(line), close(), onLine, onClose, name }
//           SerialLink (Web Serial, real robot) and DemoLink (sim.js) both implement it.
//   Robot = the protocol on top of a Link: handshake, liveness pings, parameter table,
//           typed events, and promise-style requests.
(function () {
  'use strict';

  class SerialLink {
    static get supported() { return 'serial' in navigator; }

    constructor() {
      this.name = 'USB serial';
      this.onLine = () => {};
      this.onClose = () => {};
      this.port = null;
      this._queue = Promise.resolve();
      this._closing = false;
    }

    async open() {
      this.port = await navigator.serial.requestPort(); // throws if the user cancels the picker
      await this.port.open({ baudRate: 115200 });
      try { await this.port.setSignals({ dataTerminalReady: true, requestToSend: true }); } catch (e) { /* not fatal */ }
      this.writer = this.port.writable.getWriter();
      this._enc = new TextEncoder();
      this._readLoop();
    }

    async _readLoop() {
      const dec = new TextDecoder();
      let buf = '';
      try {
        while (this.port.readable && !this._closing) {
          this.reader = this.port.readable.getReader();
          try {
            for (;;) {
              const { value, done } = await this.reader.read();
              if (done) break;
              buf += dec.decode(value, { stream: true });
              let i;
              while ((i = buf.indexOf('\n')) >= 0) {
                const line = buf.slice(0, i).replace(/\r$/, '');
                buf = buf.slice(i + 1);
                if (line.length) this.onLine(line);
              }
              if (buf.length > 16384) buf = ''; // runaway line without a newline
            }
          } finally {
            this.reader.releaseLock();
          }
        }
      } catch (e) { /* device unplugged or port closed */ }
      if (!this._closing) this.onClose();
    }

    send(line) {
      if (!this.writer) return;
      this._queue = this._queue
        .then(() => this.writer.write(this._enc.encode(line + '\n')))
        .catch(() => {});
    }

    async close() {
      this._closing = true;
      try { await this._queue; } catch (e) { /* ignore */ }
      try { await this.reader?.cancel(); } catch (e) { /* ignore */ }
      try { this.writer?.releaseLock(); } catch (e) { /* ignore */ }
      try { await this.port?.close(); } catch (e) { /* ignore */ }
      this.onClose();
    }
  }

  class Robot {
    constructor(link) {
      this.link = link;
      this.handlers = {};
      this.params = new Map(); // name -> {n,g,desc,ty,v,d,lo,hi,st}
      this.state = 'handshake'; // handshake | ready | exited | closed
      this.lastRx = Date.now();
      this.waiters = [];
      this.fw = '';
      link.onLine = (l) => this._line(l);
      link.onClose = () => this._closed();
    }

    on(evt, fn) { (this.handlers[evt] = this.handlers[evt] || []).push(fn); return this; }
    emit(evt, ...a) { for (const f of this.handlers[evt] || []) { try { f(...a); } catch (e) { console.error(e); } } }

    start() {
      this.send('TUNE');
      this._hs = setInterval(() => { if (this.state === 'handshake') this.send('TUNE', true); }, 500);
      this._ping = setInterval(() => {
        if (this.state === 'ready') {
          this.send('PING', true);
          if (Date.now() - this.lastRx > 3000 && !this._stale) { this._stale = true; this.emit('stale', true); }
        }
      }, 500);
    }

    // Start (or restart) the handshake, e.g. after the robot left tuning mode.
    handshake() {
      this.state = 'handshake';
      this._stale = false;
      this.emit('state', this.state);
      this.send('TUNE');
    }

    send(line, quiet) {
      this.link.send(line);
      this.emit('log', 'tx', line, !!quiet);
    }

    get(name) { const p = this.params.get(name); return p ? p.v : undefined; }
    set(name, value) {
      const p = this.params.get(name);
      const v = p && p.ty === 'i' ? Math.round(value) : value;
      this.send('SET ' + name + ' ' + String(+Number(v).toFixed(9)));
    }

    // Send a command and resolve with the first reply matching `type` (and optional `match`),
    // or reject on an {"t":"err","c":<cmd>} or timeout.
    request(cmd, type, { match, timeout = 4000 } = {}) {
      return new Promise((resolve, reject) => {
        const w = { type, match, cmd: cmd.split(' ')[0].toUpperCase(), resolve, reject };
        w.timer = setTimeout(() => { this.waiters = this.waiters.filter((x) => x !== w); reject(new Error('timeout waiting for ' + type)); }, timeout);
        this.waiters.push(w);
        this.send(cmd);
      });
    }

    _line(line) {
      this.lastRx = Date.now();
      if (this._stale) { this._stale = false; this.emit('stale', false); }
      if (line[0] !== '{') { this.emit('log', 'dbg', line, false); return; }
      let o;
      try { o = JSON.parse(line); } catch (e) { this.emit('log', 'dbg', line, false); return; }
      this.emit('log', 'rx', line, o.t === 's' || o.t === 'pong' || o.t === 'tr');
      this._msg(o);
    }

    _msg(o) {
      // resolve waiters
      for (const w of [...this.waiters]) {
        const isErr = o.t === 'err' && String(o.c).toUpperCase() === w.cmd;
        if (isErr || (o.t === w.type && (!w.match || w.match(o)))) {
          clearTimeout(w.timer);
          this.waiters = this.waiters.filter((x) => x !== w);
          if (isErr) w.reject(new Error(o.m)); else w.resolve(o);
        }
      }
      switch (o.t) {
        case 'hello':
          this.fw = o.fw;
          if (this.state !== 'ready') {
            this.state = 'ready';
            this.emit('state', this.state);
            this.send('PARAMS');
          }
          break;
        case 'param': this.params.set(o.n, o); break;
        case 'params_end': this.emit('params'); break;
        case 'set': {
          const p = this.params.get(o.n);
          if (p) p.v = o.v;
          this.emit('param', o.n, o.v);
          break;
        }
        case 's': this.emit('telemetry', o); break;
        case 'test': this.emit('test', o); break;
        case 'tr': this.emit('tr', o); break;
        case 'test_done': this.emit('test_done', o); break;
        case 'err': this.emit('error', o); break;
        case 'exit':
          this.state = 'exited';
          this.emit('state', this.state, o.why);
          break;
        default: this.emit(o.t, o);
      }
    }

    _closed() {
      if (this.state === 'closed') return;
      this.state = 'closed';
      clearInterval(this._hs);
      clearInterval(this._ping);
      for (const w of this.waiters) { clearTimeout(w.timer); w.reject(new Error('disconnected')); }
      this.waiters = [];
      this.emit('state', this.state);
    }

    async close() {
      try { this.send('STOP', true); this.send('STREAM 0', true); } catch (e) { /* ignore */ }
      await this.link.close();
      this._closed();
    }
  }

  window.SerialLink = SerialLink;
  window.Robot = Robot;
})();
