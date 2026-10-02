// Demo robot: speaks the same protocol as Main/Tuning.ino with simple made-up physics, so the page
// can be tried (and tested) without hardware. Nothing here reflects how the real robot behaves.
(function () {
  'use strict';

  const KD = 'Derivative term; per microsecond, so useful values are large.';
  const PIDS = [
    ['center', 'PID - wall follower (fwd)', [2, 0, 0.5], [20, 1, 1e6], [0.1, 0.001, 100], 'Steers to keep the right-wall gap.'],
    ['scale', 'PID - distance scale (fwd)', [0.0045, 0, 0.0008], [0.05, 0.01, 1e5], [0.0001, 0.00001, 10], 'Slows the robot near the target encoder count.'],
    ['gyro', 'PID - heading hold (gyro)', [1, 0.001, 0.03], [20, 0.1, 1e6], [0.1, 0.0005, 100], 'Currently unused by fwd().'],
    ['climb', 'PID - ramp climb', [2, 0, 0.1], [20, 1, 1e6], [0.1, 0.001, 100], 'Heading hold on a ramp.'],
    ['turn', 'PID - turn (absoluteturn)', [4.5, 0, 0.3], [20, 1, 1e6], [0.1, 0.001, 100], 'Turn speed from heading error.'],
    ['par', 'PID - obstacle parallel', [1, 0, 0.1], [20, 1, 1e6], [0.1, 0.001, 100], 'Squares up to the wall.'],
    ['wig', 'PID - obstacle wiggle', [8, 0, 0.1], [50, 1, 1e6], [0.5, 0.001, 100], 'Side-sensor balance.'],
  ];
  const POS = ['', 'front right', 'right front', 'right back', 'back', 'left back', 'left front', 'front left'];

  function buildParams() {
    const out = [];
    for (const [id, group, def, hi, st, desc] of PIDS) {
      ['kp', 'ki', 'kd'].forEach((k, i) => out.push({
        n: id + '.' + k, g: group, desc: k === 'kd' ? KD : desc, ty: 'f', v: def[i], d: def[i], lo: 0, hi: hi[i], st: st[i],
      }));
    }
    out.push({ n: 'col.black', g: 'Color thresholds', desc: 'Black if clear/baseline is below this.', ty: 'f', v: 0.1, d: 0.1, lo: 0, hi: 1, st: 0.005 });
    out.push({ n: 'col.silver', g: 'Color thresholds', desc: 'Silver if red is above this.', ty: 'i', v: 800, d: 800, lo: 0, hi: 65535, st: 10 });
    out.push({ n: 'col.white', g: 'Color thresholds', desc: 'White if clear/baseline is above this.', ty: 'f', v: 0.85, d: 0.85, lo: 0, hi: 2, st: 0.005 });
    out.push({ n: 'col.clear', g: 'Color thresholds', desc: 'Baseline clear value.', ty: 'f', v: 1000, d: 1000, lo: 1, hi: 65535, st: 10 });
    for (let i = 1; i <= 7; i++) out.push({ n: 'off.' + i, g: 'Distance offsets (mm)', desc: 'Sensor ' + i + ' (' + POS[i] + ')', ty: 'i', v: 0, d: 0, lo: -200, hi: 200, st: 1 });
    return out;
  }

  // same maths as Main/PID.cpp (derivative per microsecond, integral clamped)
  class PID {
    constructor(kp, ki, kd, tUs) { this.kp = kp; this.ki = ki; this.kd = kd; this.prev = 0; this.cum = 0; this.prevT = tUs; }
    get(e, tUs) {
      const dt = (tUs - this.prevT) || 1;
      const delta = (e - this.prev) / dt;
      this.cum = Math.max(-3000, Math.min(3000, this.cum + e));
      this.prevT = tUs; this.prev = e;
      return this.kp * e + this.ki * this.cum + this.kd * delta;
    }
  }

  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const wrap180 = (a) => { while (a > 180) a -= 360; while (a < -180) a += 360; return a; };
  const noise = (s) => (Math.random() - 0.5) * 2 * s;
  const TILES = {
    white: [330, 340, 320, 1000], black: [20, 20, 18, 60], silver: [900, 900, 880, 2800],
    blue: [100, 200, 420, 700], red: [420, 110, 90, 620],
  };
  const TRUE_MM = [0, 420, 82, 78, 140, 90, 85, 415]; // what each sensor "sees" in the demo corridor
  const BIAS = [0, 6, -4, 9, 3, -5, 7, 2];            // hidden per-sensor error for the calibration demo

  class DemoRobot {
    constructor(emit) {
      this.emit = emit;
      this.p = buildParams();
      this.byName = new Map(this.p.map((x) => [x.n, x]));
      this.tuning = false;
      this.hdg = 10; this.enc = [0, 0, 0];
      this.motor = { A: 0, B: 0, C: 0, D: 0 };
      this.drive = null; this.lastMotion = 0;
      this.streamMs = 0; this.mask = ''; this.lastStream = 0;
      this.test = null;
      this.tile = 'white';
      this.t0 = performance.now();
      this.timer = setInterval(() => this.tick(), 15);
      setTimeout(() => this.emit('reading walls'), 50);
      setTimeout(() => this.emit('pause'), 400);
    }
    stop() { clearInterval(this.timer); }
    now() { return performance.now() - this.t0; }
    j(o) { this.emit(JSON.stringify(o)); }
    val(n) { return this.byName.get(n).v; }

    handle(line) {
      const a = line.trim().split(/\s+/);
      const c = (a[0] || '').toUpperCase();
      if (!c) return;
      if (!this.tuning) { if (c === 'TUNE') { this.tuning = true; this.j({ t: 'hello', fw: 'demo-1', ms: Math.round(this.now()), nparams: this.p.length }); } return; }
      const busy = !!this.test;
      if (c === 'PING') return this.j({ t: 'pong', ms: Math.round(this.now()) });
      if (c === 'STOP') { this.stopMotion(); this.endTest('stopped'); return this.j({ t: 'ok', c: 'STOP' }); }
      if (c === 'EXIT') { this.stopMotion(); this.endTest('exit'); this.tuning = false; this.mask = ''; return this.j({ t: 'exit', why: 'exit' }); }
      if (c === 'TUNE') return this.j({ t: 'hello', fw: 'demo-1', ms: Math.round(this.now()), nparams: this.p.length });
      if (c === 'SET') {
        const v = Number(a[2]);
        const p = this.byName.get(a[1]);
        if (a.length < 3 || !Number.isFinite(v)) return this.j({ t: 'err', c: 'SET', m: 'usage: SET <name> <number>' });
        if (!p) return this.j({ t: 'err', c: 'SET', m: 'unknown parameter' });
        if (v < p.lo || v > p.hi) return this.j({ t: 'err', c: 'SET', m: 'value out of range' });
        p.v = p.ty === 'i' ? Math.round(v) : v;
        return this.j({ t: 'set', n: p.n, v: p.v });
      }
      if (busy) return this.j({ t: 'err', c: a[0], m: 'busy: send STOP first' });
      if (c === 'PARAMS') { this.p.forEach((p) => this.j(Object.assign({ t: 'param' }, p))); return this.j({ t: 'params_end', n: this.p.length }); }
      if (c === 'STREAM') {
        const hz = Number(a[1]);
        if (!(hz >= 0 && hz <= 30)) return this.j({ t: 'err', c: 'STREAM', m: 'usage: STREAM <0..30> [dgce]' });
        this.streamMs = hz ? 1000 / hz : 0; this.mask = hz ? (a[2] || 'dgce') : '';
        return this.j({ t: 'ok', c: 'STREAM' });
      }
      if (c === 'MOT') {
        const id = (a[1] || '').toUpperCase(), s = Number(a[2]);
        if (!/^[A-D]$/.test(id) || !(s >= -255 && s <= 255)) return this.j({ t: 'err', c: 'MOT', m: 'usage: MOT <A|B|C|D> <-255..255>' });
        this.motor[id] = s; this.drive = null; this.lastMotion = this.now(); return;
      }
      if (c === 'DRIVE') {
        const d = (a[1] || '').toLowerCase(), s = Number(a[2]);
        if (!(s >= 0 && s <= 255)) return this.j({ t: 'err', c: 'DRIVE', m: 'usage: DRIVE <fw|bk|tl|tr> <0..255>' });
        if (!['fw', 'bk', 'tl', 'tr'].includes(d)) return this.j({ t: 'err', c: 'DRIVE', m: 'direction must be fw, bk, tl or tr' });
        this.drive = { d, s }; this.lastMotion = this.now(); return;
      }
      if (c === 'ENCRESET') { this.enc = [0, 0, 0]; return this.j({ t: 'ok', c: 'ENCRESET' }); }
      if (c === 'CALDIST') {
        const n = Number(a[1]), mm = Number(a[2]);
        if (!(n >= 1 && n <= 7 && mm >= 20 && mm <= 1000)) return this.j({ t: 'err', c: 'CALDIST', m: 'usage: CALDIST <1..7> <20..1000 mm>' });
        // pretend to take 100 samples over ~1 s
        return setTimeout(() => {
          const mean = mm + BIAS[n] + noise(0.2);
          this.j({ t: 'caldist', n, true: mm, valid: 100, samples: 100, mean: +mean.toFixed(2), min: Math.floor(mean - 2), max: Math.ceil(mean + 2), offset: Math.round(mean - mm), current: this.val('off.' + n) });
        }, 900);
      }
      if (c === 'CLEARREF') {
        const clear = TILES[this.tile][3] * (1 + noise(0.01));
        this.byName.get('col.clear').v = +clear.toFixed(1);
        return setTimeout(() => this.j({ t: 'clearref', clear: this.val('col.clear') }), 500);
      }
      if (c === 'TEST') return this.startTest(a);
      this.j({ t: 'err', c: a[0], m: 'unknown command' });
    }

    stopMotion() { this.drive = null; for (const k in this.motor) this.motor[k] = 0; }

    startTest(a) {
      const kind = (a[1] || '').toUpperCase();
      if (kind === 'TURN') {
        const deg = Number(a[2]);
        if (!(Math.abs(deg) >= 2 && Math.abs(deg) <= 360)) return this.j({ t: 'err', c: 'TEST', m: 'usage: TEST TURN <2..360 deg, - for left>' });
        const start = this.hdg, target = (start + deg + 720) % 360;
        const diff = wrap180(target - start);
        this.test = { kind: 'turn', start, target, right: diff > 0, limit: 2000 * Math.abs(diff) / 90, t0: this.now(), w: 0, pid: new PID(this.val('turn.kp'), this.val('turn.ki'), this.val('turn.kd'), this.now() * 1000), lastTr: -99 };
        return this.j({ t: 'test', name: 'turn', cols: ['time_ms', 'error_deg', 'pid_out', 'speed', 'heading'], info: 'start ' + Math.round(start) + ' deg, target ' + Math.round(target) + ' deg, limit ' + Math.round(this.test.limit) + ' ms' });
      }
      if (kind === 'FWD') {
        const mm = Number(a[2]);
        if (!(mm >= 50 && mm <= 1200)) return this.j({ t: 'err', c: 'TEST', m: 'usage: TEST FWD <50..1200 mm> [wall|gyro] [realtiming 0|1]' });
        const gyro = (a[3] || '').toLowerCase() === 'gyro';
        const pulses = mm / (80 * Math.PI) * 5 * 195;
        this.test = { kind: 'fwd', gyro, mm, pulses, t0: this.now(), x: 0, y: 25, th: 0, v: 0, lastTr: -99, c: new PID(0, 0, 0, this.now() * 1000), g: new PID(0, 0, 0, this.now() * 1000), s: new PID(0, 0, 0, this.now() * 1000) };
        return this.j({ t: 'test', name: 'fwd', cols: ['time_ms', 'error', 'pid_out', 'scale', 'speed_AC', 'speed_BD', 'mm'], info: mm + ' mm, steer by ' + (gyro ? 'gyro' : 'wall') + ', ' + Math.round(pulses) + ' pulses' });
      }
      this.j({ t: 'err', c: 'TEST', m: 'usage: TEST TURN <deg> | TEST FWD <mm> [wall|gyro] [realtiming 0|1]' });
    }

    endTest(why) {
      const T = this.test;
      if (!T) return;
      this.test = null;
      const ms = Math.round(this.now() - T.t0);
      if (T.kind === 'turn') this.j({ t: 'test_done', name: 'turn', why, final_err: +wrap180(T.target - this.hdg).toFixed(2), extra: 0, ms });
      else this.j({ t: 'test_done', name: 'fwd', why, final_err: +(T.x - T.mm).toFixed(2), extra: +T.x.toFixed(1), ms });
    }

    stepTest(dt) {
      const T = this.test, now = this.now(), tUs = now * 1000, el = now - T.t0;
      if (T.kind === 'turn') {
        if (el > T.limit) return this.endTest('limit');
        T.pid.kp = this.val('turn.kp'); T.pid.ki = this.val('turn.ki'); T.pid.kd = this.val('turn.kd');
        const d = wrap180(T.target - this.hdg);
        const out = T.pid.get(Math.abs(d), tUs);
        const spd = Math.round(clamp(out, 20, 150));
        const rate = spd < 35 ? 0 : (spd - 30) * 1.3;           // deg/s the motors would like to give
        T.w += (rate * (T.right ? 1 : -1) - T.w) * Math.min(1, dt / 0.12); // motor lag
        this.hdg = (this.hdg + T.w * dt + 720) % 360;
        if (el - T.lastTr >= 15) { T.lastTr = el; this.j({ t: 'tr', v: [Math.round(el), +d.toFixed(3), +out.toFixed(3), spd, +this.hdg.toFixed(3)] }); }
        return;
      }
      if (el > 15000) return this.endTest('timeout');
      const enc = T.x / (80 * Math.PI) * 5 * 195;
      if (enc > T.pulses) return this.endTest('distance');
      T.c.kp = this.val('center.kp'); T.c.ki = this.val('center.ki'); T.c.kd = this.val('center.kd');
      T.g.kp = this.val('gyro.kp'); T.g.ki = this.val('gyro.ki'); T.g.kd = this.val('gyro.kd');
      T.s.kp = this.val('scale.kp'); T.s.ki = this.val('scale.ki'); T.s.kd = this.val('scale.kd');
      const err = T.gyro ? T.th * 57.3 : T.y + 300 * T.th;
      const adj = (T.gyro ? T.g : T.c).get(err, tUs);
      const scale = T.s.get(T.pulses - enc, tUs);
      if (scale * 120 < 25) return this.endTest('scale-exit');
      const ac = clamp(scale * (120 - adj), 20, 150), bd = clamp(scale * (120 + adj), 20, 150);
      const v = ((ac < 22 ? 0 : ac) + (bd < 22 ? 0 : bd)) / 2 * 0.9;  // mm/s (motors don't move below ~22)
      T.th += (bd - ac) * -0.004 * dt;
      T.y += v * T.th * dt;
      T.x += v * dt;
      if (el - T.lastTr >= 15) { T.lastTr = el; this.j({ t: 'tr', v: [Math.round(el), +err.toFixed(3), +adj.toFixed(3), +scale.toFixed(3), +ac.toFixed(3), +bd.toFixed(3), +T.x.toFixed(3)] }); }
    }

    telemetry() {
      const o = { t: 's', ms: Math.round(this.now()) };
      if (this.mask.includes('d')) {
        const r = [], d = [];
        for (let i = 1; i <= 7; i++) {
          const raw = i === 4 && Math.random() < 0.03 ? -1 : Math.round(TRUE_MM[i] + BIAS[i] + noise(1.6));
          r.push(raw); d.push(raw === -1 ? -1 : Math.max(0, raw - this.val('off.' + i)));
        }
        o.r = r; o.d = d;
      }
      if (this.mask.includes('g')) { o.hdg = +this.hdg.toFixed(1); o.rol = +noise(0.6).toFixed(1); o.pit = +noise(0.6).toFixed(1); o.cal = [3, 3, 2, 3]; }
      if (this.mask.includes('c')) {
        const base = TILES[this.tile];
        const c = base.map((x) => Math.round(x * (1 + noise(0.03))));
        const clear = this.val('col.clear');
        o.col = c; o.ratio = +(c[3] / clear).toFixed(3);
        let cls = 0;
        if (o.ratio < this.val('col.black')) cls = -1;
        else if (c[0] > this.val('col.silver')) cls = 3;
        else if (o.ratio > this.val('col.white')) cls = 0;
        else if (c[2] > c[1] + 10 && c[2] > c[0] + 10) cls = 1;
        else if (c[0] > c[1] + 10 && c[0] > c[2] + 10) cls = 2;
        o.cls = cls;
      }
      if (this.mask.includes('e')) o.enc = this.enc.map(Math.round);
      this.j(o);
    }

    tick() {
      const dt = 0.015, now = this.now();
      if (this.tuning && !this.test && (this.drive || Object.values(this.motor).some((v) => v)) && now - this.lastMotion > 400) this.stopMotion(); // deadman
      if (this.test) this.stepTest(dt);
      else if (this.drive) {
        const { d, s } = this.drive, sp = s < 35 ? 0 : s;
        if (d === 'fw' || d === 'bk') { const k = (d === 'fw' ? 1 : -1) * sp * 4 * dt; this.enc = this.enc.map((e) => e + k); }
        else this.hdg = (this.hdg + (d === 'tr' ? 1 : -1) * sp * 0.9 * dt + 360) % 360;
      } else {
        this.enc[0] += this.motor.A * 4 * dt; this.enc[1] += this.motor.B * 4 * dt; this.enc[2] += -this.motor.D * -4 * dt;
      }
      if (this.tuning && !this.test && this.streamMs && now - this.lastStream >= this.streamMs) { this.lastStream = now; this.telemetry(); }
    }
  }

  class DemoLink {
    constructor() {
      this.name = 'Demo robot';
      this.onLine = () => {};
      this.onClose = () => {};
      this.sim = null;
    }
    async open() { this.sim = new DemoRobot((l) => setTimeout(() => this.onLine(l), 0)); }
    send(line) { const s = this.sim; if (s) setTimeout(() => { if (this.sim === s) s.handle(line); }, 0); }
    async close() { if (this.sim) this.sim.stop(); this.sim = null; this.onClose(); }
    setTile(t) { if (this.sim) this.sim.tile = t; }
  }

  window.DemoLink = DemoLink;
})();
