// Theseus Tuner UI. Talks to Main/Tuning.ino through protocol.js (or to the demo robot in sim.js).
(function () {
  'use strict';

  const $ = (s, r = document) => r.querySelector(s);
  const $$ = (s, r = document) => Array.from(r.querySelectorAll(s));

  function h(tag, props, ...kids) {
    const e = document.createElement(tag);
    for (const [k, v] of Object.entries(props || {})) {
      if (k === 'class') e.className = v;
      else if (k === 'text') e.textContent = v;
      else if (k.startsWith('on')) e.addEventListener(k.slice(2).toLowerCase(), v);
      else if (v === true) e.setAttribute(k, '');
      else if (v !== false && v != null) e.setAttribute(k, v);
    }
    for (const c of kids.flat()) if (c != null) e.append(c.nodeType ? c : String(c));
    return e;
  }
  const SVGNS = 'http://www.w3.org/2000/svg';
  function sv(tag, attrs, text) {
    const e = document.createElementNS(SVGNS, tag);
    for (const [k, v] of Object.entries(attrs || {})) e.setAttribute(k, v);
    if (text !== undefined) e.textContent = text;
    return e;
  }
  const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
  const fmt = (v, d = 1) => (v === null || v === undefined || Number.isNaN(v) ? '–' : Number(v).toFixed(d));
  // short number: up to 4 significant digits, no trailing zeros, no exponent for sane values
  const short = (v) => (v === 0 ? '0' : String(Number(Number(v).toPrecision(4))));

  // ---- constants copied from Main.ino / Distance.ino (used only for the derived read-outs) ----
  const TARGET_SIDE_GAP_MM = 80;     // (TILE_MM - ROBOT_WIDTH_MM) / 2
  const SIDE_WALL_MAX_MM = 200;
  const TARGET_GAP_MM = 52.5;        // (TILE_MM - ROBOT_LENGTH_MM) / 2
  const WHEEL_MM_PER_COUNT = (80 * Math.PI) / (5 * 195); // wheel_diameter * pi / (wheel_cpr * gear_ratio)
  const POS = ['', 'front right', 'right front', 'right back', 'back', 'left back', 'left front', 'front left'];
  const CLASS_NAME = { '-1': 'Black', '0': 'White', '1': 'Blue', '2': 'Red', '3': 'Silver' };
  const CLASS_KEYS = ['white', 'black', 'silver', 'blue', 'red'];

  const S = {
    robot: null, link: null, demo: false,
    tel: null, telT0: 0,
    cal: { dist: {}, color: {}, sampling: null },
    testKind: 'turn', test: null, pending: null, runs: [], ghost: null, runSeq: 0,
    driveSpeed: 120,
    log: [],
    bindings: new Map(), // param name -> [fn(value)] refreshers
  };
  const ready = () => S.robot && S.robot.state === 'ready';

  // ============================== small UI helpers ==============================
  function toast(msg, bad) {
    const t = h('div', { class: 'toast' + (bad ? ' bad' : ''), text: msg });
    $('#toasts').append(t);
    setTimeout(() => t.remove(), bad ? 6000 : 3000);
  }
  function banner(html, kind, actions) {
    const b = $('#banner');
    b.className = 'banner' + (kind ? ' ' + kind : '');
    b.replaceChildren(h('span', { text: html }));
    for (const a of actions || []) b.append(h('button', { class: 'btn small', text: a.label, onclick: () => { a.fn(); } }));
    b.hidden = false;
  }
  const hideBanner = () => { $('#banner').hidden = true; };
  function setKV(el, pairs) {
    el.replaceChildren(...pairs.flatMap(([k, v]) => [h('span', { text: k }), h('span', { text: v })]));
  }
  function bind(name, fn, scope) {
    if (!S.bindings.has(name)) S.bindings.set(name, []);
    S.bindings.get(name).push({ fn, scope: scope || 'misc' });
  }
  function unbind(scope) {
    for (const [k, arr] of Array.from(S.bindings)) {
      const keep = arr.filter((b) => b.scope !== scope);
      if (keep.length) S.bindings.set(k, keep); else S.bindings.delete(k);
    }
  }
  function unbindAll() { S.bindings.clear(); }
  function refreshBindings(name) {
    const v = S.robot.get(name);
    for (const b of S.bindings.get(name) || []) b.fn(v);
  }
  function send(cmd, quiet) { if (ready()) S.robot.send(cmd, quiet); }

  // ============================== tabs ==============================
  let activeTab = 'dashboard';
  function showTab(name) {
    activeTab = name;
    $$('#tabs button').forEach((b) => b.classList.toggle('active', b.dataset.tab === name));
    $$('.tab').forEach((t) => t.classList.toggle('active', t.id === 'tab-' + name));
    if (name !== 'motors') Drive.releaseAll();
    if (name === 'log') renderLog();
    if (name === 'params') renderCpp();
    window.dispatchEvent(new Event('resize')); // charts re-measure now that they are visible
  }
  $('#tabs').addEventListener('click', (e) => { const b = e.target.closest('button[data-tab]'); if (b) showTab(b.dataset.tab); });

  // ============================== connection ==============================
  async function connect(useDemo) {
    if (S.robot) return;
    if (!useDemo && !SerialLink.supported) {
      banner('This browser has no Web Serial. Use Chrome or Edge on a desktop/laptop (opened from localhost, https, or a local file). The Demo button still works anywhere.', 'bad');
      return;
    }
    const link = useDemo ? new DemoLink() : new SerialLink();
    try { await link.open(); } catch (e) {
      if (e && e.name !== 'NotFoundError') toast('Could not open the port: ' + (e.message || e), true);
      return;
    }
    S.link = link; S.demo = useDemo;
    const r = new Robot(link);
    S.robot = r;
    wire(r);
    $('#demo-panel').hidden = !useDemo;
    updateTop();
    r.start();
    S.handshakeTimer = setTimeout(() => {
      if (S.robot === r && r.state === 'handshake') {
        banner('Robot not answering yet. Flip the pause switch to PAUSE (the robot only enters tuning mode while paused) and make sure the board is powered and this is the Giga\'s USB port.', '');
      }
    }, 4000);
  }

  function wire(r) {
    r.on('state', (st, why) => {
      updateTop();
      if (st === 'ready') {
        hideBanner();
        clearTimeout(S.handshakeTimer);
        r.send('STREAM 8 dgce');
        toast('Connected: robot in tuning mode (' + r.fw + ')');
      } else if (st === 'exited') {
        Drive.releaseAll();
        banner('Robot left tuning mode (' + (why === 'switch' ? 'pause switch flipped to RUN' : why) + '). Flip the switch back to PAUSE, then:', '', [{ label: 'Enter tuning mode again', fn: () => { hideBanner(); r.handshake(); } }]);
      } else if (st === 'closed') {
        Drive.releaseAll();
        const lost = S.robot === r;
        S.robot = null; S.link = null; S.tel = null;
        unbindAll();
        $('#demo-panel').hidden = true;
        updateTop();
        clearUiForDisconnect();
        if (lost && !S.closing) banner('Connection lost.', 'bad'); else hideBanner();
      }
    });
    r.on('stale', (stale) => { if (stale) banner('No data from the robot for 3 s: check the cable and the pause switch.', 'bad'); else hideBanner(); });
    r.on('params', () => { buildParamUi(); offerSavedProfile(); });
    r.on('param', (n) => { refreshBindings(n); if (S.test && PID_GROUPS.includes(GROUP_OF(n))) S.test.changed = true; });
    r.on('telemetry', onTelemetry);
    r.on('test', onTestStart);
    r.on('tr', onTrace);
    r.on('test_done', onTestDone);
    r.on('error', (o) => { toast(o.c + ': ' + o.m, true); if (S.pending && o.c === 'TEST') { S.pending = null; setTestRunning(false); } });
    r.on('log', (dir, text, noisy) => appendLog(dir, text, noisy));
  }

  function updateTop() {
    const r = S.robot, st = r ? r.state : 'none';
    const pill = $('#status');
    const map = {
      none: ['Disconnected', 'pill-off'],
      handshake: ['Waiting for robot… flip pause switch to PAUSE', 'pill-wait'],
      ready: [(S.demo ? 'DEMO · ' : '') + 'Tuning mode · ' + (r && r.fw), 'pill-ok'],
      exited: ['Robot left tuning mode', 'pill-wait'],
      closed: ['Disconnected', 'pill-off'],
    };
    pill.textContent = map[st][0];
    pill.className = 'pill ' + map[st][1];
    $('#btn-connect').hidden = !!r; $('#btn-demo').hidden = !!r;
    $('#btn-disconnect').hidden = !r; $('#btn-exit').hidden = !(r && st === 'ready');
    $$('[data-needs-robot]').forEach((b) => { b.disabled = !(st === 'ready'); });
  }

  $('#btn-connect').onclick = () => connect(false);
  $('#btn-demo').onclick = () => connect(true);
  $('#btn-disconnect').onclick = async () => { if (!S.robot) return; S.closing = true; await S.robot.close(); S.closing = false; };
  $('#btn-exit').onclick = () => send('EXIT');
  $('#demo-tile').onchange = (e) => S.link && S.link.setTile && S.link.setTile(e.target.value);

  // ============================== emergency stop & drive keep-alive ==============================
  const Drive = {
    held: new Map(), timer: 0,
    hold(key, cmdFn) {
      if (!ready()) return;
      this.held.set(key, cmdFn);
      send(cmdFn(), true);
      if (!this.timer) this.timer = setInterval(() => { for (const f of this.held.values()) send(f(), true); }, 100);
      $$('[data-hold="' + key + '"]').forEach((b) => b.classList.add('held'));
    },
    release(key, stopCmd) {
      if (!this.held.delete(key)) return;
      if (stopCmd) send(stopCmd, true);
      if (!this.held.size) { clearInterval(this.timer); this.timer = 0; }
      $$('[data-hold="' + key + '"]').forEach((b) => b.classList.remove('held'));
    },
    releaseAll() {
      const had = this.held.size > 0;
      this.held.clear(); clearInterval(this.timer); this.timer = 0;
      $$('.held').forEach((b) => b.classList.remove('held'));
      if (had && ready()) send('STOP', true);
    },
  };
  function emergencyStop() { Drive.releaseAll(); send('STOP'); }
  $('#btn-stop').onclick = emergencyStop;
  $('#pad-stop').onclick = emergencyStop;
  $('#btn-test-stop').onclick = emergencyStop;
  window.addEventListener('blur', () => Drive.releaseAll());
  document.addEventListener('visibilitychange', () => { if (document.hidden) Drive.releaseAll(); });

  const typing = (e) => /^(INPUT|TEXTAREA|SELECT)$/.test(e.target.tagName);
  const KEYDIR = { ArrowUp: 'fw', w: 'fw', W: 'fw', ArrowDown: 'bk', s: 'bk', S: 'bk', ArrowLeft: 'tl', a: 'tl', A: 'tl', ArrowRight: 'tr', d: 'tr', D: 'tr' };
  document.addEventListener('keydown', (e) => {
    if (typing(e)) return;
    if (e.key === ' ') { e.preventDefault(); emergencyStop(); return; }
    if (activeTab === 'motors' && KEYDIR[e.key] && !e.repeat && !e.ctrlKey && !e.metaKey) {
      e.preventDefault();
      const d = KEYDIR[e.key];
      Drive.hold('dir:' + d, () => 'DRIVE ' + d + ' ' + S.driveSpeed);
    }
  });
  document.addEventListener('keyup', (e) => {
    if (e.key === ' ' && !typing(e)) e.preventDefault(); // a focused button would otherwise be clicked again on Space-up
    const d = KEYDIR[e.key];
    if (d) Drive.release('dir:' + d, 'STOP');
  });

  // ============================== dashboard ==============================
  const SENS = [null,
    { x: 45, y: -85, a: 0 }, { x: 70, y: -40, a: 90 }, { x: 70, y: 40, a: 90 }, { x: 0, y: 85, a: 180 },
    { x: -70, y: 40, a: 270 }, { x: -70, y: -40, a: 270 }, { x: -45, y: -85, a: 0 }];
  const rayEls = [];
  function buildRobotSvg() {
    const svg = $('#robot-svg');
    svg.replaceChildren();
    for (const r of [100, 200, 300]) svg.append(sv('circle', { cx: 0, cy: 0, r, fill: 'none', stroke: 'var(--line)', 'stroke-dasharray': '3 5' }));
    svg.append(sv('rect', { x: -70, y: -85, width: 140, height: 170, rx: 10, fill: 'var(--code-bg)', stroke: 'var(--muted)', 'stroke-width': 2 }));
    svg.append(sv('path', { d: 'M -18 -60 L 0 -80 L 18 -60 Z', fill: 'var(--muted)' }));
    for (let i = 1; i <= 7; i++) {
      const p = SENS[i];
      const g = sv('g', { transform: `translate(${p.x} ${p.y}) rotate(${p.a})` });
      const ray = sv('line', { x1: 0, y1: 0, x2: 0, y2: -1, stroke: 'var(--ray-ok)', 'stroke-width': 5, 'stroke-linecap': 'round' });
      g.append(ray);
      svg.append(g);
      const dot = sv('circle', { cx: p.x, cy: p.y, r: 9, fill: 'var(--card)', stroke: 'var(--ink)', 'stroke-width': 1.5 });
      const num = sv('text', { x: p.x, y: p.y + 4, 'text-anchor': 'middle', class: 'svgtxt small' }, String(i));
      const lab = sv('text', { x: 0, y: 0, 'text-anchor': 'middle', class: 'svgtxt' }, '');
      svg.append(dot, num, lab);
      rayEls[i] = { ray, lab, p };
    }
    const tb = $('#dist-table tbody');
    tb.replaceChildren(...[1, 2, 3, 4, 5, 6, 7].map((i) => h('tr', {}, h('td', { text: i }), h('td', { text: POS[i] }), h('td', { class: 'num', id: 'dt-c' + i }), h('td', { class: 'num', id: 'dt-r' + i }), h('td', { class: 'num', id: 'dt-o' + i }))));
  }
  const distChart = new LineChart($('#chart-dist'), { title: 'Corrected distance (mm)', series: [1, 2, 3, 4, 5, 6, 7].map((i) => ({ label: String(i) })), windowMs: 15, xLabel: 's' });

  function buildCompass() {
    const c = $('#compass');
    c.replaceChildren(sv('circle', { cx: 0, cy: 0, r: 62, fill: 'none', stroke: 'var(--line)', 'stroke-width': 2 }));
    [['N', 0, -48], ['E', 48, 5], ['S', 0, 58], ['W', -48, 5]].forEach(([t, x, y]) => c.append(sv('text', { x, y, 'text-anchor': 'middle', class: 'svgtxt small' }, t)));
    c.append(sv('g', { id: 'needle' }, ''));
    $('#needle').append(sv('path', { d: 'M 0 -50 L 7 0 L -7 0 Z', fill: 'var(--danger)' }), sv('path', { d: 'M 0 50 L 7 0 L -7 0 Z', fill: 'var(--muted)' }));
  }

  function onTelemetry(o) {
    S.tel = Object.assign(S.tel || {}, o);
    const tl = S.tel;
    if (!S.telT0) S.telT0 = o.ms;
    if (o.d) {
      const rowsV = [];
      for (let i = 1; i <= 7; i++) {
        const d = o.d[i - 1], r = o.r[i - 1];
        const { ray, lab, p } = rayEls[i];
        const valid = d >= 0;
        const len = valid ? clamp(d, 4, 200) : 22;
        ray.setAttribute('y2', -len);
        ray.setAttribute('stroke', valid ? 'var(--ray-ok)' : 'var(--ray-bad)');
        ray.setAttribute('stroke-dasharray', valid ? '' : '2 8');
        const ang = p.a * Math.PI / 180;
        const tx = p.x + Math.sin(ang) * (len + 16), ty = p.y - Math.cos(ang) * (len + 16);
        lab.setAttribute('x', tx); lab.setAttribute('y', ty + 4);
        lab.textContent = valid ? d + (d > 200 ? '+' : '') : '–';
        $('#dt-c' + i).textContent = valid ? d : 'none';
        $('#dt-r' + i).textContent = r >= 0 ? r : 'none';
        $('#dt-o' + i).textContent = S.robot.get('off.' + i) ?? '';
        rowsV.push(valid ? d : null);
        const live = $('#cal-live-' + i); if (live) live.textContent = valid ? d + ' / ' + r : 'none';
      }
      distChart.push((o.ms - S.telT0) / 1000, rowsV);
      const dd = o.d;
      const kv = [];
      const f = dd[1], b = dd[2];
      if (f >= 0 && b >= 0 && f <= SIDE_WALL_MAX_MM && b <= SIDE_WALL_MAX_MM) {
        const D = (f + b) / 2;
        kv.push(['Right-wall error (center())', fmt(Math.trunc((TARGET_SIDE_GAP_MM - D) + (b - f)), 0) + ' mm']);
        kv.push(['  gap / angle term', fmt(D) + ' mm / ' + (b - f) + ' mm']);
      } else kv.push(['Right-wall error (center())', '0 (no right wall)']);
      if (dd[0] >= 0 && dd[6] >= 0) {
        const gap = (dd[0] + dd[6]) / 2;
        kv.push(['Front gap (sensors 1, 7)', fmt(gap) + ' mm']);
        kv.push(['  centering offset', fmt(gap - TARGET_GAP_MM) + ' mm (target gap ' + TARGET_GAP_MM + ')']);
        kv.push(['  1 vs 7 difference', (dd[0] - dd[6]) + ' mm']);
      }
      setKV($('#derived'), kv);
    }
    if (o.hdg !== undefined) {
      $('#needle').setAttribute('transform', `rotate(${tl.hdg})`);
      setKV($('#gyro-kv'), [['Heading', fmt(tl.hdg) + '°'], ['Orientation y', fmt(tl.rol) + '°'], ['Orientation z (pitch)', fmt(tl.pit) + '°']]);
      $('#gyro-cal').replaceChildren(...['sys', 'gyro', 'acc', 'mag'].map((n, i) => h('span', { class: 'pip p' + tl.cal[i], text: n + ' ' + tl.cal[i] })));
    }
    if (o.col) {
      const [r, g, b, c] = o.col;
      const mx = Math.max(r, g, b, 1), br = clamp(o.ratio, 0.2, 1);
      $('#swatch').style.background = `rgb(${Math.round(r / mx * 255 * br)},${Math.round(g / mx * 255 * br)},${Math.round(b / mx * 255 * br)})`;
      const name = CLASS_NAME[o.cls] || '?';
      $('#color-class').textContent = name;
      $('#cal-live-class').textContent = name;
      setKV($('#color-kv'), [['R / G / B', r + ' / ' + g + ' / ' + b], ['Clear', c], ['Baseline', fmt(S.robot.get('col.clear'), 0)], ['Ratio', fmt(o.ratio, 3)]]);
      const MAXR = 1.2;
      $('#ratio-fill').style.width = clamp(o.ratio / MAXR, 0, 1) * 100 + '%';
      $('#mk-black').style.left = clamp((S.robot.get('col.black') || 0) / MAXR, 0, 1) * 100 + '%';
      $('#mk-white').style.left = clamp((S.robot.get('col.white') || 0) / MAXR, 0, 1) * 100 + '%';
      onColorSample(o);
    }
    if (o.enc) {
      const avg = (o.enc[0] + o.enc[1] + o.enc[2]) / 3;
      const kv = [['Motor A', o.enc[0]], ['Motor B', o.enc[1]], ['Motor D', o.enc[2]], ['Average', fmt(avg, 0)], ['Distance', fmt(avg * WHEEL_MM_PER_COUNT) + ' mm']];
      setKV($('#enc-kv'), kv); setKV($('#enc-kv2'), kv);
      ['A', 'B', 'D'].forEach((m, i) => { const e = $('#menc-' + m); if (e) e.textContent = o.enc[i]; });
    }
  }
  $('#btn-encreset').onclick = $('#btn-encreset2').onclick = () => send('ENCRESET');

  // ============================== calibrate: distance ==============================
  function buildCalDist() {
    const tb = $('#cal-dist-table tbody');
    tb.replaceChildren(...[1, 2, 3, 4, 5, 6, 7].map((i) => h('tr', {},
      h('td', { text: i }), h('td', { text: POS[i] }), h('td', { class: 'num', id: 'cal-live-' + i, text: '–' }),
      h('td', { class: 'num', id: 'cal-off-' + i, text: '–' }),
      h('td', { class: 'num', id: 'cal-mean-' + i, text: '–' }), h('td', { class: 'num', id: 'cal-spread-' + i, text: '–' }),
      h('td', { class: 'num', id: 'cal-sug-' + i, text: '–' }),
      h('td', {}, h('button', { class: 'btn small', id: 'cal-meas-' + i, 'data-needs-robot': true, text: 'Measure', onclick: () => measureDist(i) }),
        ' ', h('button', { class: 'btn small primary', id: 'cal-apply-' + i, disabled: true, text: 'Apply', onclick: () => applyDist(i) })))));
  }
  async function measureDist(n) {
    const mm = Number($('#cal-true').value);
    if (!(mm >= 20 && mm <= 1000)) { toast('Target distance must be 20 to 1000 mm', true); return; }
    const btn = $('#cal-meas-' + n);
    btn.disabled = true; btn.textContent = 'Measuring…';
    try {
      const o = await S.robot.request('CALDIST ' + n + ' ' + mm, 'caldist', { match: (x) => x.n === n, timeout: 6000 });
      S.cal.dist[n] = o;
      renderCalDist(n);
      if (o.valid === 0) toast('Sensor ' + n + ' returned no valid readings', true);
    } catch (e) { toast('Measure failed: ' + e.message, true); }
    btn.textContent = 'Measure'; btn.disabled = !ready();
  }
  function applyDist(n) {
    const o = S.cal.dist[n];
    if (!o || o.offset === undefined) return;
    S.robot.set('off.' + n, o.offset);
    toast('Sensor ' + n + ' offset set to ' + o.offset + ' mm. Measure again to verify.');
  }
  function renderCalDist(n) {
    const o = S.cal.dist[n];
    const off = S.robot ? S.robot.get('off.' + n) : undefined;
    $('#cal-off-' + n).textContent = off === undefined ? '–' : off;
    if (!o) return;
    $('#cal-mean-' + n).textContent = o.valid ? fmt(o.mean) + ' mm (target ' + o.true + ')' : 'no reading';
    $('#cal-spread-' + n).textContent = o.valid ? o.min + '–' + o.max + ' (' + o.valid + '/' + o.samples + ')' : '–';
    $('#cal-sug-' + n).textContent = o.valid ? (o.offset > 0 ? '+' : '') + o.offset + ' mm' : '–';
    $('#cal-apply-' + n).disabled = !(ready() && o.valid > 0);
  }

  // ============================== calibrate: color ==============================
  const colorRows = {};
  function buildCalColor() {
    const tb = $('#cal-color-table tbody');
    tb.replaceChildren(...CLASS_KEYS.map((k) => {
      const tr = h('tr', {}, h('td', { text: k[0].toUpperCase() + k.slice(1) }),
        h('td', {}, h('button', { class: 'btn small', 'data-needs-robot': true, text: 'Sample', onclick: () => startColorSample(k) })),
        h('td', { class: 'num', id: 'cc-r-' + k, text: '–' }), h('td', { class: 'num', id: 'cc-g-' + k, text: '–' }), h('td', { class: 'num', id: 'cc-b-' + k, text: '–' }),
        h('td', { class: 'num', id: 'cc-c-' + k, text: '–' }), h('td', { class: 'num', id: 'cc-ratio-' + k, text: '–' }));
      colorRows[k] = tr;
      return tr;
    }));
    renderColorSuggest();
  }
  function startColorSample(k) {
    S.cal.sampling = { k, rows: [], until: Date.now() + 1500 };
    $('#cc-ratio-' + k).textContent = 'sampling…';
  }
  function onColorSample(o) {
    const sm = S.cal.sampling;
    if (!sm) return;
    sm.rows.push(o.col);
    if (Date.now() < sm.until) return;
    S.cal.sampling = null;
    const stat = (i) => { const v = sm.rows.map((r) => r[i]); return { mean: v.reduce((a, b) => a + b, 0) / v.length, min: Math.min(...v), max: Math.max(...v) }; };
    S.cal.color[sm.k] = { n: sm.rows.length, r: stat(0), g: stat(1), b: stat(2), c: stat(3) };
    renderColorTable();
    renderColorSuggest();
  }
  function renderColorTable() {
    const clear = S.robot ? S.robot.get('col.clear') : 0;
    for (const k of CLASS_KEYS) {
      const d = S.cal.color[k];
      if (!d) continue;
      $('#cc-r-' + k).textContent = Math.round(d.r.mean); $('#cc-g-' + k).textContent = Math.round(d.g.mean);
      $('#cc-b-' + k).textContent = Math.round(d.b.mean); $('#cc-c-' + k).textContent = Math.round(d.c.mean);
      $('#cc-ratio-' + k).textContent = fmt(d.c.min / clear, 3) + ' – ' + fmt(d.c.max / clear, 3);
    }
  }
  // Same decision order as classifyColor() in Main/color.ino.
  function classify(r, g, b, c, clear, th) {
    if (c === 0) return 0;
    if (c / clear < th.black) return -1;
    if (r > th.silver) return 3;
    if (c / clear > th.white) return 0;
    if (b > g + 10 && b > r + 10) return 1;
    if (r > g + 10 && r > b + 10) return 2;
    return 0;
  }
  const EXPECT = { white: 0, black: -1, silver: 3, blue: 1, red: 2 };
  function mid(lo, hi) { return (lo + hi) / 2; }
  function suggestColor() {
    const clear = S.robot.get('col.clear');
    const D = S.cal.color, have = (k) => !!D[k];
    const ratio = (k) => ({ min: D[k].c.min / clear, max: D[k].c.max / clear });
    const notes = [], out = {};
    if (have('black')) {
      const others = ['white', 'silver', 'blue', 'red'].filter(have);
      if (others.length) {
        const lo = ratio('black').max, hi = Math.min(...others.map((k) => ratio(k).min));
        out.black = +mid(lo, hi).toFixed(3);
        if (lo >= hi) notes.push('Black and ' + others.join('/') + ' ratios overlap: the black threshold cannot separate them (sample again, or check the sensor height).');
      }
    }
    if (have('silver')) {
      const others = ['white', 'blue', 'red'].filter(have);
      if (others.length) {
        const lo = Math.max(...others.map((k) => D[k].r.max)), hi = D.silver.r.min;
        out.silver = Math.round(mid(lo, hi));
        if (lo >= hi) notes.push('Silver red level overlaps ' + others.join('/') + ': silver cannot be told apart by red alone.');
      }
    }
    if (have('white')) {
      const others = ['blue', 'red'].filter(have);
      if (others.length) {
        const lo = Math.max(...others.map((k) => ratio(k).max)), hi = ratio('white').min;
        out.white = +mid(lo, hi).toFixed(3);
        if (lo >= hi) notes.push('White ratio overlaps blue/red: the white threshold cannot separate them.');
      }
    }
    return { out, notes, clear };
  }
  function renderColorSuggest() {
    const box = $('#cal-color-suggest');
    if (!S.robot) { box.textContent = 'Connect to the robot to calibrate colors.'; return; }
    $('#cal-clear').textContent = fmt(S.robot.get('col.clear'), 1);
    renderColorTable();
    const have = CLASS_KEYS.filter((k) => S.cal.color[k]);
    if (have.length < 2) { box.replaceChildren(h('span', { class: 'hint', text: 'Sample at least white plus one other tile to get suggestions.' })); return; }
    const { out, notes, clear } = suggestColor();
    const cur = { black: S.robot.get('col.black'), silver: S.robot.get('col.silver'), white: S.robot.get('col.white') };
    const th = Object.assign({}, cur, out);
    const kids = [];
    const rows = ['black', 'silver', 'white'].map((k) => h('tr', {}, h('td', { text: 'col.' + k }), h('td', { class: 'num', text: String(cur[k]) }), h('td', { class: 'num', text: out[k] === undefined ? '(needs more samples)' : String(out[k]) })));
    kids.push(h('table', { class: 'tbl' }, h('thead', {}, h('tr', {}, h('th', { text: 'Threshold' }), h('th', { text: 'Now' }), h('th', { text: 'Suggested' }))), h('tbody', {}, rows)));
    const checks = have.map((k) => {
      const d = S.cal.color[k];
      const got = classify(d.r.mean, d.g.mean, d.b.mean, d.c.mean, clear, th);
      return k + ' → ' + CLASS_NAME[got] + (got === EXPECT[k] ? ' ✓' : ' ✗');
    });
    kids.push(h('p', { class: 'hint', text: 'With the suggested thresholds the mean readings classify as: ' + checks.join(' · ') }));
    for (const n of notes) kids.push(h('p', { class: 'warn', text: '⚠ ' + n }));
    kids.push(h('button', { class: 'btn primary', 'data-needs-robot': true, disabled: !ready(), text: 'Apply suggested thresholds', onclick: () => { for (const k of Object.keys(out)) S.robot.set('col.' + k, out[k]); toast('Color thresholds applied'); } }));
    box.replaceChildren(...kids);
  }
  $('#btn-clearref').onclick = async () => {
    try { const o = await S.robot.request('CLEARREF', 'clearref', { timeout: 6000 }); S.robot.params.get('col.clear').v = o.clear; refreshBindings('col.clear'); renderColorSuggest(); toast('White reference set to ' + o.clear); }
    catch (e) { toast('CLEARREF failed: ' + e.message, true); }
  };

  // ============================== PID tab ==============================
  const GROUP_OF = (n) => n.split('.')[0];
  const KINDS = {
    'turn': { label: 'Turn', groups: ['turn'] },
    'fwd-wall': { label: 'Forward (wall follow)', groups: ['center', 'scale'] },
    'fwd-gyro': { label: 'Forward (gyro hold)', groups: ['gyro', 'scale'] },
  };
  const PID_GROUPS = ['center', 'scale', 'gyro', 'climb', 'turn', 'par', 'wig'];
  const CHARTS = {
    turn: [
      { title: 'Heading error (°)', cols: ['error_deg'], hl: [0, 3, -3] },
      { title: 'PID output / motor speed', cols: ['pid_out', 'speed'] },
      { title: 'Heading (°)', cols: ['heading'] },
    ],
    fwd: [
      { title: 'Steering error', cols: ['error'], hl: [0] },
      { title: 'Steering PID output', cols: ['pid_out'], hl: [0] },
      { title: 'Wheel speeds', cols: ['speed_AC', 'speed_BD'] },
      { title: 'Distance (mm)', cols: ['mm'], target: true },
      { title: 'Scale', cols: ['scale'] },
    ],
  };
  const kindFam = (k) => (k === 'turn' ? 'turn' : 'fwd');
  let testCharts = []; // [{chart, cfg, fam}]

  function sliderToValue(p, pos) {
    const exp = p.n.endsWith('.kd') ? 3 : 2;
    let v = p.lo + (p.hi - p.lo) * Math.pow(pos / 1000, exp);
    v = p.ty === 'i' ? Math.round(v) : Number(v.toPrecision(3));
    return clamp(v, p.lo, p.hi);
  }
  function valueToSlider(p, v) {
    const exp = p.n.endsWith('.kd') ? 3 : 2;
    const f = clamp((v - p.lo) / (p.hi - p.lo), 0, 1);
    return Math.round(1000 * Math.pow(f, 1 / exp));
  }
  function gainRow(p, label) {
    const range = h('input', { type: 'range', min: 0, max: 1000, step: 1, value: valueToSlider(p, p.v), 'aria-label': p.n });
    const num = h('input', { type: 'number', step: p.st, value: short(p.v), 'aria-label': p.n + ' value' });
    const rev = h('button', { class: 'rev', title: 'Revert to firmware default (' + short(p.d) + ')', text: '↺' });
    const def = h('div', { class: 'gaindef', text: 'default ' + short(p.d) + ' · range 0 – ' + short(p.hi) + (p.n.endsWith('.kd') ? ' · per µs' : '') });
    const row = h('div', { class: 'gain' }, h('label', { text: label }), range, num, rev, def);
    let deb = 0;
    const apply = (v) => { clearTimeout(deb); deb = setTimeout(() => S.robot && S.robot.set(p.n, v), 50); };
    range.addEventListener('input', () => { const v = sliderToValue(p, Number(range.value)); num.value = short(v); apply(v); });
    num.addEventListener('change', () => { const v = Number(num.value); if (!Number.isFinite(v)) { num.value = short(S.robot.get(p.n)); return; } apply(clamp(v, p.lo, p.hi)); });
    rev.addEventListener('click', () => S.robot.set(p.n, p.d));
    bind(p.n, (v) => {
      if (document.activeElement !== num) num.value = short(v);
      if (document.activeElement !== range) range.value = valueToSlider(p, v);
      row.classList.toggle('mod', Math.abs(v - p.d) > 1e-12);
    }, 'gains');
    row.classList.toggle('mod', Math.abs(p.v - p.d) > 1e-12);
    return row;
  }
  function groupBlock(id) {
    const ps = [...S.robot.params.values()].filter((p) => GROUP_OF(p.n) === id);
    if (!ps.length) return null;
    return h('div', { class: 'gains' }, h('h3', {}, ps[0].g, h('span', { class: 'hint', text: '' })), ps.map((p) => gainRow(p, p.n.split('.')[1])));
  }
  function renderGains() {
    const wrap = $('#gains-wrap');
    wrap.replaceChildren();
    if (!S.robot) return;
    unbind('gains'); // gain rows are re-bound on every render
    const main = KINDS[S.testKind].groups, rest = PID_GROUPS.filter((g) => !main.includes(g));
    for (const g of main) { const b = groupBlock(g); if (b) wrap.append(b); }
    const det = h('details', { class: 'gains' }, h('summary', { text: 'Other controllers (not used by this test)' }));
    for (const g of rest) { const b = groupBlock(g); if (b) det.append(b); }
    wrap.append(det);
  }
  function renderTestOpts() {
    const o = $('#test-opts');
    if (S.testKind === 'turn') {
      o.replaceChildren(h('label', { text: 'Angle (°)' }), h('input', { id: 'opt-angle', type: 'number', value: 90, min: 2, max: 360, step: 5 }),
        h('label', { text: 'Direction' }), h('select', { id: 'opt-dir' }, h('option', { value: 'right', text: 'Right (clockwise)' }), h('option', { value: 'left', text: 'Left' })));
    } else {
      o.replaceChildren(h('label', { text: 'Distance (mm)' }), h('input', { id: 'opt-mm', type: 'number', value: 300, min: 50, max: 1200, step: 50 }),
        h('label', { text: 'Loop timing' }), h('label', {}, h('input', { id: 'opt-real', type: 'checkbox', checked: true }), ' mirror real fwd() (reads color sensor each tick)'));
    }
  }
  $('#test-kind').addEventListener('click', (e) => {
    const b = e.target.closest('button[data-kind]');
    if (!b || S.test) return;
    S.testKind = b.dataset.kind;
    $$('#test-kind button').forEach((x) => x.classList.toggle('active', x === b));
    renderTestOpts(); renderGains(); buildTestCharts();
  });
  function buildTestCharts() {
    const fam = kindFam(S.testKind), wrap = $('#test-charts');
    wrap.replaceChildren();
    testCharts = CHARTS[fam].map((cfg) => {
      const canvas = h('canvas', { height: 180 });
      wrap.append(canvas);
      return { cfg, fam, chart: new LineChart(canvas, { title: cfg.title, series: cfg.cols.map((c) => ({ label: c })), hlines: cfg.hl || [], xLabel: 's' }) };
    });
    $('#test-title').textContent = KINDS[S.testKind].label;
    const g = S.ghost && kindFam(S.ghost.kind) === fam ? S.ghost : null;
    if (g) applyRowsToCharts(g.rows, g.cols, true, g);
    renderSummary(null);
  }
  function applyRowsToCharts(rows, cols, ghost, run) {
    for (const t of testCharts) {
      const idx = t.cfg.cols.map((c) => cols.indexOf(c));
      const mapped = rows.map((r) => [r[0] / 1000].concat(idx.map((i) => (i < 0 ? null : r[i]))));
      if (ghost) t.chart.setGhost(mapped, '#' + run.id);
      else t.chart.setRows(mapped);
    }
  }
  function gainsSnapshot() {
    const g = {};
    for (const id of PID_GROUPS) g[id] = ['kp', 'ki', 'kd'].map((k) => S.robot.get(id + '.' + k));
    return g;
  }
  function setTestRunning(on) {
    $('#btn-run').disabled = on || !ready();
    $('#test-status').textContent = on ? 'running… (Stop or Space to abort)' : '';
    $$('#test-kind button').forEach((b) => { b.disabled = on; });
  }
  $('#btn-run').onclick = () => {
    if (!ready() || S.test) return;
    let cmd;
    if (S.testKind === 'turn') {
      const deg = Number($('#opt-angle').value) * ($('#opt-dir').value === 'left' ? -1 : 1);
      if (!(Math.abs(deg) >= 2 && Math.abs(deg) <= 360)) { toast('Angle must be 2 to 360°', true); return; }
      cmd = 'TEST TURN ' + deg;
    } else {
      const mm = Number($('#opt-mm').value);
      if (!(mm >= 50 && mm <= 1200)) { toast('Distance must be 50 to 1200 mm', true); return; }
      cmd = 'TEST FWD ' + mm + ' ' + (S.testKind === 'fwd-gyro' ? 'gyro' : 'wall') + ' ' + ($('#opt-real').checked ? 1 : 0);
    }
    Drive.releaseAll();
    S.pending = { kind: S.testKind, gains: gainsSnapshot(), target: S.testKind === 'turn' ? null : Number($('#opt-mm').value) };
    setTestRunning(true);
    S.robot.send(cmd);
  };
  function onTestStart(o) {
    const p = S.pending || { kind: S.testKind, gains: gainsSnapshot() };
    S.test = { kind: p.kind, name: o.name, cols: o.cols, info: o.info, rows: [], gains: p.gains, target: p.target };
    setTestRunning(true);
    const fam = kindFam(p.kind);
    if (!testCharts.length || testCharts[0].fam !== fam) buildTestCharts();
    for (const t of testCharts) {
      t.chart.clear();
      if (t.cfg.target && p.target) t.chart.setHlines([p.target]);
    }
    const g = S.ghost && kindFam(S.ghost.kind) === fam ? S.ghost : null;
    if (g) applyRowsToCharts(g.rows, g.cols, true, g); else testCharts.forEach((t) => t.chart.setGhost(null));
    renderSummary(null, 'Running: ' + o.info);
  }
  function onTrace(o) {
    const T = S.test;
    if (!T) return;
    T.rows.push(o.v);
    for (const t of testCharts) {
      const idx = t.cfg.cols.map((c) => T.cols.indexOf(c));
      t.chart.push(o.v[0] / 1000, idx.map((i) => (i < 0 ? null : o.v[i])));
    }
  }
  function onTestDone(o) {
    const T = S.test;
    S.test = null; S.pending = null;
    setTestRunning(false);
    if (!T) return;
    const run = { id: ++S.runSeq, kind: T.kind, cols: T.cols, rows: T.rows, gains: T.gains, changed: !!T.changed, why: o.why, finalErr: o.final_err, extra: o.extra, ms: o.ms, info: T.info, target: T.target };
    run.metrics = metrics(run);
    S.runs.unshift(run);
    S.ghost = run; // next run of this kind is drawn against this one
    renderSummary(run);
    renderRuns();
  }
  function metrics(run) {
    const m = {};
    const ci = (c) => run.cols.indexOf(c);
    if (run.kind === 'turn') {
      const e = run.rows.map((r) => r[ci('error_deg')]);
      if (e.length) {
        const s0 = Math.sign(e[0]) || 1;
        m.overshoot = Math.max(0, ...e.map((x) => -s0 * x));
        const k = e.findIndex((x) => Math.abs(x) <= 3);
        m.settle = k >= 0 ? run.rows[k][0] : null;
      }
    }
    return m;
  }
  function renderSummary(run, running) {
    const box = $('#test-summary');
    if (!run) { box.replaceChildren(h('span', { class: 'hint', text: running || 'Run a test to see the response here. The dashed line is the previous run of the same kind.' })); return; }
    const mk = (label, val) => h('span', { class: 'metric' }, label + ' ', h('b', { text: val }));
    const kids = [mk('ended', run.why)];
    if (run.kind === 'turn') {
      kids.push(mk('final error', fmt(run.finalErr) + '°'), mk('overshoot', fmt(run.metrics.overshoot) + '°'), mk('first within ±3°', run.metrics.settle == null ? 'never' : Math.round(run.metrics.settle) + ' ms'));
    } else {
      kids.push(mk('travelled', fmt(run.extra) + ' mm'), mk('vs target', (run.finalErr >= 0 ? '+' : '') + fmt(run.finalErr) + ' mm'));
    }
    kids.push(mk('time', run.ms + ' ms'));
    if (run.changed) kids.push(h('span', { class: 'warn', text: '⚠ gains were changed during this run, so the table shows the gains it started with' }));
    box.replaceChildren(...kids);
  }
  function renderRuns() {
    const tb = $('#run-table tbody');
    tb.replaceChildren(...S.runs.slice(0, 30).map((run) => {
      const groups = KINDS[run.kind].groups;
      const gtxt = groups.map((g) => g + ' ' + run.gains[g].map(short).join('/')).join('  ');
      const res = run.kind === 'turn' ? 'err ' + fmt(run.finalErr) + '°, over ' + fmt(run.metrics.overshoot) + '°' : 'went ' + fmt(run.extra) + ' mm (' + (run.finalErr >= 0 ? '+' : '') + fmt(run.finalErr) + ')';
      const tr = h('tr', { class: 'run' + (S.ghost === run ? ' ghost' : ''), title: 'Overlay this run on the next one', onclick: () => { S.ghost = run; renderRuns(); if (kindFam(run.kind) === kindFam(S.testKind)) applyRowsToCharts(run.rows, run.cols, true, run); } },
        h('td', { text: run.id }), h('td', { text: KINDS[run.kind].label }), h('td', { class: 'gainscol', text: gtxt + ' (kp/ki/kd)' + (run.changed ? ' ⚠ changed mid-run' : '') }), h('td', { text: res + ' · ' + run.why }),
        h('td', {}, h('button', { class: 'btn small', text: 'Restore gains', title: 'Set these gains on the robot again', onclick: (ev) => { ev.stopPropagation(); restoreGains(run); } })));
      return tr;
    }));
  }
  function restoreGains(run) {
    for (const g of KINDS[run.kind].groups) ['kp', 'ki', 'kd'].forEach((k, i) => S.robot.set(g + '.' + k, run.gains[g][i]));
    toast('Gains from run #' + run.id + ' restored');
  }

  // ============================== motors ==============================
  function buildMotors() {
    const speed = $('#drive-speed');
    speed.oninput = () => { S.driveSpeed = Number(speed.value); $('#drive-speed-v').textContent = speed.value; };
    speed.onchange = () => speed.blur(); // so arrow keys drive instead of nudging the slider
    $$('.dpad [data-dir]').forEach((b) => {
      const d = b.dataset.dir, key = 'dir:' + d;
      b.dataset.hold = key;
      b.addEventListener('pointerdown', (e) => { e.preventDefault(); b.setPointerCapture(e.pointerId); Drive.hold(key, () => 'DRIVE ' + d + ' ' + S.driveSpeed); });
      for (const ev of ['pointerup', 'pointercancel', 'lostpointercapture']) b.addEventListener(ev, () => Drive.release(key, 'STOP'));
    });
    const rows = $('#motor-rows');
    rows.replaceChildren(...['A', 'B', 'C', 'D'].map((m) => {
      const sl = h('input', { type: 'range', min: -255, max: 255, step: 5, value: 100, 'aria-label': 'Motor ' + m + ' speed' });
      const val = h('b', { class: 'num', text: '100' });
      sl.oninput = () => { val.textContent = sl.value; };
      const key = 'mot:' + m;
      const hold = h('button', { class: 'btn', 'data-needs-robot': true, 'data-hold': key, text: 'Hold to run' });
      hold.addEventListener('pointerdown', (e) => { e.preventDefault(); hold.setPointerCapture(e.pointerId); Drive.hold(key, () => 'MOT ' + m + ' ' + sl.value); });
      for (const ev of ['pointerup', 'pointercancel', 'lostpointercapture']) hold.addEventListener(ev, () => Drive.release(key, 'MOT ' + m + ' 0'));
      const pulse = h('button', { class: 'btn pulse', 'data-needs-robot': true, text: 'Pulse 0.5 s', onclick: () => { Drive.hold(key, () => 'MOT ' + m + ' ' + sl.value); setTimeout(() => Drive.release(key, 'MOT ' + m + ' 0'), 500); } });
      return h('div', { class: 'motor' }, h('b', { text: 'Motor ' + m }), sl, val, hold, h('span', { class: 'enc' }, pulse, ' ', m === 'C' ? 'no enc' : h('span', { id: 'menc-' + m, text: '–' })));
    }));
  }

  // ============================== parameters / profile ==============================
  function buildParamUi() {
    unbindAll(); // everything below re-binds (the robot may have re-entered tuning mode)
    // PID tab
    renderTestOpts(); renderGains(); buildTestCharts();
    // calibration tables show current offsets
    for (let i = 1; i <= 7; i++) { bind('off.' + i, () => renderCalDist(i), 'cal'); renderCalDist(i); }
    for (const n of ['col.black', 'col.silver', 'col.white', 'col.clear']) bind(n, renderColorSuggest, 'cal');
    renderColorSuggest();
    // parameters tab
    const list = $('#params-list');
    list.replaceChildren();
    const groups = new Map();
    for (const p of S.robot.params.values()) { if (!groups.has(p.g)) groups.set(p.g, []); groups.get(p.g).push(p); }
    for (const [g, ps] of groups) {
      list.append(h('div', { class: 'card plist' }, h('h2', { text: g }), ps.map((p) => {
        const inp = h('input', { type: 'number', step: p.st, value: short(p.v), 'aria-label': p.n });
        const rev = h('button', { class: 'btn small', text: 'Reset', title: 'default ' + short(p.d) });
        const row = h('div', { class: 'prow' }, h('span', { class: 'name mono', text: p.n }), inp, h('span', { class: 'desc', text: p.desc + ' (default ' + short(p.d) + ', range ' + short(p.lo) + ' – ' + short(p.hi) + ')' }), rev);
        inp.addEventListener('change', () => { const v = Number(inp.value); if (!Number.isFinite(v)) { inp.value = short(S.robot.get(p.n)); return; } if (v < p.lo || v > p.hi) { toast(p.n + ' must be between ' + short(p.lo) + ' and ' + short(p.hi), true); inp.value = short(S.robot.get(p.n)); return; } S.robot.set(p.n, v); });
        rev.addEventListener('click', () => S.robot.set(p.n, p.d));
        const refresh = (v) => { if (document.activeElement !== inp) inp.value = short(v); row.classList.toggle('mod', Math.abs(v - p.d) > 1e-12); };
        bind(p.n, (v) => { refresh(v); renderCpp(); }, 'params');
        refresh(p.v);
        return row;
      })));
    }
    renderCpp();
    updateTop();
  }

  const PROFILE_KEY = 'theseus-tuner-profile';
  function profile() {
    const params = {};
    for (const p of S.robot.params.values()) if (p.n !== 'col.clear') params[p.n] = p.v; // baseline is measured at boot / by Set white reference
    return { version: 1, saved: new Date().toISOString(), fw: S.robot.fw, params };
  }
  function applyProfile(pr, label) {
    let n = 0;
    for (const [name, v] of Object.entries(pr.params || {})) {
      const p = S.robot.params.get(name);
      if (!p || name === 'col.clear' || !Number.isFinite(v)) continue;
      if (Math.abs(p.v - v) > 1e-12) { S.robot.set(name, v); n++; }
    }
    toast(label + ': ' + n + ' value' + (n === 1 ? '' : 's') + ' sent to the robot');
  }
  function readSaved() { try { return JSON.parse(localStorage.getItem(PROFILE_KEY)); } catch (e) { return null; } }
  function offerSavedProfile() {
    const pr = readSaved();
    if (!pr || !pr.params) return;
    const differs = Object.entries(pr.params).some(([n, v]) => { const p = S.robot.params.get(n); return p && n !== 'col.clear' && Math.abs(p.v - v) > 1e-12; });
    $('#profile-info').textContent = 'Saved in this browser: ' + new Date(pr.saved).toLocaleString();
    if (differs) banner('A profile saved on ' + new Date(pr.saved).toLocaleString() + ' differs from the values on the robot (the robot resets to its compiled defaults when it reboots).', '', [
      { label: 'Apply it', fn: () => { applyProfile(pr, 'Saved profile'); hideBanner(); } }, { label: 'Dismiss', fn: hideBanner }]);
  }
  $('#btn-save-local').onclick = () => {
    if (!S.robot) return;
    try { localStorage.setItem(PROFILE_KEY, JSON.stringify(profile())); toast('Profile saved in this browser'); $('#profile-info').textContent = 'Saved just now'; }
    catch (e) { toast('Could not save (browser storage blocked): ' + e.message, true); }
  };
  $('#btn-load-local').onclick = () => { const pr = readSaved(); if (!pr) { toast('No saved profile in this browser', true); return; } if (S.robot) applyProfile(pr, 'Saved profile'); };
  $('#btn-dl').onclick = () => { if (S.robot) download('theseus-profile.json', JSON.stringify(profile(), null, 2), 'application/json'); };
  $('#file-load').onchange = async (e) => {
    const f = e.target.files[0]; e.target.value = '';
    if (!f || !S.robot) return;
    try { applyProfile(JSON.parse(await f.text()), 'File'); } catch (err) { toast('Not a valid profile file', true); }
  };
  $('#btn-reset-all').onclick = () => { if (!S.robot) return; let n = 0; for (const p of S.robot.params.values()) if (p.n !== 'col.clear' && Math.abs(p.v - p.d) > 1e-12) { S.robot.set(p.n, p.d); n++; } toast('Reset ' + n + ' value(s) to firmware defaults'); };
  function download(name, text, type) {
    const a = h('a', { href: URL.createObjectURL(new Blob([text], { type })), download: name });
    document.body.append(a); a.click(); a.remove();
  }

  // C++ that can be pasted into Main/Tunables.cpp and Main/Distance.ino to make the tuned values the new defaults.
  const num = (v) => String(Number(Number(v).toPrecision(8)));
  const flt = (v) => { const s = num(v); return (/[.e]/i.test(s) ? s : s + '.0') + 'f'; };
  function buildCpp() {
    if (!S.robot) return '// Connect to the robot to generate code from its current values.';
    const g = (id) => ['kp', 'ki', 'kd'].map((k) => num(S.robot.get(id + '.' + k))).join(', ');
    const off = [0, 1, 2, 3, 4, 5, 6, 7].map((i) => (i === 0 ? 0 : S.robot.get('off.' + i)));
    return [
      '// ---- Main/Tunables.cpp: replace the `tune` initializer ----',
      'TuneState tune = {',
      '  /* center   */ {' + g('center') + '},',
      '  /* scale    */ {' + g('scale') + '},',
      '  /* gyro     */ {' + g('gyro') + '},',
      '  /* climb    */ {' + g('climb') + '},',
      '  /* turn     */ {' + g('turn') + '},',
      '  /* parallel */ {' + g('par') + '},',
      '  /* wiggle   */ {' + g('wig') + '},',
      '  /* colBlack  */ ' + flt(S.robot.get('col.black')) + ',',
      '  /* colSilver */ ' + Math.round(S.robot.get('col.silver')) + ',',
      '  /* colWhite  */ ' + flt(S.robot.get('col.white')),
      '};',
      '',
      '// ---- Main/Distance.ino: replace SENSOR_OFFSET_MM ----',
      'int SENSOR_OFFSET_MM[8] = {' + off.join(', ') + '};',
      '',
      '// (The color baseline `clear` is measured at boot by init_color(): power the robot on over a white tile.)',
    ].join('\n');
  }
  function renderCpp() { $('#cpp').textContent = buildCpp(); }
  $('#btn-copy-cpp').onclick = async () => {
    const t = buildCpp();
    try { await navigator.clipboard.writeText(t); $('#copy-info').textContent = 'Copied'; }
    catch (e) { const r = document.createRange(); r.selectNodeContents($('#cpp')); const s = getSelection(); s.removeAllRanges(); s.addRange(r); $('#copy-info').textContent = 'Selected: press Ctrl/Cmd+C'; }
    setTimeout(() => { $('#copy-info').textContent = ''; }, 2500);
  };

  // ============================== log ==============================
  function appendLog(dir, text, noisy) {
    S.log.push({ t: Date.now(), dir, text, noisy });
    if (S.log.length > 3000) S.log.splice(0, 500);
    if (activeTab === 'log') { const node = logNode(S.log[S.log.length - 1]); if (node) { const el = $('#log'); const atEnd = el.scrollTop + el.clientHeight >= el.scrollHeight - 20; el.append(node); while (el.childNodes.length > 500) el.firstChild.remove(); if (atEnd) el.scrollTop = el.scrollHeight; } }
  }
  function logNode(e) {
    if (e.noisy && !$('#log-noisy').checked) return null;
    if (e.dir === 'dbg' && !$('#log-debug').checked) return null;
    const ts = new Date(e.t).toISOString().slice(11, 23);
    const cls = e.dir === 'tx' ? 'tx' : e.dir === 'dbg' ? 'dbg' : (e.text.includes('"t":"err"') ? 'er' : '');
    return h('div', { class: cls, text: ts + (e.dir === 'tx' ? ' > ' : e.dir === 'dbg' ? ' · ' : ' < ') + e.text });
  }
  function renderLog() {
    const el = $('#log');
    el.replaceChildren(...S.log.slice(-1500).map(logNode).filter(Boolean).slice(-500));
    el.scrollTop = el.scrollHeight;
  }
  $('#log-noisy').onchange = $('#log-debug').onchange = renderLog;
  $('#btn-log-clear').onclick = () => { S.log = []; renderLog(); };
  $('#btn-log-dl').onclick = () => download('tuner-log.txt', S.log.map((e) => new Date(e.t).toISOString() + ' ' + e.dir + ' ' + e.text).join('\n'), 'text/plain');
  $('#raw-form').onsubmit = (e) => { e.preventDefault(); const v = $('#raw-cmd').value.trim(); if (v && ready()) S.robot.send(v); $('#raw-cmd').value = ''; };

  // ============================== disconnect cleanup / init ==============================
  function clearUiForDisconnect() {
    S.test = null; S.pending = null; S.cal.sampling = null; S.telT0 = 0;
    setTestRunning(false);
    $('#gains-wrap').replaceChildren(); $('#params-list').replaceChildren();
    renderCpp(); renderColorSuggest();
  }

  buildRobotSvg(); buildCompass(); buildCalDist(); buildCalColor(); buildMotors();
  renderTestOpts(); buildTestCharts(); renderRuns();
  // anything that needs a live robot starts disabled
  $$('#btn-run, #btn-encreset, #btn-encreset2, #btn-clearref').forEach((b) => b.setAttribute('data-needs-robot', ''));
  updateTop();
  renderCpp();
  if (!SerialLink.supported) banner('This browser has no Web Serial support. Use Chrome or Edge on a laptop to talk to the robot. The Demo button works anywhere.', '');
})();
