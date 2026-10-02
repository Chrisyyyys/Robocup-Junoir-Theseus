// Tiny dependency-free line chart (canvas). Rows are arrays: [x, v1, v2, ...] aligned with `series`.
// null values break the line. Supports a rolling window, horizontal reference lines, a faded
// "ghost" run for comparison, and a hover read-out.
(function () {
  'use strict';

  const css = (n) => getComputedStyle(document.documentElement).getPropertyValue(n).trim();

  function niceStep(range, target) {
    const raw = range / Math.max(1, target);
    const mag = Math.pow(10, Math.floor(Math.log10(raw)));
    const n = raw / mag;
    return (n < 1.5 ? 1 : n < 3 ? 2 : n < 7 ? 5 : 10) * mag;
  }

  function fmt(v) {
    if (v === null || v === undefined || Number.isNaN(v)) return '–';
    const a = Math.abs(v);
    if (a >= 1000) return v.toFixed(0);
    if (a >= 100) return v.toFixed(1);
    if (a >= 1) return v.toFixed(2);
    return v.toPrecision(2);
  }

  class LineChart {
    // opts: {title, series:[{label}], hlines:[numbers], windowMs, yMin, yMax, xLabel}
    constructor(canvas, opts) {
      this.canvas = canvas;
      this.ctx = canvas.getContext('2d');
      this.title = opts.title || '';
      this.series = opts.series || [];
      this.hlines = opts.hlines || [];
      this.windowMs = opts.windowMs || 0;
      this.fixedMin = opts.yMin;
      this.fixedMax = opts.yMax;
      this.xLabel = opts.xLabel || '';
      this.rows = [];
      this.ghost = null;
      this.ghostLabel = '';
      this.hoverX = null;
      this._raf = 0;
      if (window.ResizeObserver) new ResizeObserver(() => this.schedule()).observe(canvas);
      canvas.addEventListener('mousemove', (e) => {
        const r = canvas.getBoundingClientRect();
        this.hoverX = e.clientX - r.left;
        this.schedule();
      });
      canvas.addEventListener('mouseleave', () => { this.hoverX = null; this.schedule(); });
      this.schedule();
    }

    clear() { this.rows = []; this.schedule(); }
    setRows(rows) { this.rows = rows; this.schedule(); }
    setGhost(rows, label) { this.ghost = rows && rows.length ? rows : null; this.ghostLabel = label || ''; this.schedule(); }
    setHlines(h) { this.hlines = h; this.schedule(); }

    push(x, vals) {
      this.rows.push([x].concat(vals));
      if (this.windowMs) {
        const cut = x - this.windowMs;
        while (this.rows.length && this.rows[0][0] < cut) this.rows.shift();
      }
      this.schedule();
    }

    schedule() {
      if (this._raf) return;
      this._raf = requestAnimationFrame(() => { this._raf = 0; this.draw(); });
    }

    draw() {
      const c = this.canvas, ctx = this.ctx;
      const dpr = window.devicePixelRatio || 1;
      const W = c.clientWidth, H = c.clientHeight;
      if (!W || !H) return;
      if (c.width !== Math.round(W * dpr) || c.height !== Math.round(H * dpr)) {
        c.width = Math.round(W * dpr);
        c.height = Math.round(H * dpr);
      }
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, W, H);

      const ink = css('--ink'), muted = css('--muted'), line = css('--line');
      const L = 46, R = 10, T = 22, B = 22;
      const pw = W - L - R, ph = H - T - B;
      const rows = this.rows, ghost = this.ghost;

      // ranges
      let xMin = Infinity, xMax = -Infinity, yMin = Infinity, yMax = -Infinity;
      const scan = (rs) => {
        for (const r of rs) {
          if (r[0] < xMin) xMin = r[0];
          if (r[0] > xMax) xMax = r[0];
          for (let i = 1; i < r.length; i++) {
            const v = r[i];
            if (v === null || v === undefined || Number.isNaN(v)) continue;
            if (v < yMin) yMin = v;
            if (v > yMax) yMax = v;
          }
        }
      };
      scan(rows);
      if (ghost) scan(ghost);
      if (!isFinite(xMin)) { xMin = 0; xMax = 1; }
      if (!isFinite(yMin)) { yMin = 0; yMax = 1; }
      for (const h of this.hlines) { if (h < yMin) yMin = h; if (h > yMax) yMax = h; }
      if (this.fixedMin !== undefined) yMin = this.fixedMin;
      if (this.fixedMax !== undefined) yMax = this.fixedMax;
      if (yMax - yMin < 1e-9) { yMax += 1; yMin -= 1; }
      else if (this.fixedMin === undefined || this.fixedMax === undefined) {
        const pad = (yMax - yMin) * 0.08;
        if (this.fixedMin === undefined) yMin -= pad;
        if (this.fixedMax === undefined) yMax += pad;
      }
      if (xMax - xMin < 1e-9) xMax = xMin + 1;
      const X = (x) => L + (x - xMin) / (xMax - xMin) * pw;
      const Y = (y) => T + (1 - (y - yMin) / (yMax - yMin)) * ph;

      // grid + y ticks
      ctx.font = '11px system-ui, sans-serif';
      ctx.textBaseline = 'middle';
      ctx.textAlign = 'right';
      const step = niceStep(yMax - yMin, Math.max(2, Math.floor(ph / 34)));
      ctx.lineWidth = 1;
      for (let v = Math.ceil(yMin / step) * step; v <= yMax + 1e-9; v += step) {
        const y = Math.round(Y(v)) + 0.5;
        ctx.strokeStyle = line;
        ctx.beginPath(); ctx.moveTo(L, y); ctx.lineTo(W - R, y); ctx.stroke();
        ctx.fillStyle = muted;
        ctx.fillText(Math.abs(v) < step * 1e-6 ? '0' : v.toFixed(Math.max(0, -Math.floor(Math.log10(step) + 1e-9))), L - 6, y);
      }
      // x ticks
      ctx.textAlign = 'center'; ctx.textBaseline = 'top';
      const xs = niceStep(xMax - xMin, Math.max(2, Math.floor(pw / 80)));
      for (let v = Math.ceil(xMin / xs) * xs; v <= xMax + 1e-9; v += xs) {
        ctx.fillStyle = muted;
        ctx.fillText(v.toFixed(Math.max(0, -Math.floor(Math.log10(xs) + 1e-9))), X(v), H - B + 5);
      }
      // reference lines
      ctx.setLineDash([4, 4]);
      ctx.strokeStyle = muted;
      for (const h of this.hlines) {
        const y = Math.round(Y(h)) + 0.5;
        ctx.beginPath(); ctx.moveTo(L, y); ctx.lineTo(W - R, y); ctx.stroke();
      }
      ctx.setLineDash([]);

      // series
      const drawRows = (rs, alpha, dashed) => {
        for (let s = 0; s < this.series.length; s++) {
          ctx.strokeStyle = css('--c' + (s + 1));
          ctx.globalAlpha = alpha;
          ctx.lineWidth = dashed ? 1.5 : 2;
          ctx.setLineDash(dashed ? [5, 4] : []);
          ctx.beginPath();
          let pen = false;
          for (const r of rs) {
            const v = r[s + 1];
            if (v === null || v === undefined || Number.isNaN(v)) { pen = false; continue; }
            if (!pen) { ctx.moveTo(X(r[0]), Y(v)); pen = true; } else ctx.lineTo(X(r[0]), Y(v));
          }
          ctx.stroke();
        }
        ctx.globalAlpha = 1;
        ctx.setLineDash([]);
      };
      ctx.save();
      ctx.beginPath(); ctx.rect(L, T, pw, ph); ctx.clip();
      if (ghost) drawRows(ghost, 0.45, true);
      drawRows(rows, 1, false);
      ctx.restore();

      // title + legend
      ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
      ctx.fillStyle = ink; ctx.font = '600 12px system-ui, sans-serif';
      ctx.fillText(this.title, L, 10);
      let lx = L + ctx.measureText(this.title).width + 14;
      ctx.font = '11px system-ui, sans-serif';
      this.series.forEach((s, i) => {
        ctx.fillStyle = css('--c' + (i + 1));
        ctx.fillRect(lx, 6, 10, 4);
        ctx.fillStyle = muted;
        ctx.fillText(s.label, lx + 14, 10);
        lx += 14 + ctx.measureText(s.label).width + 12;
      });
      if (ghost && this.ghostLabel) { ctx.fillStyle = muted; ctx.fillText('- - ' + this.ghostLabel, lx, 10); }

      // hover read-out
      if (this.hoverX !== null && rows.length && this.hoverX >= L && this.hoverX <= W - R) {
        const xv = xMin + (this.hoverX - L) / pw * (xMax - xMin);
        let best = rows[0];
        for (const r of rows) if (Math.abs(r[0] - xv) < Math.abs(best[0] - xv)) best = r;
        const hx = Math.round(X(best[0])) + 0.5;
        ctx.strokeStyle = muted; ctx.lineWidth = 1;
        ctx.beginPath(); ctx.moveTo(hx, T); ctx.lineTo(hx, T + ph); ctx.stroke();
        const txt = (this.xLabel ? this.xLabel + ' ' : 't ') + fmt(best[0]) + '   ' +
          this.series.map((s, i) => s.label + ' ' + fmt(best[i + 1])).join('   ');
        ctx.font = '11px system-ui, sans-serif';
        const tw = ctx.measureText(txt).width + 10;
        const bx = Math.min(Math.max(hx - tw / 2, L), W - R - tw);
        ctx.fillStyle = css('--card'); ctx.strokeStyle = line;
        ctx.fillRect(bx, T + 2, tw, 18); ctx.strokeRect(bx + 0.5, T + 2.5, tw, 18);
        ctx.fillStyle = ink; ctx.textAlign = 'left'; ctx.textBaseline = 'middle';
        ctx.fillText(txt, bx + 5, T + 11);
      }
    }
  }

  window.LineChart = LineChart;
})();
