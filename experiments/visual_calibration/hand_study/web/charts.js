// Small SVG charts: scatter/line plots, grouped bars, a heatmap. Hover tooltips via data-tip.
const NS = 'http://www.w3.org/2000/svg'
const css = (v) => getComputedStyle(document.documentElement).getPropertyValue(v).trim()

function el(tag, attrs = {}, parent) {
  const e = document.createElementNS(NS, tag)
  for (const k in attrs) e.setAttribute(k, attrs[k])
  if (parent) parent.appendChild(e)
  return e
}

function nice(lo, hi, n = 5) {
  if (!(hi > lo)) { hi = lo + 1; lo -= 1 }
  const step0 = (hi - lo) / n
  const mag = Math.pow(10, Math.floor(Math.log10(step0)))
  const step = [1, 2, 2.5, 5, 10].map((m) => m * mag).find((s) => s >= step0)
  const ticks = []
  for (let t = Math.ceil(lo / step) * step; t <= hi + 1e-9; t += step) ticks.push(+t.toFixed(10))
  return ticks
}

function extent(vals, pad = 0.05) {
  const v = vals.filter((x) => x != null && isFinite(x))
  if (!v.length) return [0, 1]
  let lo = Math.min(...v), hi = Math.max(...v)
  if (hi - lo < 1e-9) { lo -= 1; hi += 1 }
  const p = (hi - lo) * pad
  return [lo - p, hi + p]
}

function plot(target, o) {
  const W = o.w || 340, H = o.h || 200, m = { l: 40, r: 10, t: 8, b: 30, ...(o.m || {}) }
  const svg = el('svg', { viewBox: `0 0 ${W} ${H}`, role: 'img', 'aria-label': o.label || '' })
  const xs = (x) => m.l + (x - o.x[0]) / (o.x[1] - o.x[0]) * (W - m.l - m.r)
  const ys = (y) => H - m.b - (y - o.y[0]) / (o.y[1] - o.y[0]) * (H - m.t - m.b)
  for (const t of nice(o.y[0], o.y[1], o.yt || 4)) {
    el('line', { x1: m.l, x2: W - m.r, y1: ys(t), y2: ys(t), class: 'gl' }, svg)
    el('text', { x: m.l - 4, y: ys(t) + 3, 'text-anchor': 'end' }, svg).textContent = t
  }
  for (const t of nice(o.x[0], o.x[1], o.xt || 5)) {
    el('text', { x: xs(t), y: H - m.b + 12, 'text-anchor': 'middle' }, svg).textContent = t
  }
  el('line', { x1: m.l, x2: W - m.r, y1: H - m.b, y2: H - m.b, class: 'ax' }, svg)
  if (o.xlabel) el('text', { x: (m.l + W - m.r) / 2, y: H - 3, 'text-anchor': 'middle' }, svg).textContent = o.xlabel
  if (o.ylabel) el('text', { x: 10, y: (m.t + H - m.b) / 2, 'text-anchor': 'middle', transform: `rotate(-90 10 ${(m.t + H - m.b) / 2})` }, svg).textContent = o.ylabel
  const clip = (y) => Math.max(o.y[0], Math.min(o.y[1], y))
  for (const h of o.hlines || []) {
    if (h.y < o.y[0] || h.y > o.y[1]) continue
    el('line', { x1: m.l, x2: W - m.r, y1: ys(h.y), y2: ys(h.y), stroke: h.color || css('--muted'), 'stroke-dasharray': h.dash || '4 3', 'stroke-width': 1 }, svg)
    if (h.label) el('text', { x: W - m.r - 2, y: ys(h.y) - 3, 'text-anchor': 'end' }, svg).textContent = h.label
  }
  for (const s of o.series) {
    const color = s.color || css('--s1')
    if (s.type === 'line') {
      const d = s.data.filter((p) => p[1] != null).map((p, i) => `${i ? 'L' : 'M'}${xs(p[0]).toFixed(1)},${ys(clip(p[1])).toFixed(1)}`).join('')
      el('path', { d, fill: 'none', stroke: color, 'stroke-width': s.width || 2, 'stroke-dasharray': s.dash || '', 'stroke-linejoin': 'round', 'stroke-linecap': 'round' }, svg)
    } else {
      for (const p of s.data) {
        if (p[1] == null || p[0] == null) continue
        const cx = xs(p[0]), cy = ys(clip(p[1]))
        const attrs = { 'data-tip': p[2] || '', fill: s.hollow ? css('--surface') : color, stroke: s.hollow ? color : css('--surface'), 'stroke-width': s.hollow ? 1.5 : 1, opacity: s.opacity || 0.9 }
        if (s.shape === 'diamond') el('path', { ...attrs, d: `M${cx},${cy - 5}L${cx + 5},${cy}L${cx},${cy + 5}L${cx - 5},${cy}Z` }, svg)
        else el('circle', { ...attrs, cx, cy, r: s.r || 3.2 }, svg)
      }
    }
  }
  target.appendChild(svg)
  return svg
}

function bars(target, o) {
  const W = o.w || 900, H = o.h || 220, m = { l: 42, r: 8, t: 10, b: 54 }
  const svg = el('svg', { viewBox: `0 0 ${W} ${H}`, role: 'img', 'aria-label': o.label || '' })
  const vals = o.groups.flatMap((g) => g.values).filter((v) => v != null)
  const top = Math.max(1e-9, ...vals) * 1.08
  const ys = (y) => H - m.b - y / top * (H - m.t - m.b)
  for (const t of nice(0, top, 4)) {
    el('line', { x1: m.l, x2: W - m.r, y1: ys(t), y2: ys(t), class: 'gl' }, svg)
    el('text', { x: m.l - 4, y: ys(t) + 3, 'text-anchor': 'end' }, svg).textContent = t
  }
  if (o.ylabel) el('text', { x: 10, y: (m.t + H - m.b) / 2, 'text-anchor': 'middle', transform: `rotate(-90 10 ${(m.t + H - m.b) / 2})` }, svg).textContent = o.ylabel
  const cw = (W - m.l - m.r) / o.cats.length, n = o.groups.length
  const bw = Math.min(14, (cw - 6) / n - 2)
  o.cats.forEach((c, i) => {
    const x0 = m.l + i * cw + (cw - n * (bw + 2)) / 2
    o.groups.forEach((g, k) => {
      const v = g.values[i]
      if (v == null) return
      const x = x0 + k * (bw + 2), y = ys(v), h = H - m.b - y
      el('path', { d: `M${x},${H - m.b}V${y + Math.min(3, h)}q0,-3 3,-3h${bw - 6}q3,0 3,3V${H - m.b}Z`, fill: g.color, 'data-tip': `${c} · ${g.name}: ${v.toFixed(2)}${o.unit || ''}` }, svg)
    })
    const t = el('text', { x: m.l + i * cw + cw / 2, y: H - m.b + 10, 'text-anchor': 'end', transform: `rotate(-40 ${m.l + i * cw + cw / 2} ${H - m.b + 10})` }, svg)
    t.textContent = c
  })
  el('line', { x1: m.l, x2: W - m.r, y1: H - m.b, y2: H - m.b, class: 'ax' }, svg)
  const lg = el('g', {}, svg)
  let lx = m.l
  for (const g of o.groups) {
    el('rect', { x: lx, y: 0, width: 10, height: 10, rx: 2, fill: g.color }, lg)
    const t = el('text', { x: lx + 14, y: 9 }, lg); t.textContent = g.name
    lx += 22 + g.name.length * 6
  }
  target.appendChild(svg)
}

function heat(target, o) {
  const n = o.rows.length, c = 22, m = { l: 80, t: 80 }
  const W = m.l + o.cols.length * c + 10, H = m.t + n * c + 10
  const svg = el('svg', { viewBox: `0 0 ${W} ${H}`, role: 'img', 'aria-label': o.label || '', style: `max-width:${W * 1.4}px` })
  const neg = css('--div-neg'), mid = css('--div-mid'), pos = css('--div-pos')
  const mix = (a, b, f) => { const pa = [1, 3, 5].map((i) => parseInt(a.slice(i, i + 2), 16)), pb = [1, 3, 5].map((i) => parseInt(b.slice(i, i + 2), 16)); return `rgb(${pa.map((v, i) => Math.round(v + (pb[i] - v) * f)).join(',')})` }
  o.cols.forEach((cname, j) => {
    const t = el('text', { x: m.l + j * c + c / 2, y: m.t - 4, transform: `rotate(-55 ${m.l + j * c + c / 2} ${m.t - 4})` }, svg); t.textContent = cname
  })
  o.rows.forEach((rname, i) => {
    const t = el('text', { x: m.l - 4, y: m.t + i * c + c / 2 + 3, 'text-anchor': 'end' }, svg); t.textContent = rname
    o.cols.forEach((cname, j) => {
      const v = o.values[i][j]
      const fill = v == null ? css('--bg') : v < 0 ? mix(mid, neg, Math.min(1, -v)) : mix(mid, pos, Math.min(1, v))
      el('rect', { x: m.l + j * c + 1, y: m.t + i * c + 1, width: c - 2, height: c - 2, rx: 3, fill, 'data-tip': v == null ? `${rname} × ${cname}: —` : `error of ${rname} vs ${cname}: r = ${v.toFixed(2)}${o.tips ? ' · ' + o.tips[i][j] : ''}` }, svg)
      if (o.sig && o.sig[i][j]) el('circle', { cx: m.l + j * c + c / 2, cy: m.t + i * c + c / 2, r: 3, fill: css('--text'), 'pointer-events': 'none' }, svg)
    })
  })
  target.appendChild(svg)
}

document.addEventListener('mousemove', (e) => {
  const tip = document.getElementById('tip')
  const t = e.target.closest && e.target.closest('[data-tip]')
  if (!t || !t.getAttribute('data-tip')) { tip.hidden = true; return }
  tip.innerHTML = t.getAttribute('data-tip')
  tip.hidden = false
  tip.style.left = Math.min(e.clientX + 14, innerWidth - 290) + 'px'
  tip.style.top = (e.clientY + 14) + 'px'
})
