const $ = (id) => document.getElementById(id)
const f1 = (v) => (v == null || !isFinite(v) ? '—' : v.toFixed(1))
const f2 = (v) => (v == null || !isFinite(v) ? '—' : v.toFixed(2))
const sgn = (v) => (v == null ? '—' : (v >= 0 ? '+' : '') + v.toFixed(1))
let live = { session: null, id: null, stop: 90 }
let report = null

document.getElementById('toc').innerHTML = [...document.querySelectorAll('section')]
  .map((s) => `<a href="#${s.id}">${s.querySelector('h2').textContent}</a>`).join('')

async function getJSON(u) { const r = await fetch(u); return r.json() }

async function tickProgress() {
  try {
    const p = await getJSON('/api/progress')
    const chips = []
    for (const s of p.sessions) {
      const st = s.progress.state || '—'
      const eta = s.progress.last_s && s.progress.todo ? Math.round((s.progress.todo - s.progress.done_now) * s.progress.last_s / 60) : null
      chips.push(`<span class="chip ${st === 'error' ? 'err' : ''}">${s.name.replace(/^\d+-/, '')}: <b>${s.collected}/${s.planned}</b> photographed · <b>${s.solved}</b> solved · ${st}${eta && st === 'running' ? ` · ~${eta} min left` : ''}${s.errors.length ? ` · ${s.errors.length} solve errors` : ''}</span>`)
    }
    const h = p.hand || {}, t = p.turntable || {}
    chips.push(`<span class="chip ${h.torque ? 'on' : ''}">hand torque <b>${h.torque ? 'on' : 'off'}</b></span>`)
    chips.push(`<span class="chip ${t.torque ? 'on' : ''}">plate <b>${t.angle != null ? t.angle.toFixed(1) + '°' : '—'}</b> torque <b>${t.torque ? 'on' : 'off'}</b>${t.busy ? ' · moving' : ''}</span>`)
    $('chips').innerHTML = chips.join('')
    tickSolver(p.sessions).catch(console.error)
    const newest = p.sessions.filter((s) => s.latest).sort((a, b) => (b.progress.started || 0) - (a.progress.started || 0))[0]
    if (newest && (newest.name !== live.session || newest.latest !== live.id)) {
      live.session = newest.name; live.id = newest.latest
      showLive()
    }
  } catch (e) { $('chips').innerHTML = `<span class="chip err">${e}</span>` }
}

async function showLive() {
  const d = await getJSON(`/api/config/${live.session}/${live.id}`)
  const img = (stop) => `/overlay/${live.session}/${live.id}/${stop}.jpg?v=${live.id}`
  $('live-big').src = img(live.stop)
  $('thumbs').innerHTML = d.views.map((v) => `<img src="${img(v.stop)}" data-stop="${v.stop}" class="${v.stop === live.stop ? 'sel' : ''}" alt="plate ${v.stop}°" title="plate ${v.stop}°">`).join('')
  $('thumbs').onclick = (e) => { const s = e.target.dataset.stop; if (s) { live.stop = +s; showLive() } }
  $('live-cap').innerHTML = `Pose <b>${d.id}</b> (${d.kind}). Outlines: <span class="key k-meas">measured (vision)</span> and <span class="key k-ass">assumed by orca_core</span> on each photo; fit RMS ${f1(d.est.rms_px)} px (3840-px image), solved in ${d.est.solve_s} s.`
  const rows = Object.keys(d.q).map((j) => {
    const e = d.est.joints[j] - d.est.assumed[j]
    const noise = Math.max(d.est.split ? Math.abs(d.est.split.even[j] - d.est.split.odd[j]) : 0, 2 * (d.est.std[j] > 0 ? d.est.std[j] : 0), 1.5)
    return `<tr><td>${j}</td><td>${f1(d.q[j])}</td><td>${f1(d.est.assumed[j])}</td><td>${f1(d.est.joints[j])}</td><td class="${Math.abs(e) > noise ? 'bad' : ''}">${sgn(e)}</td><td>${f1(noise)}</td></tr>`
  }).join('')
  $('live-table').innerHTML = `<table><thead><tr><th>joint</th><th>commanded</th><th>assumed</th><th>measured</th><th>error</th><th>noise</th></tr></thead><tbody>${rows}</tbody></table><p class="cap">Error = measured − assumed, in red when larger than the noise (view-split difference, 2σ, or 1.5°, whichever is largest).</p>`
}

function render(R) {
  report = R
  const J = R.joints, run0 = R.runs[0]
  // summary bars
  $('err-bars').innerHTML = ''
  const noise = J.map((j) => R.vision.joints[j].split_med)
  bars($('err-bars'), {
    cats: J, unit: '°', ylabel: 'deg',
    groups: [
      { name: 'RMS error: measured − assumed (orca_core)', color: css('--s2'), values: J.map((j) => R.errors[j].vs_assumed && R.errors[j].vs_assumed.rms) },
      { name: 'RMS error: measured − commanded', color: css('--s7'), values: J.map((j) => R.errors[j].vs_commanded && R.errors[j].vs_commanded.rms) },
      { name: 'vision noise (median view split)', color: css('--muted'), values: noise },
    ],
  })
  const worst = J.map((j) => [j, R.errors[j].vs_assumed ? R.errors[j].vs_assumed.rms : 0]).sort((a, b) => b[1] - a[1]).slice(0, 3)
  $('err-cap').innerHTML = `Over ${R.n} solved poses. <b>Assumed</b> is what orca_core believes from the motor position. <b>Commanded</b> adds the motors' tracking error. A joint matters when its error clearly exceeds the grey vision noise. Largest now: ${worst.map(([j, v]) => `<b>${j}</b> ${v.toFixed(1)}°`).join(', ')}.`

  // maps
  const grid = $('map-grid'); grid.innerHTML = ''
  for (const j of J) {
    const M = R.maps[j][run0]
    if (!M || !M.points.length) continue
    const pts = M.points
    const swept = pts.filter((p) => p.swept), other = pts.filter((p) => !p.swept)
    const div = document.createElement('div'); div.className = 'panel'
    div.innerHTML = `<h3>${j}</h3>`
    const mx = extent([...pts.map((p) => p.m), ...M.motor_limits])
    const yv = extent([...pts.map((p) => p.meas), ...M.rom, M.line[0][1], M.line[1][1]])
    const tip = (p) => `${p.id}<br>motor ${p.m.toFixed(3)} rad · commanded ${f1(p.cmd)}°<br>measured ${f1(p.meas)}° · assumed ${f1(p.assumed)}°<br>error ${sgn(p.meas - p.assumed)}° · split ${f1(p.split)}° · ${f1(p.cur)} mA`
    const stops = ['lo', 'hi'].filter((s) => M.stops[s].measured != null).map((s) => {
      const lim = M.motor_limits[s === 'lo' ? (M.line[0][1] < M.line[1][1] ? 0 : 1) : (M.line[0][1] < M.line[1][1] ? 1 : 0)]
      const near = swept.filter((p) => Math.abs(p.cmd - M.stops[s].configured) < 0.5)
      return near.map((p) => [p.m, p.meas, `real stop (${s}): commanded ${M.stops[s].configured}°, reached ${f1(p.meas)}°`])
    }).flat()
    plot(div, {
      h: 190, x: mx, y: yv, ylabel: 'joint °', label: `${j} joint angle against motor position`,
      hlines: [{ y: M.rom[0], label: `limit ${M.rom[0]}°` }, { y: M.rom[1], label: `limit ${M.rom[1]}°` }],
      series: [
        { type: 'line', data: M.line, color: css('--s2') },
        { data: other.map((p) => [p.m, p.meas, tip(p)]), color: css('--s1'), hollow: true, r: 2.6, opacity: 0.6 },
        { data: swept.map((p) => [p.m, p.meas, tip(p)]), color: css('--s1') },
        { data: stops, color: css('--text'), shape: 'diamond' },
      ],
    })
    const res = pts.map((p) => p.meas - p.assumed)
    const ry = extent([...res, -3, 3])
    plot(div, {
      h: 120, x: mx, y: ry, xlabel: 'motor position (rad)', ylabel: 'resid °', label: `${j} residual`,
      hlines: [{ y: 0, color: css('--s2'), dash: '0' }],
      series: [{ data: pts.map((p) => [p.m, p.meas - p.assumed, tip(p)]), color: css('--s1'), r: 2.6 }],
    })
    const fit = M.fit
    const parts = []
    if (fit && swept.length >= 3 && fit.coverage >= 0.3) {
      const g = fit.gain
      parts.push(`gain ${g.toFixed(2)} (${g > 1 ? 'moves ' + Math.round((g - 1) * 100) + '% more' : 'moves ' + Math.round((1 - g) * 100) + '% less'} than assumed)`)
      parts.push(`at assumed 0° it really is ${sgn(fit.offset_at_zero)}°`)
      const lim = fit.at_motor_limits.slice().sort((a, b) => a - b)
      parts.push(`calibration's hard stops ≈ ${f1(lim[0])}…${f1(lim[1])}° (configured ${M.rom[0]}…${M.rom[1]}°, from the line fit)`)
    }
    const mean = res.reduce((a, b) => a + b, 0) / res.length
    parts.push(`mean error ${sgn(mean)}°, spread ${f1(Math.sqrt(res.reduce((a, b) => a + (b - mean) ** 2, 0) / res.length))}°`)
    for (const s of ['lo', 'hi']) if (M.stops[s].measured != null) parts.push(`stop ${M.stops[s].configured}° → really ${f1(M.stops[s].measured)}°`)
    if (!swept.length) parts.push('not moved yet: the points come from other joints\' poses')
    else if (fit && fit.coverage < 0.3) parts.push(`only ${Math.round(fit.coverage * 100)}% of the range covered: no gain yet`)
    div.insertAdjacentHTML('beforeend', `<p class="pcap">${parts.join(' · ')}</p>`)
    grid.appendChild(div)
  }

  // dependencies
  $('dep-heat').innerHTML = ''; $('dep-grid').innerHTML = ''
  if (R.deps.r.length) {
    const sig = R.deps.t.map((row) => row.map((t) => t != null && Math.abs(t) >= 3))
    const tips = R.deps.t.map((row, i) => row.map((t, k) => t == null ? '' : `t = ${t.toFixed(1)} · ${R.deps.slope10[i][k].toFixed(2)}° per 10°`))
    heat($('dep-heat'), { rows: J, cols: J, values: R.deps.r, sig, tips, label: 'partial correlation of joint errors with other joints' })
    const top = Object.entries(R.deps.pairs).sort((a, b) => Math.abs(b[1].t) - Math.abs(a[1].t))
    const sigs = top.filter(([, v]) => Math.abs(v.t) >= 3)
    $('dep-cap').innerHTML = sigs.length ? `Significant couplings (n = ${R.deps.n}): ${sigs.slice(0, 8).map(([k, v]) => `<b>${k.replace('|', ' ← ')}</b> (r ${v.r.toFixed(2)})`).join(', ')}. Read “a ← b” as: the error of a depends on the angle of b. With few poses these are hints: the sparse run moves several joints together, so some pairs are confounded; the overnight grids and random poses separate them.` : `No significant coupling yet (n = ${R.deps.n}); the grids and random poses add the needed variation.`
    for (const [k, v] of top.slice(0, 12)) {
      const [a, b] = k.split('|')
      const div = document.createElement('div'); div.className = 'panel'
      div.innerHTML = `<h3>${a} error vs ${b}</h3>`
      plot(div, {
        h: 160, x: extent(v.x), y: extent([...v.y_partial, -2, 2]), xlabel: `${b} measured (°)`, ylabel: 'partial err °',
        hlines: [{ y: 0, dash: '0' }],
        series: [{ data: v.x.map((x, i) => [x, v.y_partial[i], `${b} ${f1(x)}° → ${a} error ${f1(v.y[i])}° (partial ${f1(v.y_partial[i])}°)`]), color: Math.abs(v.t) >= 3 ? css('--s1') : css('--muted'), r: 2.6 }],
      })
      div.insertAdjacentHTML('beforeend', `<p class="pcap">r = ${v.r.toFixed(2)}, t = ${v.t.toFixed(1)}${Math.abs(v.t) >= 3 ? ': significant, the lookup table needs this term' : ': not significant'}.</p>`)
      $('dep-grid').appendChild(div)
    }
  } else $('dep-cap').textContent = 'Needs at least 8 solved poses.'

  // repeatability
  const refs = Object.keys(R.repeat.refs)
  if (refs.length) {
    let html = `<table><thead><tr><th>joint</th>${refs.map((r) => `<th>${r}: spread</th><th>${r}: above − below</th>`).join('')}</tr></thead><tbody>`
    for (const j of J) html += `<tr><td>${j}</td>${refs.map((r) => { const x = R.repeat.refs[r][j]; return `<td>${f1(x.sd)}</td><td class="${x.hyst != null && Math.abs(x.hyst) > 3 ? 'bad' : ''}">${sgn(x.hyst)}</td>` }).join('')}</tr>`
    $('rep-table').innerHTML = html + '</tbody></table><p class="cap">Degrees. Spread = standard deviation of the measured angle over all visits. Above − below = hysteresis, in red over 3°.</p>'
  }
  $('hyst-grid').innerHTML = ''
  for (const [j, pts] of Object.entries(R.repeat.sweeps)) {
    const div = document.createElement('div'); div.className = 'panel'
    div.innerHTML = `<h3>${j}: sweep up vs. down</h3>`
    const x = extent(pts.map((p) => p.cmd)), y = extent([...pts.flatMap((p) => [p.up - p.cmd, p.down - p.cmd]), -2, 2])
    plot(div, {
      h: 160, x, y, xlabel: 'commanded (°)', ylabel: 'meas − cmd °', hlines: [{ y: 0, dash: '0' }],
      series: [
        { type: 'line', data: pts.map((p) => [p.cmd, p.up - p.cmd]), color: css('--s1') },
        { data: pts.map((p) => [p.cmd, p.up - p.cmd, `up · cmd ${p.cmd}° → ${f1(p.up)}°`]), color: css('--s1') },
        { type: 'line', data: pts.map((p) => [p.cmd, p.down - p.cmd]), color: css('--s3') },
        { data: pts.map((p) => [p.cmd, p.down - p.cmd, `down · cmd ${p.cmd}° → ${f1(p.down)}°`]), color: css('--s3') },
      ],
    })
    const h = pts.map((p) => p.down - p.up), mh = h.reduce((a, b) => a + b, 0) / h.length
    div.insertAdjacentHTML('beforeend', `<p class="pcap"><span class="key k-meas">up</span> <span class="key k-down">down</span> · mean down − up ${sgn(mh)}°${Math.abs(mh) > 3 ? ': clear hysteresis' : ''}.</p>`)
    $('hyst-grid').appendChild(div)
  }

  // vision
  $('vis-bars').innerHTML = ''; $('view-bars').innerHTML = ''
  bars($('vis-bars'), { cats: J, unit: '°', ylabel: 'deg', groups: [
    { name: 'median view split |even − odd|', color: css('--s1'), values: J.map((j) => R.vision.joints[j].split_med) },
    { name: '90th percentile split', color: css('--s3'), values: J.map((j) => R.vision.joints[j].split_p90) },
    { name: 'median fit σ', color: css('--s4'), values: J.map((j) => R.vision.joints[j].std_med) },
  ] })
  const va = Object.keys(R.vision.view_rms)
  if (va.length) bars($('view-bars'), { w: 520, h: 180, cats: va.map((a) => a + '°'), unit: ' px', ylabel: 'px', groups: [{ name: 'median silhouette misfit per plate stop', color: css('--s1'), values: va.map((a) => R.vision.view_rms[a]) }] })

  $('excluded-note').innerHTML = R.excluded.length ? `<b>${R.excluded.length} pose(s) excluded</b> from all graphs for suspect masks: ${R.excluded.map((x) => x.id).join(', ')}.` : 'No pose excluded: every mask passed the checks.'
  if (R.lut) renderLut(R.lut)
  if (R.mount) renderMount(R.mount)
}

function renderMount(M) {
  const J = report.joints, b = M.bundle, run = report.runs[report.runs.length - 1]
  const q = (i, k) => f2(M.quality[i][k] * 100)
  let html = `<p class="cap">The hand's pose on the plate (the mount: 3 rotations, 3 translations) was solved first from the zero pose alone and used fixed for every pose. Here it is solved once over all ${b ? b.poses.length : '?'} poses together (each pose's joints held at their estimate), and every pose's joints are solved again with it. Old results stay in <code>est_mount0.json</code>.</p>`
  if (b) html += `<p class="cap"><b>Mount change:</b> rotation ${b.change.rotation_deg.map(f2).join(', ')}°, translation ${b.change.translation_mm.map(f2).join(', ')} mm. Silhouette misfit over all poses with the joints held: ${f2(b.rms_before)} → ${f2(b.rms_after)} px.</p>`
  html += `<p class="cap"><b>${M.n} poses solved both ways.</b> Fit ${f2(M.rms_px[0])} → ${f2(M.rms_px[1])} px. Mask not explained by the model ${q(0, 'mask_only')} → ${q(1, 'mask_only')} %, model outside the mask ${q(0, 'model_only')} → ${q(1, 'model_only')} %, finger missed ${q(0, 'finger_missed')} → ${q(1, 'finger_missed')} %. A one-sided edge shift shows as model outside the mask on one side; both falling together means the shift is gone.</p>`
  html += `<table><thead><tr><th>joint</th><th>gain old → new</th><th>offset at 0 old → new (°)</th><th>line rms old → new (°)</th><th>median change of the measured angle (°)</th></tr></thead><tbody>`
  for (const j of J) {
    const v = M.joints[j][run]
    const d = M.median_change_deg[j]
    html += v ? `<tr><td>${j}</td><td>${f2(v.gain[0])} → ${f2(v.gain[1])}</td><td>${sgn(v.offset_at_zero[0])} → ${sgn(v.offset_at_zero[1])}</td><td>${f1(v.rms[0])} → ${f1(v.rms[1])}</td><td class="${d > 1.5 ? 'bad' : ''}">${f1(d)}</td></tr>`
      : `<tr><td>${j}</td><td colspan="4">—</td></tr>`
  }
  $('mount-body').innerHTML = html + '</tbody></table>'
}

function renderLut(L) {
  const J = report.joints
  let html = `<p class="cap learn">${L.summary || ''}</p><div id="lut-bars" class="chart"></div><div id="lut-tip" class="chart"></div>`
  $('lut-body').innerHTML = html
  bars($('lut-bars'), { cats: J, unit: '°', ylabel: 'held-out RMS °', groups: [
    { name: 'orca_core linear map', color: css('--s2'), values: J.map((j) => L.heldout[j] && L.heldout[j].before) },
    { name: 'lookup table', color: css('--s1'), values: J.map((j) => L.heldout[j] && L.heldout[j].after) },
    { name: 'vision noise', color: css('--muted'), values: J.map((j) => report.vision.joints[j].split_med) },
  ] })
  if (L.tips) bars($('lut-tip'), { w: 520, h: 190, cats: Object.keys(L.tips), unit: ' mm', ylabel: 'mm', groups: [
    { name: 'fingertip error, orca_core', color: css('--s2'), values: Object.values(L.tips).map((t) => t.before) },
    { name: 'fingertip error, lookup table', color: css('--s1'), values: Object.values(L.tips).map((t) => t.after) },
  ] })
}

let solverSession = null, solverShown = ''
async function tickSolver(sessions) {
  const sel = $('solver-session')
  const names = sessions.filter((s) => s.solved).map((s) => s.name)
  if (sel.options.length !== names.length) {
    sel.innerHTML = names.map((n) => `<option>${n}</option>`).join('')
    if (!solverSession || !names.includes(solverSession)) solverSession = names.find((n) => n.includes('sparse')) || names[names.length - 1]
    sel.value = solverSession
    sel.onchange = () => { solverSession = sel.value; solverShown = ''; tickSolver(sessions) }
  }
  if (!solverSession) return
  const d = await getJSON(`/api/solved/${solverSession}`)
  const key = solverSession + d.ids.join(',')
  if (key === solverShown) return
  solverShown = key
  $('contact-link').href = `/contact/${solverSession}.jpg`
  $('tiles').innerHTML = d.ids.map((id) => `<img loading="lazy" src="/tile/${solverSession}/${id}.jpg?v=${d.ids.length}" class="${d.suspect[id] ? 'suspect' : ''}" alt="${id}" title="${d.suspect[id] ? 'suspect mask: ' + d.suspect[id].join('; ') : id}">`).join('')
}
$('tiles').onclick = (e) => { if (e.target.tagName === 'IMG') { $('zoom').querySelector('img').src = e.target.src; $('zoom').hidden = false } }
$('zoom').onclick = () => { $('zoom').hidden = true }

async function tickReport() {
  try { const R = await getJSON('/api/report'); if (!R.error) render(R) } catch (e) { console.error(e) }
}
tickProgress(); tickReport()
setInterval(tickProgress, 5000)
setInterval(tickReport, 30000)

async function tickNight() {
  const N = await getJSON('/api/night')
  if (!N) return
  $('night').hidden = false
  const J = report ? report.joints : Object.keys(N.stops)
  const all = (a) => (a && a.all ? `${f2(a.all.rms)} / ${f1(a.all.max)}` : '—')
  let h = `<p class="cap">Calibration runs ${N.runs.A} (before the recalibration) and ${N.runs.B} (after). Errors in degrees, RMS / max over all joints, true (vision) − map. a = old config, b = new hard stops (orca_core's linear map, only the config's joint_roms changed), c = lookup table with couplings (fitted on the dense block)${N.sac ? ', b_sac = new stops refitted with the touch contacts' : ''}.</p>`
  h += '<h3>New hard stops (config joint_roms, old → new)</h3><table><thead><tr><th>joint</th><th>old</th><th>new</th><th>fit rms °</th></tr></thead><tbody>'
  for (const j of J) { const s = N.stops[j]; if (s) h += `<tr><td>${j}</td><td>${s.old.join(' … ')}</td><td>${s.new.join(' … ')}</td><td>${f1(s.rms)}</td></tr>` }
  h += '</tbody></table><h3>Accuracy</h3><table><thead><tr><th>set</th><th>a old</th><th>b new stops</th><th>c LUT</th>' + (N.sac ? '<th>b_sac</th>' : '') + '<th>closed loop</th></tr></thead><tbody>'
  const sets = [['dense (train)', N.accuracy.train_dense], ...Object.entries(N.drift)]
  for (const [n, d] of sets) if (d) h += `<tr><td>${n}</td><td>${all(d.a_old)}</td><td>${all(d.b_new_stops)}</td><td>${all(d.c_lut)}</td>` + (N.sac ? `<td>${all(d.b_sac)}</td>` : '') + `<td>${all(d.closed_loop_vs_wanted)}</td></tr>`
  h += '</tbody></table><p class="cap">V0 is held out (same hand state as the dense block). V1, V2 after 30 / 60 min of open-close cycling, V3 after 1 h loaded work, V4 after orca_core\'s recalibration, V5 after the short visual calibration, V6 = the closed loop: commanded through the new stops, compared with the wanted angles.</p>'
  if (N.kapandji) {
    h += '<h3>Kapandji</h3><table><thead><tr><th>finger</th><th>map</th><th>distance mm</th><th>touch</th><th>force thumb</th><th>force finger</th><th>photo</th></tr></thead><tbody>'
    for (const k of N.kapandji) h += `<tr><td>${k.finger}</td><td>${k.map}</td><td>${f1(k.distance_mm)}</td><td>${k.touch ? 'yes' : 'no'}</td><td>${f2(k.force_thumb)}</td><td>${f2(k.force_finger)}</td><td><a href="/report/${k.photo}" target="_blank">photo</a></td></tr>`
    h += '</tbody></table><p class="cap">Distance: thumb tip to fingertip on the URDF at the solved joint angles (5 views); touch: a fingertip or thumb pad above 0.12 (tactile units).</p>'
  }
  if (N.sac) {
    h += `<h3>Super Advanced Calibration</h3><p class="cap">${N.sac.contacts} touch contacts. Model gap at the contact onset (mm), new stops → SAC stops: ${N.sac.gap_before_mm.map(f1).join(', ')} → ${N.sac.gap_after_mm.map(f1).join(', ')}.</p>`
    if (N.sac.loo) h += '<p class="cap">Leave-one-out gaps (mm, without SAC → SAC fitted without that contact): ' + N.sac.loo.map((l) => `${l.finger} ${f1(l.gap_no_sac)}→${f1(l.gap_loo)}`).join(', ') + '</p>'
  }
  $('night-body').innerHTML = h
}
tickNight(); setInterval(tickNight, 60000)
