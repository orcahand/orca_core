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

async function tickHand() {
  const E = await getJSON('/api/hand/ser-7577')
  if (!E) return
  $('hand').hidden = false
  const names = { 'packaged config': 'packaged config (out of the box)', 'standard (current config)': 'own refit stops (current standard)', curve: 'per-joint curve', lut: 'coupled table (LUT)' }
  const keys = Object.keys(E.heldout)
  const J = Object.keys(E.rom)
  $('hand-cap').innerHTML = `Fitted on ${E.n_train} poses (${E.train.join(', ')}), tested on ${E.n_test} held-out poses (${E.test.join(', ')}), ${E.heldout[keys[0]].all.n} joint values. Errors in degrees, true (vision) − map. The maps the dashboard runs (orca-ui :5001, "Visual calibration (study)") are refitted on all ${E.n_train + E.n_test} poses. Created ${E.created}.`
  let h = '<table><thead><tr><th>map (held out)</th><th>RMS</th><th>median</th><th>p95</th><th>max</th><th>params</th></tr></thead><tbody>'
  for (const k of keys) { const a = E.heldout[k].all; h += `<tr><td>${names[k] || k}</td><td>${f2(a.rms)}</td><td>${f2(a.p50)}</td><td>${f1(a.p95)}</td><td>${f1(a.max)}</td><td>${E.params[k]}</td></tr>` }
  $('hand-table').innerHTML = h + '</tbody></table>'
  const cols = [css('--muted'), css('--s2'), css('--s7'), css('--s3')]
  $('hand-bars').innerHTML = ''
  bars($('hand-bars'), {
    cats: J, unit: '°', ylabel: 'held-out RMS °',
    groups: keys.map((k, i) => ({ name: names[k] || k, color: cols[i % cols.length], values: J.map((j) => E.heldout[k][j] && E.heldout[k][j].rms) })),
  })
  const grid = $('hand-grid'); grid.innerHTML = ''
  for (const j of J) {
    const pts = E.points.filter((p) => p.x[j] != null && p.measured[j] != null)
    const C = E.curves[j], rom = E.rom[j]
    const tip = (p) => `${p.session.replace(/^.*-(N\d+.*)$/, '$1')} ${p.id}${p.test ? ' (held out)' : ''}<br>estimate ${f1(p.x[j])}° · measured ${f1(p.measured[j])}°<br>error ${sgn(p.measured[j] - p.x[j])}° · split ${f1(p.split[j])}°`
    const div = document.createElement('div'); div.className = 'panel'
    const rmsOf = (k) => E.heldout[k][j] ? f1(E.heldout[k][j].rms) : '—'
    div.innerHTML = `<h3>${j}</h3>`
    const xv = extent([...pts.map((p) => p.x[j]), ...rom]), yv = extent([...pts.map((p) => p.measured[j]), ...C.curve, ...rom])
    plot(div, {
      h: 200, x: xv, y: yv, xlabel: 'orca_core estimate °', ylabel: 'measured °', label: `${j} measured against estimate`,
      hlines: [{ y: rom[0], label: `limit ${rom[0]}°` }, { y: rom[1], label: `limit ${rom[1]}°` }],
      series: [
        { type: 'line', data: [[rom[0], rom[0]], [rom[1], rom[1]]], color: css('--s2'), dash: '4 3' },
        { type: 'line', data: C.x.map((x, i) => [x, C.curve[i]]), color: css('--s7') },
        { data: pts.filter((p) => !p.test).map((p) => [p.x[j], p.measured[j], tip(p)]), color: css('--s1'), r: 2.6 },
        { data: pts.filter((p) => p.test).map((p) => [p.x[j], p.measured[j], tip(p)]), color: css('--s1'), hollow: true, r: 3 },
      ],
    })
    const p = document.createElement('p'); p.className = 'cap'
    p.textContent = `held-out RMS: standard ${rmsOf('standard (current config)')}° · curve ${rmsOf('curve')}° · LUT ${rmsOf('lut')}°`
    div.appendChild(p)
    grid.appendChild(div)
  }
}
tickHand(); setInterval(tickHand, 120000)

async function tickAdvanced() {
  const hand = $('adv-hand').value
  const A = await getJSON(`/api/advanced/${hand}`)
  if (!A) { $('adv-body').innerHTML = '<p class="cap">Not evaluated for this hand yet.</p>'; return }
  $('advanced').hidden = false
  const P = A.params, J = Object.keys(A.advanced)
  let h = `<p class="cap">Trained on ${A.n_train} poses (${A.train_sessions.join(', ')}). Created ${A.created}.</p>`
  for (const [t, T] of Object.entries(A.tests)) {
    h += `<h3>Held out: ${t} · ${T.n} poses (${T.sessions.join(', ')})</h3><table><thead><tr><th>map</th><th>RMS</th><th>median</th><th>p95</th><th>max</th><th>params</th></tr></thead><tbody>`
    for (const [k, v] of Object.entries(T.maps)) h += `<tr><td>${k}</td><td>${f2(v.all.rms)}</td><td>${f2(v.all.p50)}</td><td>${f1(v.all.p95)}</td><td>${f1(v.all.max)}</td><td>${P[k] ?? '—'}</td></tr>`
    h += '</tbody></table>'
  }
  const [t0, T0] = Object.entries(A.tests)[0]
  const maps = Object.keys(T0.maps)
  h += `<h3>Per joint, held out ${t0}: RMS / median / max</h3><table><thead><tr><th>joint</th>${maps.map((m) => `<th>${m}</th>`).join('')}</tr></thead><tbody>`
  for (const j of J) h += `<tr><td>${j}</td>${maps.map((m) => { const v = T0.maps[m][j]; return `<td>${v ? `${f1(v.rms)} / ${f1(v.p50)} / ${f1(v.max)}` : '—'}</td>` }).join('')}</tr>`
  h += '</tbody></table>'
  h += '<h3>Advanced model per joint</h3><table><thead><tr><th>joint</th><th>fit RMS</th><th>play °</th><th>directions seen</th><th>stretch °/A</th><th>current 5–95 % mA</th><th>coupling (° per ° of)</th></tr></thead><tbody>'
  for (const j of J) {
    const s = A.advanced[j], dirs = A.directions_seen[j]
    h += `<tr><td>${j}</td><td>${f2(s.fit_rms)}</td><td>${dirs.length > 1 ? f1(s.play_deg) : '— (one side)'}</td><td>${dirs.map((d) => (d > 0 ? 'below' : 'above')).join(', ')}</td><td>${f1(s.k_deg_per_a)}</td><td>${A.current_range_ma[j].join(' … ')}</td><td>${Object.entries(s.coupling).map(([p, c]) => `${p} ${c >= 0 ? '+' : ''}${c.toFixed(3)}`).join(', ') || '—'}</td></tr>`
  }
  h += '</tbody></table><h3>Which terms help (held-out RMS with one term left out)</h3><table><thead><tr><th>variant</th>' + Object.keys(A.tests).map((t) => `<th>${t}</th>`).join('') + '</tr></thead><tbody>'
  const full = Object.fromEntries(Object.entries(A.tests).map(([t, T]) => [t, T.maps.advanced.all]))
  for (const [k, d] of [['full advanced model', full], ...Object.entries(A.ablation)]) h += `<tr><td>${k}</td>${Object.keys(A.tests).map((t) => `<td>${f2(d[t].rms)}</td>`).join('')}</tr>`
  h += `</tbody></table><p class="cap">Play and stretch need poses approached from both sides and under load; with one approach side the backlash term is shrunk to 0 and folded into the offset. Inverse check (command → forward, worst on training poses): ${A.inverse_check_deg}°.</p>`
  $('adv-body').innerHTML = h
}
$('adv-hand').addEventListener('change', tickAdvanced)
tickAdvanced(); setInterval(tickAdvanced, 120000)

// ---- protocol night ----
const hm = (t) => (t ? new Date(t * 1000).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' }) : '—')
const dur = (s) => (s == null ? '—' : s >= 3600 ? `${Math.floor(s / 3600)} h ${String(Math.round((s % 3600) / 60)).padStart(2, '0')}` : `${Math.round(s / 60)} min`)
const chip = (s) => `<span class="st ${s}">${s}</span>`
const a4 = (a) => (a ? `${f2(a.rms)} / ${f2(a.p50)} / ${f1(a.p95)} / ${f1(a.max)}` : '—')
let prTags = null

async function tickProtocol() {
  if (!prTags) {
    prTags = await getJSON('/api/protocols')
    if (!prTags.length) return
    const sel = $('pr-tag')
    sel.innerHTML = prTags.map((t) => `<option>${t}</option>`).join('')
    const real = prTags.find((t) => !t.startsWith('dry'))
    if (real) sel.value = real
    const q = new URLSearchParams(location.search).get('night')
    if (q && prTags.includes(q)) sel.value = q
    sel.addEventListener('change', tickProtocol)
  }
  const P = await getJSON(`/api/protocol/${$('pr-tag').value}`)
  $('protocol').hidden = false
  const R = P.results
  $('pr-upd').textContent = `${P.hand} · runner ${P.state}${P.running ? ' (alive)' : ''} · results ${R ? R.updated : 'not yet'}`
  // headline
  let h = ''
  if (R && R.headline) {
    const H = R.headline, best = H.maps[H.best], std = H.maps['own stops'] || H.maps.packaged
    h += `<div class="answer"><b>Positional accuracy of ${P.hand}</b> on ${H.n} held-out poses (${H.set}).<br>
      Joint angle error with orca_core's own stops: <span class="big">${f2(std.deg.all.rms)}°</span> RMS (median ${f2(std.deg.all.p50)}°, p95 ${f1(std.deg.all.p95)}°, max ${f1(std.deg.all.max)}°); fingertip <span class="big">${f1(std.mm.all && std.mm.all.rms)} mm</span> RMS (p95 ${f1(std.mm.all && std.mm.all.p95)} mm).<br>
      Best map (${H.best}): <span class="big">${f2(best.deg.all.rms)}°</span> RMS, fingertip <span class="big">${f1(best.mm.all && best.mm.all.rms)} mm</span> (p95 ${f1(best.mm.all && best.mm.all.p95)} mm).</div>`
    const maps = Object.keys(H.maps)
    h += '<h3>Per joint (RMS / median / p95 / max °)</h3><div class="scroll"><table><thead><tr><th>joint</th>' + maps.map((m) => `<th>${m}</th>`).join('') + '</tr></thead><tbody>'
    for (const j of [...Object.keys(H.maps[maps[0]].deg).filter((k) => k !== 'all'), 'all']) h += `<tr><td>${j}</td>${maps.map((m) => `<td>${a4(H.maps[m].deg[j])}</td>`).join('')}</tr>`
    h += '</tbody></table></div><h3>Fingertip position error (RMS / median / p95 / max mm, forward kinematics)</h3><div class="scroll"><table><thead><tr><th>finger</th>' + maps.map((m) => `<th>${m}</th>`).join('') + '</tr></thead><tbody>'
    for (const f of ['thumb', 'index', 'middle', 'ring', 'pinky', 'all']) h += `<tr><td>${f}</td>${maps.map((m) => `<td>${a4(H.maps[m].mm[f])}</td>`).join('')}</tr>`
    h += '</tbody></table></div>'
  } else {
    h += '<p class="cap">The headline answer (positional accuracy per joint and fingertip) appears once the deep calibration and the held-out sets after the recalibration are solved.</p>'
  }
  $('pr-answer').innerHTML = h
  // status
  const el = P.started ? P.now - P.started : null
  $('pr-head').innerHTML = [
    ['state', `${P.state}${P.running ? '' : P.state === 'running' ? ' (runner gone)' : ''}`],
    ['started', hm(P.started)], ['elapsed', dur(el)], ['ETA end', hm(P.eta_end)],
    ['deadline', hm(P.deadline)], ['s / pose', f1(P.s_per_pose)],
    ['heartbeat', P.heartbeat ? `${Math.round(P.heartbeat.age_s)} s ago` : '—'], ['doing', P.heartbeat ? P.heartbeat.note : '—'],
  ].map(([k, v]) => `<div><div class="k">${k}</div><div class="v">${v}</div></div>`).join('')
  // phases
  h = ''
  const steps = Object.fromEntries(P.steps.map((s) => [s.name, s]))
  for (const ph of P.phases) {
    h += `<div class="phase"><h3>${ph.key} · ${ph.name}</h3>${chip(ph.status)}<span class="cap">ends ~${hm(ph.eta_end)}</span></div>`
    h += '<table><thead><tr><th>step</th><th>status</th><th>what</th><th>poses shot</th><th>solved (BiRefNet / GrabCut / HQ-SAM)</th><th>start</th><th>end</th><th>note</th></tr></thead><tbody>'
    for (const n of ph.steps) {
      const s = steps[n]
      const poses = s.poses != null ? `${s.shot ?? 0}/${s.poses}${s.skipped ? ` (${s.skipped} skipped)` : ''}` : '—'
      const solved = s.poses != null ? `${s.solved ?? 0} / ${(s.solved_by || {}).grabcut ?? 0} / ${(s.solved_by || {}).hqsam ?? 0}` : '—'
      h += `<tr class="${s.status === 'running' || s.status === 'fault' ? 'cur' : ''}"><td>${s.name}</td><td>${chip(s.status)}</td><td>${s.label || ''}</td><td>${poses}</td><td>${solved}</td>
        <td>${hm(s.t0 || s.eta_start)}</td><td>${s.t1 ? hm(s.t1) : '~' + hm(s.eta_end)}</td><td>${s.error || s.note || (s.current ? 'at ' + s.current : '')}</td></tr>`
    }
    h += '</tbody></table>'
  }
  $('pr-phases').innerHTML = h
  // numbers
  h = ''
  if (R && R.as_run) {
    h += '<h3>The hand as run, per held-out set (map in force; RMS / median / p95 / max °, fingertip RMS mm)</h3><table><thead><tr><th>set</th><th>joints °</th><th>fingertip mm</th><th>poses</th></tr></thead><tbody>'
    for (const [k, v] of Object.entries(R.as_run)) h += `<tr><td>${k}</td><td>${a4(v.deg.all)}</td><td>${f1(v.mm.all && v.mm.all.rms)}</td><td>${R.n[k]}</td></tr>`
    h += '</tbody></table>'
  }
  if (R && R.p1) {
    h += '<h3>P1 · V1 scored offline with other hard stops (same photos; RMS / median / p95 / max °, fingertip RMS mm)</h3><table><thead><tr><th>stops</th><th>joints °</th><th>fingertip mm</th></tr></thead><tbody>'
    for (const [k, v] of Object.entries(R.p1.maps)) h += `<tr><td>${k}</td><td>${a4(v.deg.all)}</td><td>${f1(v.mm.all && v.mm.all.rms)}</td></tr>`
    h += '</tbody></table>'
  }
  if (R && R.ablation) {
    const A = R.ablation, sets = Object.keys(A.tests)
    h += `<h3>Map variants on the held-out sets (trained on ${A.train_n} poses of S1 + S2 + deep): joint RMS ° · fingertip RMS mm</h3><div class="scroll"><table><thead><tr><th>map</th><th>params</th>${sets.map((s) => `<th>${s}</th>`).join('')}</tr></thead><tbody>`
    for (const m of A.maps) h += `<tr><td>${m}</td><td>${A.params[m]}</td>${sets.map((s) => { const v = A.tests[s][m]; return `<td>${f2(v.deg.all && v.deg.all.rms)} · ${f1(v.mm.all && v.mm.all.rms)}</td>` }).join('')}</tr>`
    h += '</tbody></table></div>'
  }
  const Q = R && R.quality
  if (Q && Q.noise_floor) {
    const J = Object.keys(Q.noise_floor)
    h += '<h3>Quality</h3><div class="scroll"><table><thead><tr><th>joint</th><th>noise floor (view split, median °)</th><th>repeatability (sd °, V repeats)</th>' + Object.keys(Q.drift || {}).map((k) => `<th>${k} mean °</th>`).join('') + Object.keys((Q.segmenters || {}).joints || {}).map((k) => `<th>${k} vs primary, median |Δ| °</th>`).join('') + '</tr></thead><tbody>'
    for (const j of J) {
      h += `<tr><td>${j}</td><td>${f2(Q.noise_floor[j])}</td><td>${Q.repeatability && Q.repeatability[j] ? f2(Q.repeatability[j].sd) : '—'}</td>`
      h += Object.values(Q.drift || {}).map((d) => `<td>${d[j] ? sgn(d[j].mean) : '—'}</td>`).join('')
      h += Object.values((Q.segmenters || {}).joints || {}).map((d) => `<td>${d[j] ? f2(d[j].p50) : '—'}</td>`).join('') + '</tr>'
    }
    h += '</tbody></table></div>'
    const S = Q.segmenters || {}
    h += `<p class="cap">Silhouette residual (median / p95 px at 960 wide): ${Object.entries(Q.silhouette_px || {}).map(([k, v]) => `${k} ${f2(v.p50)} / ${f1(v.p95)}`).join(' · ')}.<br>
      Segmenters: primary ${S.primary} on ${S.n_primary} poses; residual px ${Object.entries(S.rms_px || {}).map(([k, v]) => `${k} ${f2(v)}`).join(', ')}; mask IoU with the primary ${Object.entries(S.iou || {}).map(([k, v]) => v ? `${k} ${v.p50} (5 % ${v.p05})` : `${k} —`).join(', ') || '—'}.</p>`
  }
  if (R && R.counts) {
    h += '<p class="cap">Overlay sheets (URDF at the solution on the photos): ' + Object.keys(R.counts).map((k) => `<a href="/contact/${P.tag}-${k}.jpg" target="_blank">${k}</a>`).join(' · ') + '</p>'
  }
  if (P.faults && P.faults.length) h += '<h3>Faults</h3><pre class="log">' + P.faults.map((f) => `${hm(f.t)} ${f.where} ${f.joint || ''} ${f.why || ''}`).join('\n') + '</pre>'
  $('pr-numbers').innerHTML = h
  $('pr-log').textContent = P.log.join('\n')
}
tickProtocol(); setInterval(tickProtocol, 10000)
