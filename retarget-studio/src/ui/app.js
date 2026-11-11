// Workbench application controller. Wires the engine to two SkeletonViewers,
// playback transport, mapping/rest-pose inspection, the validation suite,
// config save/load and GLB/JSON export.

import { sourceSkeleton, targetSkeleton, defaultMapping } from '../data/skeletons.js';
import { buildSourceClip } from '../data/clip.js';
import { defaultConstraintsFor } from '../data/constraints.js';
import { loadSkeleton, canonicalRestPose, fk } from '../engine/skeleton.js';
import { retargetClip, sampleClip, analyzeMapping } from '../engine/retarget.js';
import { describeFrame, toCanonicalVec, toCanonicalQuat } from '../engine/frame.js';
import { runAllChecks } from '../engine/validation.js';
import { aggregateEvents } from '../engine/constraints.js';
import { defaultConfig, serializeConfig, parseConfig } from '../engine/config.js';
import { qdist, vdist, deg, slerp } from '../engine/vecquat.js';
import { SkeletonViewer } from '../three/viewer.js';
import { exportGLB, exportJSON, downloadBlob } from '../three/exporter.js';

const $ = (id) => document.getElementById(id);

const state = {
  config: { ...defaultConfig(), mapping: { ...defaultMapping } },
  src: null, tgt: null, srcRest: null, tgtRest: null,
  srcClip: null, retarget: null, info: null,
  srcViewer: null, tgtViewer: null,
  playing: false, time: 0, lastTs: 0,
  selectedTarget: null,
};

function statusMaps(info) {
  // Target joint status: unmapped if no source, root if root, else mapped.
  const tgtStatus = new Map();
  for (const name of state.tgt.ordered) {
    if (info.unmappedTargets.includes(name)) tgtStatus.set(name, 'unmapped');
    else if (state.tgt.byName.get(name).parent === null) tgtStatus.set(name, 'root');
    else tgtStatus.set(name, 'mapped');
  }
  // Source joint status: skipped if no target references it.
  const srcStatus = new Map();
  for (const name of state.src.ordered) {
    srcStatus.set(name, info.skippedSources.includes(name) ? 'skipped'
      : (state.src.byName.get(name).parent === null ? 'root' : 'mapped'));
  }
  return { tgtStatus, srcStatus };
}

// Constraint settings for the CURRENT target skeleton, or null when disabled
// (null == the initial unconstrained retarget, bit-identical).
function activeConstraintSettings() {
  const c = state.config.constraints;
  if (!c?.enabled) return null;
  return c.targets[state.tgt.name] || null;
}

// Fill defaults for the current target the first time constraints are used.
function ensureTargetConstraints() {
  const targets = state.config.constraints.targets;
  if (!targets[state.tgt.name]) {
    targets[state.tgt.name] = defaultConstraintsFor(state.tgt.name) || {
      limits: {}, contacts: [],
      ik: { maxIterations: 16, tolerance: 1e-3, minImprovement: 1e-6, maxFrameStepDeg: 12 },
    };
  }
  return targets[state.tgt.name];
}

function rebuild() {
  const cfg = state.config;
  state.retarget = retargetClip(sourceSkeleton, targetSkeleton, state.srcClip,
    cfg.mapping, {
      policy: cfg.policy,
      rootGroundOffset: cfg.rootGroundOffset,
      constraints: activeConstraintSettings(),
    });
  state.info = analyzeMapping(sourceSkeleton, targetSkeleton, cfg.mapping);
  renderMappingTable();
  renderConstraints();
  $('root-scale').textContent = state.retarget.rootScale.toFixed(4);
  $('mapping-summary').textContent =
    `目标独有（无驱动）: ${state.info.unmappedTargets.join(', ') || '无'} · ` +
    `源被跳过: ${state.info.skippedSources.join(', ') || '无'}`;
}

function buildRigs() {
  const { tgtStatus, srcStatus } = statusMaps(state.info);
  state.srcViewer.buildRig(state.src, state.srcRest, srcStatus);
  state.tgtViewer.buildRig(state.tgt, state.tgtRest, tgtStatus);
  state.srcViewer.frameCamera([0, 1, 0], 1.3);
  state.tgtViewer.frameCamera([0, 1, 0], 1.3);
}

// Evaluate both skeletons at the current time.
function updatePose() {
  const rt = state.retarget;
  // Source pose at the same normalized time.
  const srcSample = sampleSource(state.time);
  state.srcViewer.update(srcSample.pose);
  const tgtSample = sampleClip(rt, state.time);
  state.tgtViewer.update(tgtSample.pose);
  if (state.config.ghostSource) {
    state.tgtViewer.setGhost(state.src, state.srcRest, srcSample.pose);
  } else {
    state.tgtViewer.clearGhost();
  }
  updateConstraintNow();
  updateSelectionDetail(tgtSample, srcSample);
}

// Source sampling evaluates FK directly from the authored clip, using the same
// canonical conversion and sign-safe interpolation as the retarget pipeline.
let _sourceSampler;
function sourceSampler() {
  if (_sourceSampler) return _sourceSampler;
  const clip = state.srcClip;
  const duration = clip.duration;
  function sampleCanonicalSource(t) {
    const loop = state.config.playback.loop;
    const time = loop ? ((t % duration) + duration) % duration : Math.min(t, duration);
    let i = 0;
    while (i < clip.frames.length - 2 && clip.frames[i + 1].time <= time) i++;
    const a = clip.frames[i];
    const b = clip.frames[Math.min(i + 1, clip.frames.length - 1)];
    const alpha = Math.max(0, Math.min(1, (time - a.time) / (b.time - a.time || 1)));
    const rootPos = lerpVec(
      toCanonicalVec(state.src.frame, a.rootPos),
      toCanonicalVec(state.src.frame, b.rootPos), alpha);
    const rootQuat = slerp(
      toCanonicalQuat(state.src.frame, a.rootRot),
      toCanonicalQuat(state.src.frame, b.rootRot), alpha);
    const localQuats = new Map();
    const rootName = state.src.ordered[0];
    for (const name of state.src.ordered) {
      if (name === rootName) continue;
      localQuats.set(name, slerp(
        toCanonicalQuat(state.src.frame, a.locals[name]),
        toCanonicalQuat(state.src.frame, b.locals[name]), alpha));
    }
    const pose = fk(state.src, state.srcRest, localQuats, rootPos, rootQuat);
    return { time, rootPos, rootQuat, localQuats, pose };
  }
  _sourceSampler = { sampleCanonicalSource };
  return _sourceSampler;
}

function sampleSource(time) {
  return sourceSampler().sampleCanonicalSource(time);
}

function lerpVec(a, b, t) {
  return [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t];
}

// ---- mapping table ----------------------------------------------------------

function renderMappingTable() {
  const tbody = document.querySelector('#mapping-table tbody');
  tbody.innerHTML = '';
  const info = state.info;
  const map = state.config.mapping;

  for (const name of state.tgt.ordered) {
    const tr = document.createElement('tr');
    if (name === state.selectedTarget) tr.classList.add('selected');
    const source = map[name] || '';
    const row = info.rows.find((r) => r.target === name);
    const isUnmapped = info.unmappedTargets.includes(name);
    tr.innerHTML = `
      <td>${name}</td>
      <td class="${isUnmapped ? 'tag-unmapped' : 'tag-mapped'}">${source || '— 无对应 —'}</td>
      <td>${row ? row.worldRestDeltaDeg.toFixed(1) : '—'}</td>
      <td>${row && row.directionRestDeltaDeg !== null ? row.directionRestDeltaDeg.toFixed(1) : '—'}</td>
      <td>${row ? row.lengthRatio.toFixed(2) : '—'}</td>`;
    tr.addEventListener('click', () => selectTarget(name));
    tbody.appendChild(tr);
  }

  // Source-skipped summary rows appended visually via hint; also fill the
  // mapping editor dropdowns.
  const tgtSel = $('edit-target');
  tgtSel.innerHTML = state.tgt.ordered.map((n) => `<option value="${n}">${n}</option>`).join('');
  const srcSel = $('edit-source');
  const srcOptions = ['<option value="">（无 / 取消映射）</option>']
    .concat(state.src.ordered.map((n) => {
      const skipped = info.skippedSources.includes(n);
      return `<option value="${n}">${n}${skipped ? '（当前未用）' : ''}</option>`;
    }));
  srcSel.innerHTML = srcOptions.join('');
  if (state.selectedTarget) {
    tgtSel.value = state.selectedTarget;
    srcSel.value = state.config.mapping[state.selectedTarget] || '';
  }
}

function selectTarget(name) {
  state.selectedTarget = name;
  $('edit-target').value = name;
  $('edit-source').value = state.config.mapping[name] || '';
  renderMappingTable();
  updateSelectionDetail(lastTgtSample, lastSrcSample);
}

let lastTgtSample = null;
let lastSrcSample = null;

function updateSelectionDetail(tgtSample, srcSample) {
  lastTgtSample = tgtSample;
  lastSrcSample = srcSample;
  const name = state.selectedTarget;
  const el = $('sel-detail');
  if (!name) { el.innerHTML = ''; return; }
  const p = tgtSample.pose.worldPos.get(name);
  const q = tgtSample.pose.worldQuat.get(name);
  const sourceName = state.config.mapping[name];
  let srcLine = '该目标关节无源对应（按策略跳过或补偿）。';
  if (sourceName && srcSample.pose.worldPos.has(sourceName)) {
    const sp = srcSample.pose.worldPos.get(sourceName);
    const sq = srcSample.pose.worldQuat.get(sourceName);
    srcLine = `
      <div class="kv"><span>源 ${sourceName} 位置 (m)</span><code>${fmtV(sp)}</code></div>
      <div class="kv"><span>源世界朝向</span><code>${fmtQ(sq)}</code></div>
      <div class="kv"><span>世界朝向误差 (°)</span><code>${deg(qdist(q, sq)).toFixed(2)}</code></div>`;
  }
  el.innerHTML = `
    <div class="kv"><span>目标关节</span><code>${name}</code></div>
    <div class="kv"><span>世界位置 (m)</span><code>${fmtV(p)}</code></div>
    <div class="kv"><span>世界朝向</span><code>${fmtQ(q)}</code></div>
    <div class="kv"><span>到父节点距离 (m)</span><code>${parentDist(name, tgtSample).toFixed(4)}</code></div>
    ${srcLine}`;
}

function parentDist(name, sample) {
  const j = state.tgt.byName.get(name);
  if (j.parent === null) return 0;
  return vdist(sample.pose.worldPos.get(j.parent), sample.pose.worldPos.get(name));
}
const fmtV = (v) => `[${v.map((x) => x.toFixed(3)).join(', ')}]`;
const fmtQ = (q) => `[${q.map((x) => x.toFixed(3)).join(', ')}]`;

// ---- constraints panel ------------------------------------------------------

function frameIndexAt(rt, t) {
  const { frames, duration, loop } = rt;
  let time = t;
  if (loop) time = ((t % duration) + duration) % duration;
  else time = Math.min(t, duration);
  let i = 0;
  while (i < frames.length - 2 && frames[i + 1].time <= time) i++;
  return i;
}

function seekTo(t) {
  state.time = t;
  $('scrub').value = Math.round((t / state.retarget.duration) * 1000);
  $('time-val').textContent = t.toFixed(2);
}

const AXIS_LABELS = ['X', 'Y', 'Z'];
function axisLabel(axis) {
  const i = axis.indexOf(Math.max(...axis.map(Math.abs)));
  const sign = axis[i] < 0 ? '−' : '';
  return `${sign}${AXIS_LABELS[i]} [${axis.map((x) => Number(x.toFixed(2))).join(',')}]`;
}

// Re-render the whole constraints panel from config + the latest report.
function renderConstraints() {
  const c = state.config.constraints;
  const body = $('constraints-body');
  $('constraints-enabled').checked = !!c.enabled;
  body.hidden = !c.enabled;
  if (!c.enabled) return;

  const settings = state.config.constraints.targets[state.tgt.name];
  $('constraints-target-name').textContent = state.tgt.name;
  if (!settings) {
    // Enabled but this target has no settings yet: the retarget ran
    // unconstrained. Offer to materialize defaults.
    $('limits-table').querySelector('tbody').innerHTML = '';
    $('contacts-table').querySelector('tbody').innerHTML = '';
    $('constraint-events').innerHTML =
      '<p class="hint">该角色尚无约束设置，点击“恢复该角色默认”生成。</p>';
    $('constraint-stats').textContent = '';
    return;
  }

  // Limit rows.
  const limitBody = $('limits-table').querySelector('tbody');
  limitBody.innerHTML = '';
  for (const joint of Object.keys(settings.limits)) {
    settings.limits[joint].forEach((spec, ai) => {
      const tr = document.createElement('tr');
      tr.innerHTML = `
        <td>${joint}</td>
        <td>${axisLabel(spec.axis)}</td>
        <td><input type="number" step="1" value="${spec.min}" data-j="${joint}" data-i="${ai}" data-k="min" /></td>
        <td><input type="number" step="1" value="${spec.max}" data-j="${joint}" data-i="${ai}" data-k="max" /></td>
        <td><button class="btn danger-mini" data-del-limit="${joint}|${ai}">删</button></td>`;
      limitBody.appendChild(tr);
    });
  }
  limitBody.querySelectorAll('input').forEach((inp) => {
    inp.addEventListener('change', () => {
      const spec = settings.limits[inp.dataset.j][Number(inp.dataset.i)];
      spec[inp.dataset.k] = Number(inp.value) || 0;
      rebuild();
    });
  });
  limitBody.querySelectorAll('[data-del-limit]').forEach((btn) => {
    btn.addEventListener('click', () => {
      const [joint, ai] = btn.dataset.delLimit.split('|');
      settings.limits[joint].splice(Number(ai), 1);
      if (!settings.limits[joint].length) delete settings.limits[joint];
      rebuild();
    });
  });

  // Contact rows.
  const contactBody = $('contacts-table').querySelector('tbody');
  contactBody.innerHTML = '';
  settings.contacts.forEach((ct, ci) => {
    const tr = document.createElement('tr');
    const params = ct.mode === 'point'
      ? `目标 <input type="text" class="ct-target" value="${(ct.target || [0, 0, 0]).join(',')}" />`
      : `地面Y <input type="number" class="ct-ground" step="0.01" value="${ct.groundY}" />
         <label class="check"><input type="checkbox" class="ct-pin" ${ct.pin !== false ? 'checked' : ''}/>钉扎</label>`;
    tr.innerHTML = `
      <td><input type="checkbox" class="ct-enabled" ${ct.enabled !== false ? 'checked' : ''} /></td>
      <td><code>${ct.id}</code><br/><span class="hint">${ct.effector}</span></td>
      <td><input type="text" class="chain-input ct-chain" value="${(ct.chain || []).join(',')}" /></td>
      <td><input type="number" class="prio ct-prio" step="1" value="${ct.priority}" /></td>
      <td class="ct-params">${params}</td>
      <td><button class="btn danger-mini" data-del-contact="${ci}">删</button></td>`;
    contactBody.appendChild(tr);
    tr.querySelector('.ct-enabled').addEventListener('change', (e) => {
      ct.enabled = e.target.checked; rebuild();
    });
    tr.querySelector('.ct-chain').addEventListener('change', (e) => {
      ct.chain = e.target.value.split(',').map((s) => s.trim()).filter(Boolean);
      rebuild();
    });
    tr.querySelector('.ct-prio').addEventListener('change', (e) => {
      ct.priority = Number(e.target.value) || 0; rebuild();
    });
    tr.querySelector('.ct-ground')?.addEventListener('change', (e) => {
      ct.groundY = Number(e.target.value) || 0; rebuild();
    });
    tr.querySelector('.ct-pin')?.addEventListener('change', (e) => {
      ct.pin = e.target.checked; rebuild();
    });
    tr.querySelector('.ct-target')?.addEventListener('change', (e) => {
      const v = e.target.value.split(',').map((s) => Number(s.trim()));
      if (v.length === 3 && v.every(Number.isFinite)) { ct.target = v; rebuild(); }
    });
    tr.querySelector('[data-del-contact]').addEventListener('click', () => {
      settings.contacts.splice(ci, 1); rebuild();
    });
  });

  // IK solver inputs.
  $('ik-maxiter').value = settings.ik?.maxIterations ?? 16;
  $('ik-tol').value = settings.ik?.tolerance ?? 1e-3;
  $('ik-maxstep').value = settings.ik?.maxFrameStepDeg ?? 0;

  // Add-row selects.
  $('limit-joint').innerHTML = state.tgt.ordered
    .filter((n) => state.tgt.byName.get(n).parent !== null)
    .map((n) => `<option value="${n}">${n}</option>`).join('');
  $('contact-effector').innerHTML = $('limit-joint').innerHTML;

  renderConstraintEvents();
}

function renderConstraintEvents() {
  const host = $('constraint-events');
  const rep = state.retarget?.constraintReport;
  host.innerHTML = '';
  if (!rep) {
    $('constraint-stats').textContent = state.config.constraints.enabled
      ? '该目标角色未配置约束，当前为无约束结果。' : '';
    return;
  }
  const s = rep.stats;
  const acts = s.contactActivations.map((a) => `${a.id}×${a.activations}`).join('，') || '无';
  $('constraint-stats').textContent =
    `限制触发 ${s.clampCount} 次 · 接触激活 ${acts} · 最大迭代 ${s.maxIterations} · 限速帧 ${s.rateLimited}`
    + (rep.warnings.length ? ` ｜ 警告：${rep.warnings.join('；')}` : '');
  for (const e of aggregateEvents(rep)) {
    const div = document.createElement('div');
    div.className = 'event-item';
    const desc = e.type === 'limit'
      ? `<span class="tag-limit">限制</span> ${e.joint} 轴${e.axisIndex} 峰值超出 ${deg(e.maxValue).toFixed(1)}°`
      : `<span class="tag-contact">接触</span> ${e.id}（优先级 ${e.priority}）峰值残差 ${(e.maxValue * 1000).toFixed(1)}mm`;
    div.innerHTML = `<span>${desc}</span><code>${e.t0.toFixed(2)}–${e.t1.toFixed(2)}s · ${e.frames}帧</code>`;
    div.title = '点击定位到峰值帧';
    div.addEventListener('click', () => seekTo(e.tPeak));
    host.appendChild(div);
  }
  if (!host.children.length) host.innerHTML = '<p class="hint">本片无约束触发。</p>';
}

// Per-frame constraint state: residuals with priorities, plus viewport
// emphasis on the exact joints involved. Updated on every pose change.
function updateConstraintNow() {
  const host = $('constraint-now');
  const rep = state.retarget?.constraintReport;
  if (!rep) {
    setNowHtml(host, '<p class="hint">约束未启用，当前为初版无约束结果。</p>');
    state.tgtViewer.setJointEmphasis(null);
    state.tgtViewer.setMarkers([]);
    return;
  }
  const fr = rep.perFrame[frameIndexAt(state.retarget, state.time)];
  const emphasis = new Map();
  const markers = [];
  const rows = [];
  for (const l of fr.limits) {
    rows.push(`<div class="kv"><span>限制触发 ${l.joint} 轴${l.axisIndex}</span>`
      + `<code class="hit">超出 ${deg(l.excessRad).toFixed(2)}°</code></div>`);
    emphasis.set(l.joint, 0xe0b13e);
  }
  const settings = state.config.constraints.targets[state.tgt.name];
  for (const c of fr.contacts) {
    if (!c.active) {
      // Inactive but still bleeding off a correction under the rate cap.
      if (c.rateLimited) {
        rows.push(`<div class="kv"><span>接触 ${c.id}（优先级 ${c.priority}）</span>`
          + '<code class="ok">修正释放中 · 限速中</code></div>');
      }
      continue;
    }
    const spec = settings?.contacts.find((x) => x.id === c.id);
    if (spec?.effector) emphasis.set(spec.effector, 0xff6b6b);
    if (c.target) markers.push({ position: c.target, residual: c.residual });
    const cls = c.residual > 1e-3 ? 'hit' : 'ok';
    const extra = c.rateLimited ? ' · 限速中' : '';
    rows.push(`<div class="kv"><span>接触 ${c.id}（优先级 ${c.priority}）</span>`
      + `<code class="${cls}">残差 ${(c.residual * 1000).toFixed(1)}mm${extra}</code></div>`);
  }
  setNowHtml(host, rows.join('') || '<p class="hint">当前帧无约束触发。</p>');
  state.tgtViewer.setJointEmphasis(emphasis);
  state.tgtViewer.setMarkers(markers);
}

// Cache the per-frame status HTML: updatePose runs every animation frame.
const _nowCache = { html: null };
function setNowHtml(host, html) {
  if (_nowCache.html === html) return;
  _nowCache.html = html;
  host.innerHTML = html;
}

// ---- validation -------------------------------------------------------------

function runChecks() {
  const results = runAllChecks(sourceSkeleton, targetSkeleton, state.srcClip,
    state.config.mapping, {
      policy: state.config.policy,
      rootGroundOffset: state.config.rootGroundOffset,
      constraints: activeConstraintSettings(),
    });
  const host = $('check-results');
  host.innerHTML = '';
  let allPass = true;
  for (const r of results) {
    if (!r.pass) allPass = false;
    const div = document.createElement('div');
    div.className = `check-item ${r.pass ? 'pass' : 'fail'}`;
    const metrics = Object.entries(r.errors || {})
      .map(([k, v]) => `<div><code>${k}</code>: ${typeof v === 'number' ? v.toExponential(3) : v}</div>`)
      .join('');
    div.innerHTML = `
      <div class="head"><span class="verdict">${r.pass ? '✓ 通过' : '✗ 失败'}</span><span>${r.name}</span></div>
      <div class="metrics">${metrics}</div>
      ${r.note ? `<div class="note">${r.note}</div>` : ''}`;
    host.appendChild(div);
  }
  return allPass;
}

// ---- transport --------------------------------------------------------------

function tick(ts) {
  if (!state.lastTs) state.lastTs = ts;
  const dt = (ts - state.lastTs) / 1000;
  state.lastTs = ts;
  if (state.playing) {
    state.time += dt * state.config.playback.speed;
    const dur = state.retarget.duration;
    if (state.config.playback.loop) {
      state.time = ((state.time % dur) + dur) % dur;
    } else if (state.time >= dur) {
      state.time = dur;
      state.playing = false;
      $('btn-play').textContent = '播放';
    }
    $('scrub').value = Math.round((state.time / dur) * 1000);
    $('time-val').textContent = state.time.toFixed(2);
  }
  updatePose();
  state.srcViewer.render();
  state.tgtViewer.render();
  requestAnimationFrame(tick);
}

// ---- config / export --------------------------------------------------------

function saveConfig() {
  const json = serializeConfig(state.config);
  downloadBlob(new Blob([JSON.stringify(json, null, 2)], { type: 'application/json' }),
    'retarget-config.json');
}

async function loadConfig(file) {
  const text = await file.text();
  const cfg = parseConfig(JSON.parse(text));
  // Validate every referenced joint still exists in the loaded skeletons.
  for (const [t, s] of Object.entries(cfg.mapping)) {
    if (!state.tgt.byName.has(t)) throw new Error(`配置引用了未知目标关节 ${t}`);
    if (s && !state.src.byName.has(s)) throw new Error(`配置引用了未知源关节 ${s}`);
  }
  state.config = cfg;
  syncUIFromConfig();
  rebuild();
  buildRigs();
  runChecks();
}

function syncUIFromConfig() {
  const c = state.config;
  document.querySelector(`input[name="policy"][value="${c.policy}"]`).checked = true;
  $('ground-offset').value = c.rootGroundOffset;
  $('speed').value = c.playback.speed;
  $('speed-val').textContent = `${c.playback.speed.toFixed(1)}×`;
  $('loop').checked = c.playback.loop;
  $('ghost').checked = c.ghostSource;
  $('constraints-enabled').checked = !!c.constraints?.enabled;
  $('constraints-body').hidden = !c.constraints?.enabled;
}

async function doExportGLB() {
  const buffer = await exportGLB(state.tgt, state.retarget);
  downloadBlob(new Blob([buffer], { type: 'model/gltf-binary' }), 'retargeted-animation.glb');
}
function doExportJSON() {
  const json = exportJSON(state.tgt, state.retarget);
  downloadBlob(new Blob([JSON.stringify(json, null, 2)], { type: 'application/json' }),
    'retargeted-animation.json');
}

// ---- boot -------------------------------------------------------------------

function init() {
  state.src = loadSkeleton(sourceSkeleton);
  state.tgt = loadSkeleton(targetSkeleton);
  state.srcRest = canonicalRestPose(state.src);
  state.tgtRest = canonicalRestPose(state.tgt);
  state.srcClip = buildSourceClip(60);

  $('src-name').textContent = state.src.name;
  $('tgt-name').textContent = state.tgt.name;
  $('src-frame-badge').textContent = describeFrame(state.src.frame);
  $('tgt-frame-badge').textContent = describeFrame(state.tgt.frame);
  $('frame-summary').textContent =
    `源: ${describeFrame(state.src.frame)} ｜ 目标: ${describeFrame(state.tgt.frame)} ｜ 统一到 Y-up/-Z-forward/米制规范帧`;
  $('duration-val').textContent = state.srcClip.duration.toFixed(2);

  state.srcViewer = new SkeletonViewer($('src-view'));
  state.tgtViewer = new SkeletonViewer($('tgt-view'));

  rebuild();
  buildRigs();
  syncUIFromConfig();

  // Transport events.
  $('btn-play').addEventListener('click', () => {
    state.playing = !state.playing;
    $('btn-play').textContent = state.playing ? '暂停' : '播放';
    if (!state.playing && state.time >= state.retarget.duration) state.time = 0;
  });
  $('btn-reset').addEventListener('click', () => {
    state.time = 0; state.playing = false;
    $('btn-play').textContent = '播放'; $('scrub').value = 0;
  });
  $('scrub').addEventListener('input', (e) => {
    state.time = (e.target.value / 1000) * state.retarget.duration;
    $('time-val').textContent = state.time.toFixed(2);
  });
  $('speed').addEventListener('input', (e) => {
    state.config.playback.speed = Number(e.target.value);
    $('speed-val').textContent = `${state.config.playback.speed.toFixed(1)}×`;
  });
  $('loop').addEventListener('change', (e) => {
    state.config.playback.loop = e.target.checked;
  });
  $('ghost').addEventListener('change', (e) => {
    state.config.ghostSource = e.target.checked;
  });

  // Policy / ground offset.
  document.querySelectorAll('input[name="policy"]').forEach((r) => {
    r.addEventListener('change', () => {
      state.config.policy = r.value;
      rebuild();
    });
  });
  $('ground-offset').addEventListener('change', (e) => {
    state.config.rootGroundOffset = Number(e.target.value) || 0;
    rebuild();
  });

  // Mapping editor.
  $('btn-apply-mapping').addEventListener('click', () => {
    const t = $('edit-target').value;
    const s = $('edit-source').value;
    if (s) state.config.mapping[t] = s;
    else delete state.config.mapping[t];
    rebuild();
    buildRigs();
    selectTarget(t);
  });

  // Constraints panel.
  $('constraints-enabled').addEventListener('change', (e) => {
    state.config.constraints.enabled = e.target.checked;
    if (e.target.checked) ensureTargetConstraints();
    rebuild();
  });
  $('btn-constraints-defaults').addEventListener('click', () => {
    const defaults = defaultConstraintsFor(state.tgt.name);
    state.config.constraints.targets[state.tgt.name] = defaults || {
      limits: {}, contacts: [],
      ik: { maxIterations: 16, tolerance: 1e-3, minImprovement: 1e-6, maxFrameStepDeg: 12 },
    };
    rebuild();
  });
  $('btn-add-limit').addEventListener('click', () => {
    const settings = ensureTargetConstraints();
    const joint = $('limit-joint').value;
    const axis = $('limit-axis').value.split(',').map(Number);
    if (!joint) return;
    (settings.limits[joint] ??= []).push({ axis, min: -30, max: 30 });
    rebuild();
  });
  $('btn-add-contact').addEventListener('click', () => {
    const settings = ensureTargetConstraints();
    const effector = $('contact-effector').value;
    const mode = $('contact-mode').value;
    if (!effector) return;
    const id = `contact-${settings.contacts.length + 1}`;
    settings.contacts.push(mode === 'point'
      ? { id, effector, chain: [], priority: 20, mode, target: [0, 1, 0], enabled: true }
      : {
        id, effector, chain: [], priority: 10, mode,
        groundY: 0.15, enter: 0.03, exit: 0.07, pin: false, enabled: true,
      });
    rebuild();
  });
  for (const [id, key] of [['ik-maxiter', 'maxIterations'], ['ik-tol', 'tolerance'], ['ik-maxstep', 'maxFrameStepDeg']]) {
    $(id).addEventListener('change', (e) => {
      const settings = ensureTargetConstraints();
      settings.ik ??= {};
      settings.ik[key] = Number(e.target.value) || 0;
      rebuild();
    });
  }

  // Validation / config / export.
  $('btn-run-checks').addEventListener('click', runChecks);
  $('btn-save').addEventListener('click', saveConfig);
  $('file-load').addEventListener('change', (e) => {
    if (e.target.files[0]) {
      loadConfig(e.target.files[0]).catch((err) => alert(`载入失败: ${err.message}`));
      e.target.value = '';
    }
  });
  $('btn-export-glb').addEventListener('click', () =>
    doExportGLB().catch((err) => alert(`导出失败: ${err.message || err}`)));
  $('btn-export-json').addEventListener('click', doExportJSON);

  // Run the suite once on load so the metrics are immediately visible.
  runChecks();
  requestAnimationFrame(tick);
}

init();
