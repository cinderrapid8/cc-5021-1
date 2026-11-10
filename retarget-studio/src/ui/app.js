// Workbench application controller. Wires the engine to two SkeletonViewers,
// playback transport, mapping/rest-pose inspection, the validation suite,
// config save/load and GLB/JSON export.

import { sourceSkeleton, targetSkeleton, defaultMapping } from '../data/skeletons.js';
import { buildSourceClip } from '../data/clip.js';
import { loadSkeleton, canonicalRestPose, fk } from '../engine/skeleton.js';
import { retargetClip, sampleClip, analyzeMapping } from '../engine/retarget.js';
import { describeFrame, toCanonicalVec, toCanonicalQuat } from '../engine/frame.js';
import { runAllChecks } from '../engine/validation.js';
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

function rebuild() {
  const cfg = state.config;
  state.retarget = retargetClip(sourceSkeleton, targetSkeleton, state.srcClip,
    cfg.mapping, { policy: cfg.policy, rootGroundOffset: cfg.rootGroundOffset });
  state.info = analyzeMapping(sourceSkeleton, targetSkeleton, cfg.mapping);
  renderMappingTable();
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

// ---- validation -------------------------------------------------------------

function runChecks() {
  const results = runAllChecks(sourceSkeleton, targetSkeleton, state.srcClip,
    state.config.mapping, { policy: state.config.policy, rootGroundOffset: state.config.rootGroundOffset });
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
