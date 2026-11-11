// Workbench configuration serialization. A saved config fully determines the
// retarget result, so reloading it must reproduce identical keyframes. The
// validation suite checks this numerically; here we only package/parse.
//
// Constraint settings (ROM limits + contacts) are stored per TARGET skeleton
// name: they describe the target character's anatomy and follow it across
// configs. The source animation is never part of the config and never
// modified. Configs written before constraints existed parse with
// constraints disabled, which reproduces the initial unconstrained behavior.

export const CONFIG_FORMAT = 'retarget-studio/config';
export const CONFIG_VERSION = 1;

export function defaultConfig() {
  return {
    format: CONFIG_FORMAT,
    version: CONFIG_VERSION,
    mapping: {}, // filled from the data default on first load
    policy: 'compensate', // 'compensate' | 'skip'
    rootGroundOffset: 0,  // meters added along canonical +Y
    playback: { speed: 1, loop: true },
    ghostSource: false,
    // enabled:false + no per-target settings == the initial unconstrained
    // retargeting. The UI fills defaults for the current target on boot.
    constraints: { enabled: false, targets: {} },
  };
}

function serializeConstraintTarget(t) {
  const limits = {};
  for (const joint of Object.keys(t.limits || {}).sort((a, b) => a.localeCompare(b))) {
    limits[joint] = (t.limits[joint] || []).map((a) => ({
      axis: a.axis.map(Number),
      min: Number(a.min),
      max: Number(a.max),
    }));
  }
  return {
    limits,
    contacts: (t.contacts || []).map((c) => {
      const out = {
        id: String(c.id),
        effector: String(c.effector),
        chain: (c.chain || []).map(String),
        priority: Number(c.priority) || 0,
        mode: c.mode === 'point' ? 'point' : 'ground',
        enabled: c.enabled !== false,
      };
      if (out.mode === 'point') {
        out.target = (c.target || [0, 0, 0]).map(Number);
      } else {
        out.groundY = Number(c.groundY) || 0;
        out.enter = Number(c.enter) || 0;
        out.exit = Number(c.exit) || 0;
        out.pin = c.pin !== false;
      }
      return out;
    }),
    ik: {
      maxIterations: Number(t.ik?.maxIterations) || 16,
      tolerance: Number(t.ik?.tolerance) || 1e-3,
      minImprovement: Number(t.ik?.minImprovement) || 1e-6,
      maxFrameStepDeg: Number(t.ik?.maxFrameStepDeg) || 0,
    },
  };
}

function serializeConstraints(c) {
  const targets = {};
  for (const name of Object.keys(c?.targets || {}).sort((a, b) => a.localeCompare(b))) {
    targets[name] = serializeConstraintTarget(c.targets[name]);
  }
  return { enabled: !!c?.enabled, targets };
}

export function serializeConfig(config) {
  // Strip anything non-deterministic and write stable key order.
  return {
    format: CONFIG_FORMAT,
    version: CONFIG_VERSION,
    mapping: Object.fromEntries(Object.entries(config.mapping).sort((a, b) =>
      a[0].localeCompare(b[0]))),
    policy: config.policy,
    rootGroundOffset: Number(config.rootGroundOffset) || 0,
    playback: {
      speed: Number(config.playback?.speed) || 1,
      loop: config.playback?.loop !== false,
    },
    ghostSource: !!config.ghostSource,
    constraints: serializeConstraints(config.constraints),
  };
}

function parseConstraintTarget(raw, label) {
  if (!raw || typeof raw !== 'object') throw new Error(`约束设置 ${label} 不是对象`);
  const limits = {};
  for (const [joint, axes] of Object.entries(raw.limits || {})) {
    if (!Array.isArray(axes)) throw new Error(`约束 ${label}/${joint} 的轴列表无效`);
    limits[joint] = axes.map((a, i) => {
      if (!Array.isArray(a?.axis) || a.axis.length !== 3) {
        throw new Error(`约束 ${label}/${joint}[${i}] 缺少三维轴`);
      }
      if (!Number.isFinite(Number(a.min)) || !Number.isFinite(Number(a.max))) {
        throw new Error(`约束 ${label}/${joint}[${i}] 缺少角度范围`);
      }
      return { axis: a.axis.map(Number), min: Number(a.min), max: Number(a.max) };
    });
  }
  const contacts = (raw.contacts || []).map((c, i) => {
    if (!c || typeof c.effector !== 'string') {
      throw new Error(`接触点 ${label}[${i}] 缺少 effector`);
    }
    if (!['ground', 'point'].includes(c.mode)) {
      throw new Error(`接触点 ${label}[${i}] 的 mode 必须是 ground 或 point`);
    }
    // Keep exactly the mode-relevant keys so parse(serialize(x)) is a fixpoint.
    const out = {
      id: String(c.id ?? `contact-${i}`),
      effector: c.effector,
      chain: Array.isArray(c.chain) ? c.chain.map(String) : [],
      priority: Number(c.priority) || 0,
      mode: c.mode,
      enabled: c.enabled !== false,
    };
    if (out.mode === 'point') {
      if (!Array.isArray(c.target) || c.target.length !== 3) {
        throw new Error(`接触点 ${label}[${i}] 缺少三维目标点`);
      }
      out.target = c.target.map(Number);
    } else {
      out.groundY = Number(c.groundY) || 0;
      out.enter = Number(c.enter) || 0;
      out.exit = Number(c.exit) || 0;
      out.pin = c.pin !== false;
    }
    return out;
  });
  return {
    limits,
    contacts,
    ik: {
      maxIterations: Number(raw.ik?.maxIterations) || 16,
      tolerance: Number(raw.ik?.tolerance) || 1e-3,
      minImprovement: Number(raw.ik?.minImprovement) || 1e-6,
      maxFrameStepDeg: Number(raw.ik?.maxFrameStepDeg) || 0,
    },
  };
}

function parseConstraints(raw) {
  if (raw == null) return { enabled: false, targets: {} }; // pre-constraint config
  if (typeof raw !== 'object') throw new Error('constraints 段无效');
  const targets = {};
  for (const [name, t] of Object.entries(raw.targets || {})) {
    targets[name] = parseConstraintTarget(t, name);
  }
  return { enabled: !!raw.enabled, targets };
}

export function parseConfig(json) {
  if (!json || json.format !== CONFIG_FORMAT) {
    throw new Error('不是有效的重定向工作台配置文件');
  }
  if (json.version !== CONFIG_VERSION) {
    throw new Error(`不支持的配置版本 ${json.version}`);
  }
  if (!json.mapping || typeof json.mapping !== 'object') {
    throw new Error('配置缺少 mapping 对象');
  }
  if (!['compensate', 'skip'].includes(json.policy)) {
    throw new Error(`未知重定向策略 ${json.policy}`);
  }
  return {
    format: CONFIG_FORMAT,
    version: CONFIG_VERSION,
    mapping: json.mapping,
    policy: json.policy,
    rootGroundOffset: Number(json.rootGroundOffset) || 0,
    playback: {
      speed: Number(json.playback?.speed) || 1,
      loop: json.playback?.loop !== false,
    },
    ghostSource: !!json.ghostSource,
    constraints: parseConstraints(json.constraints),
  };
}
