// Workbench configuration serialization. A saved config fully determines the
// retarget result, so reloading it must reproduce identical keyframes. The
// validation suite checks this numerically; here we only package/parse.

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
  };
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
  };
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
  };
}
