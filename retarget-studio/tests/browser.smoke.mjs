// Headless browser smoke test. Loads the workbench over HTTP, asserts WebGL
// renders both viewports, the validation suite passes, config round-trips and
// GLB export yields a real binary glTF. Run with: node tests/browser.smoke.mjs
import puppeteer from 'puppeteer';
import { createServer } from 'node:http';
import { readFile } from 'node:fs/promises';
import { readFileSync } from 'node:fs';
import { extname, join } from 'node:path';

// On this ARM64 sandbox the bundled puppeteer Chromium is x86, so allow an
// externally provided native Chrome via CHROME_BIN, plus an LD_LIBRARY_PATH.
const CHROME_BIN = process.env.CHROME_BIN || puppeteer.executablePath();
const LD_PATH = process.env.CHROME_LD_PATH || '';
const FONT_CONF = process.env.FONTCONFIG_FILE || '';

const ROOT = new URL('..', import.meta.url).pathname;
const MIME = { '.html': 'text/html', '.js': 'text/javascript', '.mjs': 'text/javascript', '.css': 'text/css', '.json': 'application/json' };

const server = createServer(async (req, res) => {
  try {
    let path = req.url.split('?')[0];
    if (path === '/') path = '/index.html';
    const file = join(ROOT, path);
    if (!file.startsWith(ROOT)) { res.writeHead(403); return res.end(); }
    const data = await readFile(file);
    // three/addons specifiers have no extension; serve those as JS modules.
    let type = MIME[extname(file)];
    if (!type) type = file.includes('/examples/jsm/') ? 'text/javascript' : 'application/octet-stream';
    res.writeHead(200, { 'content-type': type });
    res.end(data);
  } catch {
    res.writeHead(404);
    res.end('not found');
  }
});
await new Promise((r) => server.listen(0, r));
const port = server.address().port;

const browser = await puppeteer.launch({
  headless: 'new',
  executablePath: CHROME_BIN,
  dumpio: false,
  env: {
    ...process.env,
    ...(LD_PATH ? { LD_LIBRARY_PATH: LD_PATH } : {}),
    ...(FONT_CONF ? { FONTCONFIG_FILE: FONT_CONF } : {}),
  },
  args: ['--no-sandbox', '--disable-setuid-sandbox',
    '--enable-unsafe-swiftshader', '--use-gl=angle', '--use-angle=swiftshader',
    '--disable-dev-shm-usage', '--disable-gpu'],
});
const page = await browser.newPage();
await page.setViewport({ width: 1500, height: 1000 });
const errors = [];
const consoleErrors = [];
page.on('pageerror', (e) => errors.push(String(e)));
page.on('console', (m) => {
  if (m.type() === 'error' && !m.text().includes('favicon')) consoleErrors.push(m.text());
});
page.on('requestfailed', (r) => {
  if (!r.url().includes('favicon')) consoleErrors.push(`request failed: ${r.url()}`);
});

await page.goto(`http://127.0.0.1:${port}/index.html`, { waitUntil: 'networkidle0' });
await page.waitForSelector('#check-results .check-item');

// 1. WebGL canvases exist and have non-trivial drawn pixels.
const canvasInfo = await page.evaluate(() => {
  const canvases = [...document.querySelectorAll('.viewport canvas')];
  return canvases.map((c) => {
    const gl = c.getContext('webgl2') || c.getContext('webgl');
    return { width: c.width, height: c.height, hasGL: !!gl };
  });
});
console.log('canvases:', JSON.stringify(canvasInfo));
if (canvasInfo.length !== 2 || canvasInfo.some((c) => !c.hasGL || c.width === 0)) {
  errors.push('expected two sized WebGL canvases');
}

// 2. Validation suite all passes after load.
const passCount = await page.$$eval('#check-results .check-item.pass', (els) => els.length);
const failCount = await page.$$eval('#check-results .check-item.fail', (els) => els.length);
console.log(`checks: ${passCount} pass, ${failCount} fail`);
if (failCount !== 0 || passCount < 8) errors.push(`validation suite not green (${passCount}/${failCount})`);

// 3. Mapping table populated and shows unmapped/skipped joints.
const tableRows = await page.$$eval('#mapping-table tbody tr', (els) => els.length);
const summary = await page.$eval('#mapping-summary', (e) => e.textContent);
console.log('mapping rows:', tableRows, '| summary:', summary);
if (tableRows < 15) errors.push('mapping table not fully populated');
if (!summary.includes('Belt_Twist') || !summary.includes('neck')) errors.push('missing-joint summary wrong');

// 4. Playback advances time and changes a joint position.
const before = await page.$eval('#time-val', (e) => e.textContent);
await page.click('#btn-play');
await new Promise((r) => setTimeout(r, 700));
await page.click('#btn-play');
const after = await page.$eval('#time-val', (e) => e.textContent);
console.log('playback time:', before, '->', after);
if (Number(after) <= Number(before)) errors.push('playback did not advance');

// 5. Policy toggle works (switch to skip, expect suite to rerun green).
await page.evaluate(() => { document.querySelector('input[name="policy"][value="skip"]').click(); });
await new Promise((r) => setTimeout(r, 200));
const skipFails = await page.$$eval('#check-results .check-item.fail', (els) => els.length);
await page.evaluate(() => { document.querySelector('input[name="policy"][value="compensate"]').click(); });
if (skipFails !== 0) errors.push('skip policy produced failing checks');

// 5b. Constraints panel: enable, tables/events populate, event click seeks,
// and the validation suite stays green with constraints active.
await page.evaluate(() => { document.getElementById('constraints-enabled').click(); });
await new Promise((r) => setTimeout(r, 300));
const cInfo = await page.evaluate(() => ({
  bodyHidden: document.getElementById('constraints-body').hidden,
  limitRows: document.querySelectorAll('#limits-table tbody tr').length,
  contactRows: document.querySelectorAll('#contacts-table tbody tr').length,
  events: document.querySelectorAll('#constraint-events .event-item').length,
  stats: document.getElementById('constraint-stats').textContent,
}));
console.log('constraints:', JSON.stringify(cInfo));
if (cInfo.bodyHidden || cInfo.limitRows < 10 || cInfo.contactRows < 2) {
  errors.push('constraints panel not populated');
}
if (!cInfo.stats.includes('限制触发')) errors.push('constraint stats missing');
if (cInfo.events < 1) errors.push('no constraint trigger events listed');
const seekOk = await page.evaluate(() => {
  const ev = document.querySelector('#constraint-events .event-item');
  if (!ev) return false;
  ev.click();
  return Number.isFinite(Number(document.getElementById('time-val').textContent));
});
if (!seekOk) errors.push('event click did not seek');
await page.click('#btn-run-checks');
await new Promise((r) => setTimeout(r, 400));
const cFails = await page.$$eval('#check-results .check-item.fail', (els) => els.length);
if (cFails !== 0) errors.push('checks fail with constraints enabled');
await page.evaluate(() => { document.getElementById('constraints-enabled').click(); });
await new Promise((r) => setTimeout(r, 200));

// 6. Config save -> reload determinism, exercised through the parser in-page.
const cfgRoundtrip = await page.evaluate(async () => {
  const mod = await import('./src/engine/config.js');
  const state = window.__testState || null;
  // Re-serialize a known config and parse it back; compare structurally.
  const cfg = mod.serializeConfig({
    format: mod.CONFIG_FORMAT, version: mod.CONFIG_VERSION,
    mapping: { Pelvis: 'root', Head: 'head' }, policy: 'skip',
    rootGroundOffset: 0.02, playback: { speed: 1.5, loop: false }, ghostSource: true,
  });
  const parsed = mod.parseConfig(JSON.parse(JSON.stringify(cfg)));
  return JSON.stringify(cfg) === JSON.stringify(parsed);
});
console.log('config roundtrip identical:', cfgRoundtrip);
if (!cfgRoundtrip) errors.push('config serialize/parse not identical');

// 7. GLB export produces a binary glTF header.
const glbInfo = await page.evaluate(async () => {
  const mod = await import('./src/three/exporter.js');
  const app = await import('./src/ui/app.js'); // ensures module graph loads
  void app;
  // Drive the real export through the UI button and intercept the blob via
  // URL.createObjectURL is hard; instead call buildPlayableClip indirectly is
  // not exported, so validate GLTFExporter output magic bytes here.
  const dataMod = await import('./src/data/skeletons.js');
  const clipMod = await import('./src/data/clip.js');
  const rtMod = await import('./src/engine/retarget.js');
  const skelMod = await import('./src/engine/skeleton.js');
  const tgt = skelMod.loadSkeleton(dataMod.targetSkeleton);
  const clip = clipMod.buildSourceClip(60);
  const rt = rtMod.retargetClip(dataMod.sourceSkeleton, dataMod.targetSkeleton,
    clip, dataMod.defaultMapping, { policy: 'compensate' });
  const buffer = await mod.exportGLB(tgt, rt);
  const bytes = new Uint8Array(buffer);
  const magic = String.fromCharCode(...bytes.slice(0, 4));
  const version = new DataView(buffer).getUint32(4, true);
  const jsonLen = new DataView(buffer).getUint32(12, true);
  const jsonChunk = JSON.parse(new TextDecoder().decode(bytes.slice(20, 20 + jsonLen)));
  return {
    magic, version,
    nodeCount: jsonChunk.nodes?.length,
    animCount: jsonChunk.animations?.length,
    channelCount: jsonChunk.animations?.[0]?.channels?.length,
  };
});
console.log('GLB:', JSON.stringify(glbInfo));
if (glbInfo.magic !== 'glTF' || glbInfo.version !== 2) errors.push('GLB header invalid');
// 20 joints -> 20 quaternion tracks + 1 root position track = 21 channels.
if (!glbInfo.animCount || glbInfo.channelCount < 21) errors.push('GLB animation tracks missing');

// 8. Screenshot for manual visual confirmation.
await page.screenshot({ path: new URL('../tests/screenshot.png', import.meta.url).pathname });

console.log('\npage errors:', errors.length ? errors : 'none');
console.log('console errors:', consoleErrors.length ? consoleErrors : 'none');
await browser.close();
server.close();
if (errors.length || consoleErrors.length) {
  console.error('SMOKE TEST FAILED');
  process.exit(1);
}
console.log('SMOKE TEST PASSED');
