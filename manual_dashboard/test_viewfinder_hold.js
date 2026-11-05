#!/usr/bin/env node
'use strict';

// Host-only tests of the actual inline dashboard script, with no browser,
// packages, HTTP server, network requests, or motor controller.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const html = fs.readFileSync(path.join(__dirname, 'viewfinder.html'), 'utf8');
const scripts = [...html.matchAll(/<script\b([^>]*)>([\s\S]*?)<\/script>/gi)]
  .filter(match => !/\bsrc\s*=/.test(match[1]));
assert.equal(scripts.length, 1, 'Expect exactly one inline application script');
const startup = /poll\(0\);poll\(1\);encoders\(\);loadMotion\(\);\s*$/;
assert.match(scripts[0][2], startup, 'Only the final background startup is removed');
const source = scripts[0][2].replace(startup, '');
const buttons = ['jogAzMinus', 'jogAzPlus', 'jogElMinus', 'jogElPlus'];
const flush = async () => { for (let i = 0; i < 16; i++) await Promise.resolve(); };
const deferred = () => {
  let resolve, reject;
  const promise = new Promise((yes, no) => { resolve = yes; reject = no; });
  return {promise, resolve, reject};
};
const response = (body = {ok: true}, ok = true) => ({ok, json: async () => body});

function harness(savedSettings = {}) {
  const elements = new Map(), calls = [], replies = new Map(), timers = new Map();
  const storage = new Map(Object.entries(savedSettings));
  let now = 0, nextTimer = 1, nextUUID = 1;
  class Target {
    constructor() { this.listeners = new Map(); }
    addEventListener(type, fn, options) {
      if (!this.listeners.has(type)) this.listeners.set(type, []);
      this.listeners.get(type).push({fn, once: Boolean(options?.once)});
    }
    removeEventListener(type, fn) {
      this.listeners.set(type, (this.listeners.get(type) || []).filter(item => item.fn !== fn));
    }
    dispatch(type, fields = {}) {
      const event = {type, target: this, currentTarget: this, button: 0, buttons: 1,
        pointerId: 7, isPrimary: true, defaultPrevented: false,
        preventDefault() { this.defaultPrevented = true; }, stopPropagation() {}, ...fields};
      if (typeof this['on' + type] === 'function') this['on' + type](event);
      for (const entry of [...(this.listeners.get(type) || [])]) {
        entry.fn(event);
        if (entry.once) this.removeEventListener(type, entry.fn);
      }
      return event;
    }
  }
  const context2d = new Proxy({}, {get: (target, key) => target[key] || (() => {})});
  class Element extends Target {
    constructor(tag = 'div') {
      super(); this.tagName = tag.toUpperCase(); this.style = {}; this.dataset = {};
      this.value = ''; this.disabled = false; this.textContent = ''; this.children = [];
      this.captures = new Set(); this.attributes = {}; this.className = '';
      const classes = new Set();
      this.classList = {add: value => classes.add(value), remove: value => classes.delete(value),
        contains: value => classes.has(value), toggle: (value, force) => {
          const add = force === undefined ? !classes.has(value) : force;
          if (add) classes.add(value); else classes.delete(value); return add;
        }};
    }
    set innerHTML(value) { this._html = value; register(value); }
    get innerHTML() { return this._html || ''; }
    appendChild(child) { this.children.push(child); return child; }
    getContext() { return context2d; }
    getBoundingClientRect() { return {left: 0, top: 0, width: 1280, height: 720}; }
    setPointerCapture(id) { this.captures.add(id); }
    hasPointerCapture(id) { return this.captures.has(id); }
    releasePointerCapture(id) {
      if (this.captures.delete(id)) this.dispatch('lostpointercapture', {pointerId: id});
    }
    setAttribute(name, value) { this.attributes[name] = String(value); }
    getAttribute(name) { return this.attributes[name] ?? null; }
    requestFullscreen() {}
    focus() { document.activeElement = this; }
  }
  function register(markup) {
    for (const match of markup.matchAll(/<(\w+)\b([^>]*\bid=["']([^"']+)["'][^>]*)>/g)) {
      const [, tag, attributes, id] = match;
      const element = new Element(tag); element.id = id;
      for (const attr of attributes.matchAll(/([\w-]+)=["']([^"']*)["']/g)) {
        element.attributes[attr[1]] = attr[2]; element[attr[1]] = attr[2];
        if (attr[1].startsWith('data-')) element.dataset[attr[1].slice(5)] = attr[2];
      }
      element.disabled = /\bdisabled\b/.test(attributes);
      if (tag === 'select') {
        const rest = markup.slice(match.index + match[0].length).split('</select>')[0];
        const options = [...rest.matchAll(/<option\b([^>]*)value=["']([^"']+)["']([^>]*)>/g)];
        const selected = options.find(option => /\bselected\b/.test(option[1] + option[3])) || options[0];
        element.value = selected?.[2] || '';
      }
      elements.set(id, element);
    }
  }
  const document = new Target();
  Object.assign(document, {hidden: false, visibilityState: 'visible', activeElement: {tagName: 'BODY'},
    getElementById: id => elements.get(id) || null, createElement: tag => new Element(tag),
    querySelectorAll: selector => selector === '.marker' ? elements.get('markers').children : [],
    exitFullscreen() {}});
  const window = new Target();
  window.localStorage = {getItem: key => storage.get(key) ?? null,
    setItem: (key, value) => storage.set(key, String(value))};
  function timer(fn, delay, interval = false) {
    const id = nextTimer++;
    timers.set(id, {fn, at: now + Number(delay || 0), delay: Number(delay || 0), interval});
    return id;
  }
  async function fetch(url, options = {}) {
    const call = {url, options, body: options.body ? JSON.parse(options.body) : null, at: now};
    calls.push(call);
    const pending = replies.get(url);
    if (pending?.length) {
      const result = pending.shift();
      return typeof result === 'function' ? result(call) : result;
    }
    return response();
  }
  const crypto = {randomUUID: () => `00000000-0000-4000-8000-${String(nextUUID++).padStart(12, '0')}`};
  Object.assign(window, {document, crypto});
  register(html);
  const context = vm.createContext({document, window, console, fetch, crypto,
    setTimeout: (fn, delay) => timer(fn, delay), clearTimeout: id => timers.delete(id),
    setInterval: (fn, delay) => timer(fn, delay, true), clearInterval: id => timers.delete(id),
    Date: class extends Date { static now() { return now; } }, performance: {now: () => now},
    nativeToScreen: point => point, screenToNative: point => point,
    mapPoint: point => point, createImageBitmap: async () => { throw Error('Unexpected image request'); }});
  vm.runInContext(source, context, {filename: 'viewfinder.html:inline'});
  vm.runInContext("motionToken='test-token'; motionStatus={connected:true,last_error:null,busy:false}; updateMotionButtons();", context);
  return {
    elements, calls, document, window, timers, storage,
    el(id) { assert.ok(elements.has(id), `Missing control ${id}`); return elements.get(id); },
    eval(code) { return vm.runInContext(code, context); },
    queue(url, result) { if (!replies.has(url)) replies.set(url, []); replies.get(url).push(result); },
    requests(url) { return calls.filter(call => call.url === url); },
    async emit(target, event, fields) { target.dispatch(event, fields); await flush(); },
    async mode(value) { const el = this.el('jogMode'); el.value = value; await this.emit(el, 'change'); },
    async down(id = 'jogAzPlus', fields) { await this.emit(this.el(id), 'pointerdown', fields); },
    async advance(ms) {
      const end = now + ms; let iterations = 0;
      for (;;) {
        const next = [...timers].filter(([, task]) => task.at <= end).sort((a, b) => a[1].at - b[1].at)[0];
        if (!next) break;
        assert.ok(++iterations < 10000, 'Runaway timer');
        const [id, task] = next; now = task.at;
        if (task.interval) task.at += task.delay; else timers.delete(id);
        task.fn(); await flush();
      }
      now = end; await flush();
    },
  };
}

const tests = [];
function test(name, fn) { tests.push({name, fn}); }

test('manual step and hold work without zeroing or any reference calibration', async () => {
  for (const mode of ['step', 'continuous']) {
    const h = harness(); await h.mode(mode);
    assert.equal(h.eval('frozen===null&&views.every(v=>Object.keys(v.points).length===0)'), true);
    assert.match(h.el('motionHelp').textContent, /No zeroing or calibration required for manual jogs\./);
    assert.equal(h.el('jogAzPlus').disabled, false);
    if (mode === 'step') await h.emit(h.el('jogAzPlus'), 'click');
    else await h.down();
    assert.equal(h.requests(mode === 'step' ? '/motion/jog' : '/motion/hold/start').length, 1);
    assert.equal(h.calls.some(call => /zero|home|calibrat/.test(call.url)), false);
  }
});

test('continuous pointerdown sends one authenticated hold start, never a step', async () => {
  const h = harness();
  await h.mode('continuous');
  assert.equal(h.el('jogSpeed').value, '0.5');
  assert.equal(Number(h.el('jogSpeed').max), 15);
  await h.down();
  assert.equal(h.requests('/motion/hold/start').length, 1);
  const start = h.requests('/motion/hold/start')[0];
  assert.match(start.body.hold_id, /^[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}$/i);
  assert.deepEqual(start.body, {hold_id: start.body.hold_id, axis: 'az', velocity: 0.5});
  assert.equal(start.options.headers['X-DET-Token'], 'test-token');
  await h.emit(h.el('jogAzPlus'), 'click');
  assert.equal(h.requests('/motion/jog').length, 0);
  assert.equal(h.requests('/motion/hold/start').length, 1);
});

test('all four hold buttons use the selected speed and correct axis/sign', async () => {
  for (const [id, axis, sign] of [
    ['jogAzMinus', 'az', -1], ['jogAzPlus', 'az', 1],
    ['jogElMinus', 'el', -1], ['jogElPlus', 'el', 1],
  ]) {
    const h = harness(); await h.mode('continuous'); h.el('jogSpeed').value = '2.5';
    await h.down(id);
    const starts = h.requests('/motion/hold/start');
    assert.equal(starts.length, 1, id);
    assert.equal(starts[0].body.axis, axis, id);
    assert.equal(starts[0].body.velocity, sign * 2.5, id);
    assert.equal(h.el(id).hasPointerCapture(7), true, 'Capture the held pointer');
  }
});

test('step mode preserves discrete jogs and never starts a hold on pointerdown', async () => {
  const h = harness(); await h.mode('step'); h.el('jogSize').value = '3';
  await h.down('jogElMinus');
  assert.equal(h.requests('/motion/hold/start').length, 0);
  await h.emit(h.el('jogElMinus'), 'pointerup');
  await h.emit(h.el('jogElMinus'), 'click');
  assert.deepEqual(h.requests('/motion/jog').map(call => call.body), [{axis: 'el', delta: -3}]);
  assert.equal(h.requests('/motion/hold/start').length, 0);
});

test('continuous mode rejects speeds outside the positive finite 15 degree/s limit', async () => {
  for (const value of ['0', '-0.5', '15.01', 'NaN', 'Infinity', '']) {
    const h = harness(); await h.mode('continuous'); h.el('jogSpeed').value = value;
    await h.down();
    assert.equal(h.requests('/motion/hold/start').length, 0, `Invalid speed ${value}`);
    assert.equal(h.requests('/motion/jog').length, 0);
  }
  const h = harness(); await h.mode('continuous'); h.el('jogSpeed').value = '15';
  await h.down();
  assert.equal(h.requests('/motion/hold/start')[0]?.body.velocity, 15, 'Maximum allowed speed');
});

test('held button stays enabled through backend busy, other directions and Freeze are disabled', async () => {
  const h = harness(); await h.mode('continuous'); await h.down();
  h.eval('motionStatus.busy=true; updateMotionButtons();');
  assert.equal(h.el('jogAzPlus').disabled, false, 'Disabling the held button can lose release events');
  for (const id of buttons.filter(id => id !== 'jogAzPlus')) assert.equal(h.el(id).disabled, true, id);
  assert.equal(h.el('freeze').disabled, true);
  assert.equal(h.el('motionStop').disabled, false);
  await h.emit(h.el('jogAzPlus'), 'pointerup');
  assert.equal(h.requests('/motion/stop').length, 1);
});

test('a second pointer or direction cannot start overlapping holds', async () => {
  const h = harness(); await h.mode('continuous'); await h.down();
  await h.down('jogElPlus', {pointerId: 8});
  await h.down('jogAzPlus', {pointerId: 9});
  assert.equal(h.requests('/motion/hold/start').length, 1);
});

const releaseCases = [
  ['pointer release', async h => h.emit(h.el('jogAzPlus'), 'pointerup')],
  ['pointer cancellation', async h => h.emit(h.el('jogAzPlus'), 'pointercancel')],
  ['lost capture', async h => h.emit(h.el('jogAzPlus'), 'lostpointercapture')],
  ['window pointerup fallback', async h => h.emit(h.window, 'pointerup')],
  ['window pointercancel fallback', async h => h.emit(h.window, 'pointercancel')],
  ['window blur', async h => h.emit(h.window, 'blur')],
  ['hidden document', async h => {
    h.document.hidden = true; h.document.visibilityState = 'hidden';
    await h.emit(h.document, 'visibilitychange');
  }],
  ['pagehide', async h => h.emit(h.window, 'pagehide')],
];
for (const [reason, release] of releaseCases) {
  test(`${reason} sends scoped Stop and cancels all future renewal`, async () => {
    const h = harness(); await h.mode('continuous'); await h.down();
    const holdID = h.requests('/motion/hold/start')[0].body.hold_id;
    await release(h);
    const stops = h.requests('/motion/stop');
    assert.equal(stops.length, 1, 'Release and lost capture must not duplicate Stop');
    assert.deepEqual(stops[0].body, {hold_id: holdID});
    assert.equal(stops[0].options.keepalive, true, 'Release must survive page teardown');
    await h.advance(2000);
    assert.equal(h.requests('/motion/hold/renew').length, 0);
    await h.emit(h.el('jogAzPlus'), 'click');
    assert.equal(h.requests('/motion/jog').length, 0, 'Synthetic post-release click must not jog');
  });
}

test('visible visibilitychange does not stop an active hold', async () => {
  const h = harness(); await h.mode('continuous'); await h.down();
  await h.emit(h.document, 'visibilitychange');
  assert.equal(h.requests('/motion/stop').length, 0);
  await h.advance(200);
  assert.equal(h.requests('/motion/hold/renew').length, 1);
});

test('Stop button globally stops an active hold, then idle Stop sends the empty body', async () => {
  const h = harness(); await h.mode('continuous'); await h.down();
  const holdID = h.requests('/motion/hold/start')[0].body.hold_id;
  await h.emit(h.el('motionStop'), 'click');
  assert.deepEqual(h.requests('/motion/stop').map(call => call.body), [{hold_id: holdID, global: true}]);
  await h.advance(1000);
  assert.equal(h.requests('/motion/hold/renew').length, 0);
  await h.emit(h.el('motionStop'), 'click');
  assert.deepEqual(h.requests('/motion/stop')[1].body, {});
});

test('release sends Stop before a pending start reply and the late reply cannot renew', async () => {
  const h = harness(), startReply = deferred();
  h.queue('/motion/hold/start', startReply.promise);
  await h.mode('continuous'); await h.down();
  const holdID = h.requests('/motion/hold/start')[0].body.hold_id;
  await h.emit(h.el('jogAzPlus'), 'pointerup');
  assert.deepEqual(h.requests('/motion/stop').map(call => call.body), [{hold_id: holdID}]);
  await h.advance(1000);
  assert.equal(h.requests('/motion/hold/renew').length, 0);
  startReply.resolve(response({accepted: true})); await flush();
  await h.advance(2000);
  assert.equal(h.requests('/motion/hold/renew').length, 0);
  assert.equal(h.requests('/motion/hold/start').length, 1);
});

test('renewal starts at 200 ms and pending requests never overlap', async () => {
  const h = harness(), renewReply = deferred();
  h.queue('/motion/hold/renew', renewReply.promise);
  await h.mode('continuous'); await h.down();
  const holdID = h.requests('/motion/hold/start')[0].body.hold_id;
  await h.advance(199);
  assert.equal(h.requests('/motion/hold/renew').length, 0);
  await h.advance(1);
  assert.equal(h.requests('/motion/hold/renew').length, 1);
  assert.deepEqual(h.requests('/motion/hold/renew')[0].body, {hold_id: holdID});
  await h.advance(1000);
  assert.equal(h.requests('/motion/hold/renew').length, 1, 'Do not overlap an outstanding renewal');
  renewReply.resolve(response()); await flush(); await h.advance(200);
  assert.equal(h.requests('/motion/hold/renew').length, 2);
  assert.equal(h.requests('/motion/stop').length, 0);
});

test('release during pending renewal prevents a late response from scheduling another', async () => {
  const h = harness(), renewReply = deferred();
  h.queue('/motion/hold/renew', renewReply.promise);
  await h.mode('continuous'); await h.down(); await h.advance(200);
  assert.equal(h.requests('/motion/hold/renew').length, 1);
  await h.emit(h.el('jogAzPlus'), 'pointerup');
  assert.equal(h.requests('/motion/stop').length, 1);
  renewReply.resolve(response()); await flush(); await h.advance(2000);
  assert.equal(h.requests('/motion/hold/renew').length, 1);
});

for (const kind of ['network rejection', 'HTTP failure']) {
  test(`renew ${kind} cancels renewal and attempts scoped Stop`, async () => {
    const h = harness(), renewReply = deferred();
    h.queue('/motion/hold/renew', renewReply.promise);
    await h.mode('continuous'); await h.down(); await h.advance(200);
    const holdID = h.requests('/motion/hold/start')[0].body.hold_id;
    if (kind === 'network rejection') renewReply.reject(Error('Link unavailable'));
    else renewReply.resolve(response({error: 'Lease rejected'}, false));
    await flush();
    assert.deepEqual(h.requests('/motion/stop').map(call => call.body), [{hold_id: holdID}]);
    await h.advance(2000);
    assert.equal(h.requests('/motion/hold/renew').length, 1);
  });
}

test('a failed Stop reply never restarts renewal', async () => {
  const h = harness();
  h.queue('/motion/stop', response({error: 'Stop unavailable'}, false));
  await h.mode('continuous'); await h.down();
  await h.emit(h.el('jogAzPlus'), 'pointerup');
  await h.advance(2000);
  assert.equal(h.requests('/motion/stop').length, 1);
  assert.equal(h.requests('/motion/hold/renew').length, 0);
  assert.match(h.el('motionStatus').textContent, /stop|confirm|unavailable/i);
});

const limitsStatus = (overrides = {}) => ({connected:true, busy:false, hold_active:false,
  recovery_required:false, position:{az:12.25,el:23.5}, position_age_seconds:0.01,
  limits:{soft_limits_deg:{az:[-180,180],el:[-10,95]},acknowledged:false,
    confirmation_required:false,source:'firmware_defaults_assumed'}, ...overrides});

test('limits prefill once, preserve edited drafts, and never apply on load', async () => {
  const h = harness();
  h.queue('/motion', response(limitsStatus())); await h.eval('pollMotion()');
  assert.equal(Number(h.el('az_min').value), -180);
  h.el('az_min').value = '-42'; await h.emit(h.el('az_min'), 'input');
  const status = limitsStatus(); status.limits.soft_limits_deg.az = [-90,90];
  h.queue('/motion', response(status)); await h.eval('pollMotion()');
  assert.equal(h.el('az_min').value, '-42');
  assert.match(h.el('activeLimits').textContent, /-90/);
  assert.equal(h.calls.filter(call => call.options.method === 'POST').length, 0);
});

test('copy endpoint gets fresh idle position without moving, including frozen views', async () => {
  const h = harness(); h.eval('frozen={id:"frozen"}; updateMotionButtons()');
  assert.equal(h.el('use_az_min').disabled, false);
  h.queue('/motion', response(limitsStatus()));
  await h.emit(h.el('use_az_min'), 'click');
  assert.equal(Number(h.el('az_min').value), 12.25);
  assert.equal(h.requests('/motion')[0].options.cache, 'no-store');
  assert.equal(h.calls.filter(call => call.options.method === 'POST').length, 0);
  h.queue('/motion', response(limitsStatus({busy:true,position:{az:30,el:40}})));
  await h.emit(h.el('use_el_max'), 'click');
  assert.notEqual(Number(h.el('el_max').value), 40);
  assert.match(h.el('limitsMessage').textContent, /idle/i);
});

test('apply validates drafts and only sends four limits after fresh idle status', async () => {
  const h = harness(); h.eval('frozen={id:"frozen"}; updateMotionButtons()');
  const values = {az_min:'-30',az_max:'40',el_min:'0',el_max:'80'};
  for (const [id,value] of Object.entries(values)) h.el(id).value = value;
  for (const invalid of ['', 'NaN', 'Infinity', '40']) {
    h.el('az_min').value = invalid; await h.emit(h.el('applyLimits'), 'click');
    assert.equal(h.requests('/motion/limits').length, 0);
  }
  h.el('az_min').value = '-30';
  h.queue('/motion', response(limitsStatus()));
  const applied = limitsStatus(); applied.limits = {soft_limits_deg:{az:[-30,40],el:[0,80]},acknowledged:true};
  h.queue('/motion/limits', response(applied));
  await h.emit(h.el('applyLimits'), 'click');
  assert.deepEqual(h.requests('/motion/limits').map(call => call.body),
    [{az_min:-30,az_max:40,el_min:0,el_max:80}]);
  assert.equal(h.requests('/motion/limits')[0].options.headers['X-DET-Token'], 'test-token');
  assert.equal(h.requests('/motion/stop').length, 0);
});

test('limit reconfirmation allows Apply but busy or hardware recovery does not', async () => {
  const h = harness();
  h.eval('motionStatus={connected:true,busy:false,last_error:"Reapply limits",limits_confirmation_required:true};updateMotionButtons()');
  assert.equal(h.el('applyLimits').disabled, false);
  h.eval('motionStatus.recovery_required=true;updateMotionButtons()');
  assert.equal(h.el('applyLimits').disabled, true);
  assert.equal(h.el('motionStop').disabled, false);
  h.eval('motionStatus.recovery_required=false;motionStatus.busy=true;updateMotionButtons()');
  assert.equal(h.el('use_az_min').disabled, true);
});

test('movement preference survives reload without issuing any command', async () => {
  const h = harness(); await h.mode('continuous');
  h.el('jogSpeed').value = '0.75'; await h.emit(h.el('jogSpeed'), 'input');
  h.el('jogSize').value = '2.5'; await h.emit(h.el('jogSize'), 'input');
  const reloaded = harness(Object.fromEntries(h.storage));
  assert.equal(reloaded.el('jogMode').value, 'continuous');
  assert.equal(Number(reloaded.el('jogSpeed').value), 0.75);
  assert.equal(Number(reloaded.el('jogSize').value), 2.5);
  assert.equal(reloaded.calls.length, 0);
});

async function main() {
  let failed = 0;
  for (const {name, fn} of tests) {
    try { await fn(); console.log('PASS ' + name); }
    catch (error) { failed++; console.error('FAIL ' + name + '\n' + error.stack); }
  }
  console.log(`${tests.length - failed}/${tests.length} host-only viewfinder hold tests passed`);
  if (failed) process.exitCode = 1;
}
main().catch(error => { console.error(error); process.exitCode = 1; });
