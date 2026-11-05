const assert = require('node:assert/strict');
const {screenToNative, nativeToScreen, mapPoint} = require('./geometry.js');
for (const rotation of [0,90,180,270]) {
  const screen = nativeToScreen([.2,.7],rotation);
  const native = screenToNative(screen,rotation);
  assert(Math.abs(native[0]-.2)<1e-12 && Math.abs(native[1]-.7)<1e-12);
}
assert.deepEqual(screenToNative([.2,.3],90),[.3,.8]);
assert.deepEqual(nativeToScreen([.2,.3],270),[.3,.8]);
assert.deepEqual(mapPoint([200,300],{a:[100,200],b:[300,200]},{a:[50,80],b:[50,180]}),[0,130]);
assert.throws(()=>mapPoint([0,0],{a:[1,1],b:[1,1]},{a:[0,0],b:[1,1]}));
console.log('Coordinate rotation and cross-camera mapping checks passed');
