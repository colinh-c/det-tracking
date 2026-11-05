(function(root) {
  function screenToNative(p,r) {
    const [x,y]=p;
    if(r===90) return [y,1-x];
    if(r===180) return [1-x,1-y];
    if(r===270) return [1-y,x];
    return [x,y];
  }
  function nativeToScreen(p,r) { return screenToNative(p,(360-r)%360); }
  function mapPoint(p,source,destination) {
    const [a,b,c,d]=[source.a,source.b,destination.a,destination.b];
    const ux=b[0]-a[0],uy=b[1]-a[1],vx=d[0]-c[0],vy=d[1]-c[1];
    const norm=ux*ux+uy*uy;
    if(norm<1) throw new Error('Dot A and Dot B must be distinct');
    const re=(vx*ux+vy*uy)/norm,im=(vy*ux-vx*uy)/norm;
    return [c[0]+re*(p[0]-a[0])-im*(p[1]-a[1]),c[1]+im*(p[0]-a[0])+re*(p[1]-a[1])];
  }
  const api={screenToNative,nativeToScreen,mapPoint};
  if(typeof module!=='undefined') module.exports=api;
  else Object.assign(root,api);
})(typeof window!=='undefined'?window:globalThis);
