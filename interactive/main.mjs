import * as THREE from 'three';
import {loadVisualLod} from './visual-lod.mjs?v=9';
import {OrbitControls} from './vendor/OrbitControls.js';
const $=id=>document.getElementById(id), host=$('view');
if(new URLSearchParams(location.search).get('embed')==='1')document.body.classList.add('embedded');
const tasks={screwing:{title:'Screwing',description:'The trained policy assembles a threaded table leg, then releases it.',mpr:true},tight_insertion:{title:'Tight Insertion',description:'The trained policy inserts an L-shaped peg into a close-fitting hole, then releases it.',mpr:false}};
tasks.fabrica={title:'Multi-Part Assembly',description:'Two trained policies assemble the parts in sequence.',mpr:true};
tasks.plug={title:'iPhone Charger into Socket',description:'The trained policy plugs a real-size power adapter into a power board, then releases it.',mpr:true};
tasks.fork={title:'YCB Fork in Rack',description:'The trained policy slides a real-size YCB fork, handle first, into a rack, then releases it.',mpr:true};
const taskName=new URLSearchParams(location.search).get('task')??'screwing';
if(!tasks[taskName])throw Error('Unknown task');
const task=tasks[taskName],assets=`./assets/${taskName}/`;
$('task').value=taskName;$('task-description').textContent=task.description;
$('task').onchange=()=>{const url=new URL(location.href);url.searchParams.set('task',$('task').value);url.searchParams.delete('start');location.href=url.href;};

const fastRendering=new URLSearchParams(location.search).get('render')==='fast';
$('faster').checked=fastRendering;
$('faster').onchange=()=>{const url=new URL(location.href);if($('faster').checked)url.searchParams.set('render','fast');else url.searchParams.delete('render');location.href=url.href;};
const visualLod=fastRendering?await loadVisualLod(new URL(assets+'visual-lod.json',location.href)):null;
const renderer=new THREE.WebGLRenderer({antialias:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));host.prepend(renderer.domElement);
renderer.setClearColor(0xf0f3f5);renderer.outputColorSpace=THREE.SRGBColorSpace;
let renderDirty=true;
const scene=new THREE.Scene();scene.background=new THREE.Color(0xf0f3f5);scene.up.set(0,0,1);
const camera=new THREE.PerspectiveCamera(42,1,.01,20);camera.up.set(0,0,1);
// Opening shot: the direction is fixed, VIEW_DISTANCE tightens the framing.
// A portrait screen sees a narrower horizontal field, so it pulls back to keep
// the parts in frame. Applied once; orbiting afterwards is never overridden.
const VIEW_TARGET=new THREE.Vector3(0,0,.68),VIEW_DIRECTION=new THREE.Vector3(.7,-1,.37).normalize(),VIEW_DISTANCE=.83;
camera.position.copy(VIEW_TARGET).addScaledVector(VIEW_DIRECTION,VIEW_DISTANCE);
const orbit=new OrbitControls(camera,renderer.domElement);orbit.target.copy(VIEW_TARGET);orbit.update();orbit.addEventListener('change',()=>{renderDirty=true;});
scene.add(new THREE.HemisphereLight(0xffffff,0x71808b,2.4));
const sun=new THREE.DirectionalLight(0xffffff,2.5);sun.position.set(1,2,3);scene.add(sun);
let framed=false;
new ResizeObserver(()=>{const w=host.clientWidth,h=host.clientHeight;renderer.setSize(w,h);camera.aspect=w/h;camera.updateProjectionMatrix();
 if(!framed&&w>0&&h>0){framed=true;camera.position.copy(VIEW_TARGET).addScaledVector(VIEW_DIRECTION,VIEW_DISTANCE*(w<h?1.3:1));orbit.update();}
 renderDirty=true;}).observe(host);
let paused=true,ready=false,control,model,data,metadata,rnnMax=0,resetArrangements=0,stepMs=0,resetSource='none',resetIndex=-1;
const worker=new Worker(new URL('./simulation-worker.mjs?v=9',import.meta.url),{type:'module'});
// Do not keep WASM heaps alive in a cached page after task changes/navigation.
window.addEventListener('pagehide',()=>{worker.terminate();renderer.dispose();});
window.addEventListener('pageshow',event=>{if(event.persisted)location.reload();});
const meshes=[];let renderFrames=0,fpsStart=performance.now(),loadMs=0,fps=null;
function status(){
 const ended=control.succeeded||control.failed;
 $('play-label').textContent=paused?'Play':'Pause';
 $('play-icon').textContent=paused?'▶️':'⏸️';
 $('play').disabled=ended;
 $('play').title=ended?'Press Reset to try again':'';
 $('reset').classList.toggle('restart',ended);
 // A disturbance only lands while the policy is actually stepping.
 $('disturb').disabled=paused||ended;
 $('disturb').title=ended?'Start a new attempt first':paused?'Press Play first':'Knock the part off course';
 $('status').textContent=control.succeeded?'Assembly complete · Reset to a new init to try again':control.failed?'Attempt ended · Reset to a new init to try again':paused?'Paused · press Play to start':control.retract?'Releasing the part…':'Policy running';
}
$('play').onclick=()=>{if(!ready||control.succeeded||control.failed)return;paused=!paused;worker.postMessage({type:'play',paused});status();};
if(taskName==='fabrica')$('reset').title=new URLSearchParams(location.search).has('start')?'Repeat this starting arrangement':'Try another starting arrangement';
$('reset').onclick=()=>{if(ready){paused=true;worker.postMessage({type:'reset'});}};
$('disturb').onclick=()=>{if(ready&&!paused)worker.postMessage({type:'disturb'});};
window.addEventListener('keydown',event=>{if(!ready||event.repeat||event.target.matches('input,select,textarea,button'))return;
 if(['Space','Backspace'].includes(event.code)){event.preventDefault();$(event.code==='Space'?'play':'reset').click();}});
// Force arrow: length and colour both track magnitude, green through red.
// Drawn on top of the scene so a shove into an occluded part still reads.
const DISTURB_VIEW_MS=700;
const arrowMaterial=new THREE.MeshBasicMaterial({transparent:true,depthTest:false,depthWrite:false});
const arrow=new THREE.Group(),arrowShaft=new THREE.Mesh(new THREE.CylinderGeometry(1,1,1,16),arrowMaterial),arrowHead=new THREE.Mesh(new THREE.ConeGeometry(1,1,20),arrowMaterial);
arrow.add(arrowShaft,arrowHead);arrow.renderOrder=20;arrow.visible=false;scene.add(arrow);
let disturbance=null,shownEpoch=0;
function drawDisturbance(now){
 const age=disturbance?now-disturbance.start:Infinity;
 arrow.visible=age<DISTURB_VIEW_MS;
 if(!arrow.visible)return;
 const strength=disturbance.strength;
 const length=.06+.13*strength,head=.32*length,shaft=length-head;
 arrowShaft.scale.set(.055*length,shaft,.055*length);arrowShaft.position.set(0,shaft/2,0);
 arrowHead.scale.set(.13*length,head,.13*length);arrowHead.position.set(0,shaft+head/2,0);
 arrowMaterial.color.setHSL(.33*(1-strength),.85,.48);
 arrowMaterial.opacity=1-(age/DISTURB_VIEW_MS)**2;
 // Tip rests on the part and the shaft trails behind, so it reads as a push.
 const tip=new THREE.Vector3().fromArray(data.qpos,disturbance.address);
 arrow.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0),disturbance.direction);
 arrow.position.copy(tip).addScaledVector(disturbance.direction,-length);
}
function addGeometries(){
 for(let i=0;i<model.ngeom;i++){
  if(model.geom_group[i]>=3||model.geom_bodyid[i]===model.goalBodyId)continue;
  const type=model.geom_type[i],size=Array.from(model.geom_size.slice(3*i,3*i+3));let geometry;
  if(type===7||type===8){
   const id=model.geom_dataid[i],va=model.mesh_vertadr[id],vn=model.mesh_vertnum[id],fa=model.mesh_faceadr[id],fn=model.mesh_facenum[id];
   const reduced=visualLod?.[id];
   geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.Float32BufferAttribute(reduced?.positions??Array.from(model.mesh_vert.slice(3*va,3*(va+vn))),3));
   geometry.setIndex(reduced?new THREE.BufferAttribute(reduced.indices,1):Array.from(model.mesh_face.slice(3*fa,3*(fa+fn))));
   if(reduced?.normals)geometry.setAttribute('normal',new THREE.Float32BufferAttribute(reduced.normals,3));else geometry.computeVertexNormals();
  }else if(type===6)geometry=new THREE.BoxGeometry(...size.map(v=>2*v));
  else if(type===2)geometry=new THREE.SphereGeometry(size[0],24,16);
  else if(type===0)geometry=new THREE.PlaneGeometry(4,4);
  else if(type===5){geometry=new THREE.CylinderGeometry(size[0],size[0],2*size[1],24);geometry.rotateX(Math.PI/2);}
  else if(type===3){geometry=new THREE.CapsuleGeometry(size[0],2*size[1],8,16);geometry.rotateX(Math.PI/2);}
  else throw Error(`Unsupported visual geom type ${type}`);
  const rgba=model.geom_rgba.slice(i*4,i*4+4);
  const material=new THREE.MeshStandardMaterial({color:new THREE.Color(...rgba.slice(0,3)),roughness:.55,metalness:.1,transparent:rgba[3]<1,opacity:rgba[3],depthWrite:rgba[3]>=1});
  // Presentation palette only: MuJoCo geoms and contact parameters are untouched.
  const body=model.geom_bodyid[i];
  const palette=taskName==='screwing'?{part:'#eeeae2',fixture:'#858d94'}:taskName==='plug'?{part:'#f2f2f0',fixture:'#4a4f55'}:taskName==='fork'?{part:'#d3d8dd',fixture:'#858d94'}:taskName==='tight_insertion'?{part:'#5fb8e6',fixture:'#eeede8'}:{part:'#c7353b',fixture:taskName==='fabrica'?'#c7353b':'#eeede8'};
  if(body===model.tableBodyId){material.color.set('#b58a60');material.roughness=.85;material.metalness=0;}
  else if(body===model.fixtureBodyId){material.color.set(palette.fixture);material.roughness=.7;material.metalness=0;}
  else if(model.partIds.includes(body)){material.color.set(palette.part);material.roughness=.6;material.metalness=0;}
  // Per-geom PBR materials exported from the source GLB (manifest.visual_materials, keyed by geom index)
  // override the palette so the browser matches the simulator's look.
  const pbr=metadata.visual_materials?.[i];
  if(pbr){material.color.setRGB(rgba[0],rgba[1],rgba[2]);material.metalness=Math.min(pbr.metalness,.85);material.roughness=pbr.roughness;}
  const mesh=new THREE.Mesh(geometry,material);scene.add(mesh);meshes.push({mesh,id:i});
 }
}
function draw(){
 for(const {mesh,id} of meshes){mesh.position.fromArray(data.geom_xpos,3*id);
  const a=data.geom_xmat.slice(9*id,9*id+9),mat=new THREE.Matrix4();mat.set(a[0],a[1],a[2],0,a[3],a[4],a[5],0,a[6],a[7],a[8],0,0,0,0,1);mesh.quaternion.setFromRotationMatrix(mat);
 }
 drawDisturbance(performance.now());
 orbit.update();renderer.render(scene,camera);
}
function frame(now){
 if(disturbance&&now-disturbance.start<DISTURB_VIEW_MS+40)renderDirty=true;
 if(ready&&(!paused||renderDirty)){
  renderDirty=false;draw();
  if(!paused){renderFrames++;if(now-fpsStart>1000){fps=Math.round(renderFrames*1000/(now-fpsStart));renderFrames=0;fpsStart=now;}}
 }
 if(paused){renderFrames=0;fpsStart=now;}
 requestAnimationFrame(frame);
}
const loadStart=performance.now();
function accept(frame){
 renderDirty=true;
 paused=frame.paused;control=frame.control;data=frame.data;rnnMax=frame.rnnMax;stepMs=frame.stepMs;resetIndex=frame.resetIndex??-1;
 if(control.resetEpoch!==shownEpoch){shownEpoch=control.resetEpoch;disturbance=null;arrow.visible=false;}
 control.metadata={...metadata,...(metadata.stages?.[control.stageIndex]??{})};status();
}
let loadFailed=false;
$('retry').onclick=()=>location.reload();
function failure(error){$('loading-panel').hidden=false;$('loading-note').textContent=error;$('loading-retry').hidden=false;loadFailed=true;ready=false;$('status').textContent='Unable to run the demo';$('error').textContent=error;$('retry').hidden=false;for(const id of ['play','reset','disturb'])$(id).disabled=true;console.error(error);paused=true;}
worker.onerror=event=>failure(event.message);
worker.onmessage=({data:message})=>{
 if(message.type==='error'){failure(message.error);return;}
 if(message.type==='loading'){if(!loadFailed)$('status').textContent=message.label;return;}
 if(message.type==='disturbance'){
  disturbance={direction:new THREE.Vector3(...message.force).normalize(),strength:message.strength,newtons:message.newtons,address:message.address,start:performance.now()};
  renderDirty=true;return;
 }
 if(message.type==='ready'){
  metadata=message.metadata;model=message.model;resetArrangements=message.resetArrangements??0;resetSource=message.resetSource??'none';accept(message.frame);addGeometries();ready=true;$('loading-panel').hidden=true;
  loadMs=performance.now()-loadStart;fpsStart=performance.now();
  for(const id of ['play','reset'])$(id).disabled=false;status();
 }else if(message.type==='state')accept(message.frame);
};
window.demoResetTo=index=>{if(ready){paused=true;worker.postMessage({type:'reset',arrangement:index});}};
window.demoCamera=()=>{const r=v=>Number(v.toFixed(3)),p=camera.position,t=orbit.target;
 return {position:[r(p.x),r(p.y),r(p.z)],target:[r(t.x),r(t.y),r(t.z)],distance:r(p.distanceTo(t))};};
window.demoStatus=()=>({task:taskName,fastRendering,startIndex:control?.startIndex??0,resetEpoch:control?.resetEpoch??0,stage:control?.stageIndex??0,handoffs:control?.handoffs??[],ready,paused,steps:control?.steps??0,goals:control?.successes??0,succeeded:control?.succeeded??false,failed:control?.failed??false,distance:control?.distance,loadMs,qpos:data?Array.from(data.qpos):[],qvel:data?Array.from(data.qvel):[],targets:control?.targets??[],rnnMax,fps,resetArrangements,resetSource,stepMs,resetIndex});
const startParam=new URLSearchParams(location.search).get('start');
const disturbParam=new URLSearchParams(location.search).get('disturb');
const spreadParam=new URLSearchParams(location.search).get('spread');
const sdfParam=new URLSearchParams(location.search).get('sdf');
const substepParam=new URLSearchParams(location.search).get('substep');
worker.postMessage({type:'init',start:startParam===null?undefined:Number(startParam),task:taskName,assets:new URL(assets,location.href).href,mpr:task.mpr,disturb:disturbParam===null?undefined:Number(disturbParam),spread:spreadParam===null?undefined:Number(spreadParam),sdf:sdfParam===null?undefined:Number(sdfParam),substep:substepParam===null?undefined:Number(substepParam)});
requestAnimationFrame(frame);
