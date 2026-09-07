import {SimulationRate} from './simulation-rate.mjs';
import * as THREE from 'three';
import {loadVisualLod} from './visual-lod.mjs';
import {OrbitControls} from './vendor/OrbitControls.js';
const $=id=>document.getElementById(id), host=$('view');
if(new URLSearchParams(location.search).get('embed')==='1')document.body.classList.add('embedded');
const tasks={screwing:{title:'Screwing',description:'The trained policy assembles a threaded table leg, then releases it.',detail:'225 mm leg · Trained policy',mpr:true},tight_insertion:{title:'Tight insertion',description:'The trained policy inserts an L-shaped peg into a close-fitting hole, then releases it.',detail:'0.5 mm clearance · Trained policy',mpr:false}};
tasks.fabrica={title:'Fabrica assembly',description:'Two trained policies assemble the parts in sequence, keeping both parts free to move.',detail:'3× parts · Two trained policies',mpr:true};
const taskName=new URLSearchParams(location.search).get('task')??'screwing';
if(!tasks[taskName])throw Error('Unknown task');
const task=tasks[taskName],assets=`./assets/${taskName}/`;
$('task').value=taskName;$('task-title').textContent=task.title;$('task-description').textContent=task.description;$('task-detail').textContent=task.detail+'';
$('task').onchange=()=>{const url=new URL(location.href);url.searchParams.set('task',$('task').value);url.searchParams.delete('start');location.href=url.href;};

const fastRendering=new URLSearchParams(location.search).get('render')==='fast';
$('faster').checked=fastRendering;
$('faster').onchange=()=>{const url=new URL(location.href);if($('faster').checked)url.searchParams.set('render','fast');else url.searchParams.delete('render');location.href=url.href;};
const visualLod=fastRendering?await loadVisualLod(new URL(assets+'visual-lod.json',location.href)):null;
const renderer=new THREE.WebGLRenderer({antialias:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));host.prepend(renderer.domElement);
renderer.setClearColor(0xf0f3f5);renderer.outputColorSpace=THREE.SRGBColorSpace;
let renderDirty=true;
for(const id of ['goal','axes'])$(id).addEventListener('change',()=>{renderDirty=true;});
const scene=new THREE.Scene();scene.background=new THREE.Color(0xf0f3f5);scene.up.set(0,0,1);
const camera=new THREE.PerspectiveCamera(42,1,.01,20);camera.up.set(0,0,1);camera.position.set(.7,-1.0,1.0);
const orbit=new OrbitControls(camera,renderer.domElement);orbit.target.set(0,0,.63);orbit.update();orbit.addEventListener('change',()=>{renderDirty=true;});
scene.add(new THREE.HemisphereLight(0xffffff,0x71808b,2.4));
const sun=new THREE.DirectionalLight(0xffffff,2.5);sun.position.set(1,2,3);scene.add(sun);
new ResizeObserver(()=>{const w=host.clientWidth,h=host.clientHeight;renderer.setSize(w,h);camera.aspect=w/h;camera.updateProjectionMatrix();renderDirty=true;}).observe(host);
let paused=true,ready=false,control,model,data,metadata,rnnMax=0;
const worker=new Worker(new URL('./simulation-worker.mjs',import.meta.url),{type:'module'});
const meshes=[];let lastFrame=performance.now(),renderFrames=0,fpsStart=lastFrame,loadMs=0;
const rateMeter=new SimulationRate();let simulationRate=null;
function showRate(){
 $('sim-rate').textContent=paused?'Paused':simulationRate===null?'Measuring…':`${simulationRate.toFixed(2)}× real time`;
}
function status(){
 showRate();
 $('play').textContent=control.succeeded||control.failed?'Replay':paused?'Play':'Pause';
 $('status').textContent=control.succeeded?'Assembly complete · Replay to try again':control.failed?'Attempt ended · Replay to try again':paused?'Paused · ready to play':control.retract?'Releasing the part…':'Policy running';
 $('goals').textContent=taskName==='fabrica'?`${control.stageIndex*2+control.successes} / 4 · Part ${control.stageIndex+1}/2`:`${control.successes} / ${control.metadata.goals.length}`;$('time').textContent=`${data.time.toFixed(2)} s`;
 $('distance').textContent=Number.isFinite(control.distance)?`${(control.distance*1000).toFixed(2)} mm`:'—';
}
$('play').onclick=()=>{if(!ready)return;if(control.succeeded||control.failed){worker.postMessage({type:'reset',play:true});return;}paused=!paused;worker.postMessage({type:'play',paused});status();};
if(taskName==='fabrica')$('reset').title=new URLSearchParams(location.search).has('start')?'Repeat this starting arrangement':'Try another starting arrangement';
$('reset').onclick=()=>{if(ready){paused=true;worker.postMessage({type:'reset'});}};
window.addEventListener('keydown',event=>{if(!ready||event.repeat||event.target.matches('input,select,textarea,button'))return;
 if(['Space','Backspace'].includes(event.code)){event.preventDefault();$(event.code==='Space'?'play':'reset').click();}});
const poseAxes=[];
function makePoseAxes(address=null){
 const group=new THREE.Group();
 for(const [direction,color] of [[[1,0,0],0xff3030],[[0,1,0],0x20bb40],[[0,0,1],0x3070ff]]){
  const arrow=new THREE.ArrowHelper(new THREE.Vector3(...direction),new THREE.Vector3(),.07,color,.014,.007);
  // Frame origins can be inside the part: keep all three directions readable.
  for(const mesh of [arrow.line,arrow.cone]){mesh.material.depthTest=false;mesh.material.depthWrite=false;mesh.renderOrder=10;}
  group.add(arrow);
 }
 group.visible=false;scene.add(group);poseAxes.push({group,address});
}
function updatePoseAxes(){
 for(const {group,address} of poseAxes){
  const goal=address===null;
  group.visible=$('axes').checked&&(!goal||$('goal').checked);
  if(goal){group.position.fromArray(control.goal);group.quaternion.fromArray(control.goal,3);}
  else {group.position.fromArray(data.qpos,address);group.quaternion.set(data.qpos[address+4],data.qpos[address+5],data.qpos[address+6],data.qpos[address+3]);}
 }
}
function addGeometries(){
 const addresses=metadata.stages?.map(s=>s.object_qpos_address)??[metadata.object_qpos_address??29];
 for(const address of [...new Set(addresses)])makePoseAxes(address);
 makePoseAxes();
 for(let i=0;i<model.ngeom;i++){
  const isGoal=model.geom_bodyid[i]===model.goalBodyId;
  if(model.geom_group[i]>=3||(taskName==='fabrica'&&isGoal))continue;
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
  const palette=taskName==='screwing'?{part:'#eeeae2',fixture:'#858d94'}:{part:'#c7353b',fixture:taskName==='fabrica'?'#c7353b':'#eeede8'};
  if(body===model.tableBodyId){material.color.set('#b58a60');material.roughness=.85;material.metalness=0;}
  else if(body===model.fixtureBodyId){material.color.set(palette.fixture);material.roughness=.7;material.metalness=0;}
  else if(model.partIds.includes(body)){material.color.set(palette.part);material.roughness=.6;material.metalness=0;}
  const mesh=new THREE.Mesh(geometry,material);scene.add(mesh);meshes.push({mesh,id:i,goal:isGoal});
  if(taskName==='fabrica'){
   const partIds=model.partIds;
   const stage=partIds.indexOf(model.geom_bodyid[i]);
   if(stage>=0){const ghost=mesh.clone();ghost.material=material.clone();ghost.material.color.setRGB(.2,.9,.3);ghost.material.opacity=.2;ghost.material.transparent=true;ghost.material.depthWrite=false;scene.add(ghost);meshes.push({mesh:ghost,id:i,goal:true,stage});}
  }
 }
}
function draw(){
 updatePoseAxes();
 let ghostTransform;
 if(taskName==='fabrica'){
  const goal=control.goal,a=control.metadata.object_qpos_address;
  const target=new THREE.Matrix4().compose(new THREE.Vector3(...goal.slice(0,3)),new THREE.Quaternion(...goal.slice(3)),new THREE.Vector3(1,1,1));
  const current=new THREE.Matrix4().compose(new THREE.Vector3(...data.qpos.slice(a,a+3)),new THREE.Quaternion(data.qpos[a+4],data.qpos[a+5],data.qpos[a+6],data.qpos[a+3]),new THREE.Vector3(1,1,1));
  ghostTransform=target.multiply(current.invert());
 }
 for(const {mesh,id,goal,stage} of meshes){mesh.visible=(!goal||$('goal').checked)&&(stage===undefined||stage===control.stageIndex);mesh.position.fromArray(data.geom_xpos,3*id);
  const a=data.geom_xmat.slice(9*id,9*id+9),mat=new THREE.Matrix4();mat.set(a[0],a[1],a[2],0,a[3],a[4],a[5],0,a[6],a[7],a[8],0,0,0,0,1);mesh.quaternion.setFromRotationMatrix(mat);
  if(stage!==undefined){mesh.scale.set(1,1,1);mesh.updateMatrix();const world=ghostTransform.clone().multiply(mesh.matrix);world.decompose(mesh.position,mesh.quaternion,mesh.scale);}
 }
 orbit.update();renderer.render(scene,camera);
}
function frame(now){
 if(ready){simulationRate=rateMeter.sample(data.time,now,paused,control.resetEpoch);showRate();}
 if(ready&&(!paused||renderDirty)){
  renderDirty=false;draw();
  if(!paused){renderFrames++;if(now-fpsStart>1000){$('fps').textContent=`${Math.round(renderFrames*1000/(now-fpsStart))} fps`;renderFrames=0;fpsStart=now;}}
 }
 if(paused){renderFrames=0;fpsStart=now;}
 requestAnimationFrame(frame);
}
const loadStart=performance.now();
function accept(frame){
 renderDirty=true;
 paused=frame.paused;control=frame.control;data=frame.data;rnnMax=frame.rnnMax;
 control.metadata={...metadata,...(metadata.stages?.[control.stageIndex]??{})};status();
}
let loadFailed=false;
$('retry').onclick=()=>location.reload();
function failure(error){$('loading-panel').hidden=false;$('loading-note').textContent=error;$('loading-retry').hidden=false;loadFailed=true;ready=false;$('status').textContent='Unable to run the demo';$('error').textContent=error;$('retry').hidden=false;for(const id of ['play','reset'])$(id).disabled=true;console.error(error);paused=true;}
worker.onerror=event=>failure(event.message);
worker.onmessage=({data:message})=>{
 if(message.type==='error'){failure(message.error);return;}
 if(message.type==='loading'){if(!loadFailed)$('status').textContent=message.label;return;}
 if(message.type==='ready'){
  metadata=message.metadata;model=message.model;accept(message.frame);addGeometries();ready=true;$('loading-panel').hidden=true;
  loadMs=performance.now()-loadStart;fpsStart=performance.now();
  for(const id of ['play','reset'])$(id).disabled=false;
 }else if(message.type==='state')accept(message.frame);
};
window.demoStatus=()=>({poseAxes:poseAxes.map(({group,address})=>({address,visible:group.visible,position:group.position.toArray(),quaternion:group.quaternion.toArray()})),task:taskName,fastRendering,startIndex:control?.startIndex??0,resetEpoch:control?.resetEpoch??0,stage:control?.stageIndex??0,handoffs:control?.handoffs??[],ready,paused,steps:control?.steps??0,goals:control?.successes??0,succeeded:control?.succeeded??false,failed:control?.failed??false,distance:control?.distance,loadMs,qpos:data?Array.from(data.qpos):[],qvel:data?Array.from(data.qvel):[],targets:control?.targets??[],rnnMax,visibleGoalGeoms:meshes.filter(v=>v.goal&&v.mesh.visible).length});
const startParam=new URLSearchParams(location.search).get('start');
worker.postMessage({type:'init',start:startParam===null?undefined:Number(startParam),task:taskName,assets:new URL(assets,location.href).href,mpr:task.mpr});
requestAnimationFrame(frame);
