import {describeError} from './error-details.mjs?v=8';
import loadMuJoCo from './vendor/mujoco.js';
import * as ort from './vendor/ort.wasm.min.mjs';
import {AssemblyControl,FabricaControl} from './controller.mjs?v=8';
let mj,model,data,control,sessions,h,c,paused=true,single=false,busy=false,epoch=0,timer,deadline=0,ready=false,scheduleVersion=0;
let fixedStart;
let releaseControls=0,releaseRemaining=null,completed=false;
let sceneMetadata,disturbRemaining=0,disturbBody=-1,disturbScale=1,stepMs=0;
// Disturbance sizing: a wrench held for a few control steps, scaled by the
// part's own mass and inertia so every task gets a comparable shove. SPEED and
// SPIN are the velocity change the part would take on if it were free; a firm
// grasp absorbs most of it, so these run well above a free-body nudge. Each
// disturbance draws a fraction of that ceiling, from a light tap to a full shove.
const DISTURB_CONTROLS=5,DISTURB_SPEED=4,DISTURB_SPIN=30,DISTURB_MIN=.15;
// Reset scatters the parts and the fixture over the table top, so each attempt
// looks different. Goals ride with the fixture; footprints never overlap.
const RESET_FIXTURE_YAW=.175,RESET_CLEARANCE_M=.03,RESET_TRIES=80;
const RESET_SETTLE_SECONDS=.9,RESET_POOL=10,RESET_ATTEMPTS=60,RESET_PENETRATION_M=.0005,RESET_REST_SPEED=.05;
// Circumscribing circles are pessimistic for long thin parts, so packing relaxes
// until a spot is found; settling plus the contact and mesh checks are what
// actually rule an arrangement out. Edge inset is partial for the same reason:
// a part that really does overhang tips off and fails validation.
const RESET_PACKING=[1,.75,.55,.4],RESET_EDGE_INSET=.7;
// Threaded parts collide through signed-distance fields, and sdf_initpoints is how
// many starting points the collider seeds along a surface. Only the screwing task
// has a helix fine enough to care; every other scene keeps its compiled values,
// which are known good and cheaper. ?sdf=N overrides for experiments.
const SDF_TUNING={};
// Parts are dropped from a uniformly random orientation and left to settle, so the
// resting pose is whatever that shape can actually hold rather than a chosen face.
function randomQuatWXYZ(){
 const u1=Math.random(),u2=2*Math.PI*Math.random(),u3=2*Math.PI*Math.random();
 const a=Math.sqrt(1-u1),b=Math.sqrt(u1);
 return [b*Math.cos(u3),a*Math.sin(u2),a*Math.cos(u2),b*Math.sin(u3)];
}
let layout=null,baseGoals=null,spreadScale=1,resetPool=[],resetSource='none',resetIndex=-1;
let operation='Starting demo';
const wake=new MessageChannel();
wake.port1.onmessage=({data:version})=>{if(version===scheduleVersion)tick();};
const empty=()=>new ort.Tensor('float32',new Float32Array(1024),[1,1,1024]);
function resetRnn(){h=empty();c=empty();}
function snapshot(){
 return {paused,control:{resetEpoch:epoch,startIndex:control.startIndex??0,steps:control.steps,stageIndex:control.stageIndex??0,handoffs:control.handoffs??[],successes:control.successes,
   succeeded:completed,failed:control.failed,retract:control.retract,distance:control.distance,goal:control.goal,targets:Array.from(control.targets)},
  data:{qpos:data.qpos.slice(),qvel:data.qvel.slice(),geom_xpos:data.geom_xpos.slice(),geom_xmat:data.geom_xmat.slice(),time:data.time},
  rnnMax:Math.max(...Array.from(h.data,Math.abs),...Array.from(c.data,Math.abs)),stepMs,resetIndex};
}
function emit(){postMessage({type:'state',frame:snapshot()});}
function randomDirection(flatten=1){
 for(;;){
  const v=[0,0,0].map(()=>2*Math.random()-1),n=Math.hypot(...v);
  if(n<=.1||n>1)continue;
  // Bias away from vertical: a downward shove just presses the part into the
  // fixture, where a sideways one is what the policy visibly has to recover from.
  const b=[v[0],v[1],v[2]*flatten],m=Math.hypot(...b);
  if(m>.1)return b.map(x=>x/m);
 }
}
// The active part owns the free joint the controller reads its pose from.
function activeObjectAddress(){return control.metadata.object_qpos_address??29;}
function activeObjectBody(){
 const address=activeObjectAddress();
 for(let j=0;j<model.njnt;j++)if(model.jnt_qposadr[j]===address)return model.jnt_bodyid[j];
 throw Error('No free joint at the object address');
}
function clearDisturbance(){
 if(disturbBody>=0)data.xfrc_applied.set([0,0,0,0,0,0],6*disturbBody);
 disturbBody=-1;disturbRemaining=0;
}
function applyDisturbance(){
 clearDisturbance();
 const body=activeObjectBody(),seconds=DISTURB_CONTROLS*control.metadata.decimation*model.opt.timestep;
 const mass=model.body_mass[body],inertia=model.body_inertia.slice(3*body,3*body+3);
 const spin=(inertia[0]+inertia[1]+inertia[2])/3;
 const strength=DISTURB_MIN+(1-DISTURB_MIN)*Math.random();
 const force=randomDirection(.35).map(v=>v*mass*DISTURB_SPEED*disturbScale*strength/seconds);
 const torque=randomDirection().map(v=>v*spin*DISTURB_SPIN*disturbScale*strength/seconds);
 data.xfrc_applied.set([...force,...torque],6*body);
 disturbBody=body;disturbRemaining=DISTURB_CONTROLS;
 // The renderer draws this for longer than the impulse lasts, so it is visible.
 // strength is the fraction of this part's ceiling, so the arrow spans its full
 // colour range on every task instead of tracking the part's mass.
 postMessage({type:'disturbance',force,newtons:Math.hypot(...force),strength,address:activeObjectAddress()});
}
// Fixed starts stay bit-exact for the diagnostics page; only the button jitters.
// Table rectangle, and a circumscribing xy radius for every movable item, all
// measured once from the compiled model. Radii are rotation-invariant, so a
// yawed part still clears its neighbours.
function prepareLayout(){
 const table=mj.mj_name2id(model,1,'table');
 if(table<0)return;
 let minX=Infinity,maxX=-Infinity,minY=Infinity,maxY=-Infinity,top=-Infinity;
 for(let g=0;g<model.ngeom;g++)if(model.geom_bodyid[g]===table){
  const x=data.geom_xpos[3*g],y=data.geom_xpos[3*g+1];
  minX=Math.min(minX,x-model.geom_size[3*g]);maxX=Math.max(maxX,x+model.geom_size[3*g]);
  minY=Math.min(minY,y-model.geom_size[3*g+1]);maxY=Math.max(maxY,y+model.geom_size[3*g+1]);
  top=Math.max(top,data.geom_xpos[3*g+2]+model.geom_size[3*g+2]);
 }
 if(!Number.isFinite(minX))return;
 const radiusOf=body=>{
  const originX=data.xpos[3*body],originY=data.xpos[3*body+1];let radius=0;
  for(let g=0;g<model.ngeom;g++)if(model.geom_bodyid[g]===body&&model.geom_group[g]<3)
   radius=Math.max(radius,Math.hypot(data.geom_xpos[3*g]-originX,data.geom_xpos[3*g+1]-originY)+model.geom_rbound[g]);
  return radius;
 };
 // A sphere bound clears the table at any orientation, so the drop stays short.
 const sphereOf=body=>{
  const origin=[0,1,2].map(k=>data.xpos[3*body+k]);let radius=0;
  for(let g=0;g<model.ngeom;g++)if(model.geom_bodyid[g]===body)
   radius=Math.max(radius,Math.hypot(...[0,1,2].map(k=>data.geom_xpos[3*g+k]-origin[k]))+model.geom_rbound[g]);
  return radius;
 };
 const fixtureBody=mj.mj_name2id(model,1,'assembly_fixture');
 const fixture=fixtureBody<0?null:{body:fixtureBody,radius:radiusOf(fixtureBody),
  basePos:Array.from(model.body_pos.slice(3*fixtureBody,3*fixtureBody+3)),
  baseQuat:Array.from(model.body_quat.slice(4*fixtureBody,4*fixtureBody+4))};
 const parts=[];
 for(const name of ['object','second']){
  const body=mj.mj_name2id(model,1,name);if(body<0)continue;
  const joint=model.body_jntadr[body];
  if(joint<0||model.jnt_type[joint]!==0)continue; // free joints only
  parts.push({body,address:model.jnt_qposadr[joint],dof:model.jnt_dofadr[joint],radius:radiusOf(body),sphere:sphereOf(body)});
 }
 if(!fixture&&!parts.length)return;
 layout={table:{minX,maxX,minY,maxY,top,body:table},fixture,parts};
 baseGoals=[sceneMetadata,...(sceneMetadata.stages??[])].map(owner=>owner.goals?.map(goal=>goal.slice()));
}
// Rejection sampling over the whole table top, keeping RESET_CLEARANCE_M of open
// space between footprints so nothing starts already in contact.
function samplePlacement(radius,placed,packing){
 const inset=radius*RESET_EDGE_INSET;
 const lowX=layout.table.minX+inset,highX=layout.table.maxX-inset;
 const lowY=layout.table.minY+inset,highY=layout.table.maxY-inset;
 if(highX<lowX||highY<lowY)return null;
 for(let attempt=0;attempt<RESET_TRIES;attempt++){
  const x=lowX+(highX-lowX)*Math.random(),y=lowY+(highY-lowY)*Math.random();
  if(placed.every(other=>Math.hypot(x-other.x,y-other.y)>=(other.radius+radius)*packing+RESET_CLEARANCE_M))return {x,y,radius};
 }
 return null;
}
// Every part must find a spot. Leaving one at its canonical pose would look like
// a reset that does nothing, so a partial arrangement fails the whole candidate.
function planLayout(){
 if(!layout)return null;
 for(const packing of RESET_PACKING){
  const placed=[],parts=[];
  const fixture=layout.fixture?samplePlacement(layout.fixture.radius,placed,packing):null;
  if(fixture)placed.push(fixture);
  let complete=true;
  for(const part of layout.parts){
   const spot=samplePlacement(part.radius,placed,packing);
   if(!spot){complete=false;break;}
   placed.push(spot);parts.push(spot);
  }
  if(complete&&(parts.length||fixture))
   return {fixture,parts,fixtureYaw:(2*Math.random()-1)*RESET_FIXTURE_YAW*spreadScale,
    partQuats:layout.parts.map(()=>randomQuatWXYZ())};
 }
 return null;
}
// Positions rotate by the full angle; quaternions compose on the half angle.
const yawQuatWXYZ=(q,yaw)=>{
 const hc=Math.cos(yaw/2),hs=Math.sin(yaw/2),[w,x,y,z]=q;
 return [hc*w-hs*z,hc*x-hs*y,hc*y+hs*x,hc*z+hs*w];
};
const goalOwners=()=>[sceneMetadata,...(sceneMetadata.stages??[])];
// The fixture is a static body, so it moves through the model; the goals it defines
// have to ride the identical transform or the policy aims at the old location.
function applyFixture(plan){
 if(!layout?.fixture)return;
 const {basePos,baseQuat,body}=layout.fixture;
 const yaw=plan?.fixture?plan.fixtureYaw:0,c=Math.cos(yaw),sn=Math.sin(yaw);
 const hc=Math.cos(yaw/2),hs=Math.sin(yaw/2);
 const x=plan?.fixture?plan.fixture.x:basePos[0],y=plan?.fixture?plan.fixture.y:basePos[1];
 model.body_pos.set([x,y,basePos[2]],3*body);
 model.body_quat.set(yawQuatWXYZ(baseQuat,yaw),4*body);
 const shiftX=x-basePos[0],shiftY=y-basePos[1];
 goalOwners().forEach((owner,i)=>{
  const base=baseGoals[i];if(!base)return;
  owner.goals=base.map(goal=>{
   const dx=goal[0]-basePos[0],dy=goal[1]-basePos[1],[gx,gy,gz,gw]=goal.slice(3);
   return [basePos[0]+c*dx-sn*dy+shiftX,basePos[1]+sn*dx+c*dy+shiftY,goal[2],
    hc*gx-hs*gy,hc*gy+hs*gx,hc*gz+hs*gw,hc*gw-hs*gz];
  });
 });
}
// Tip each part onto a candidate face, drop it clear of the table and let physics
// settle it. Whatever it comes to rest in is a pose the part can genuinely hold,
// so no shape-specific knowledge is needed. Returns false if anything left the table.
function dropParts(plan,robotQpos){
 layout.parts.forEach((part,i)=>{
  const spot=plan.parts[i];if(!spot)return;
  const address=part.address;
  data.qpos[address]=spot.x;data.qpos[address+1]=spot.y;
  data.qpos[address+2]=layout.table.top+part.sphere+.002;
  data.qpos.set(plan.partQuats[i],address+3);
 });
 data.qvel.fill(0);
 const steps=Math.round(RESET_SETTLE_SECONDS/model.opt.timestep);
 for(let i=0;i<steps;i++)mj.mj_step(model,data);
 // Still moving means it never settled: rocking or rolling, not a resting pose.
 const atRest=layout.parts.every(part=>Math.max(...Array.from(data.qvel.slice(part.dof,part.dof+6),Math.abs))<RESET_REST_SPEED);
 data.qvel.fill(0); // Every attempt begins at rest, never mid-bounce.
 // Validate the exact state Reset reproduces: the robot returns to its start pose,
 // which is not where 0.9 s of settling left it.
 data.qpos.set(robotQpos,0);
 mj.mj_forward(model,data);
 const onTable=atRest&&layout.parts.every(part=>{
  const [x,y,z]=data.qpos.slice(part.address,part.address+3);
  return z>layout.table.top-.02&&x>=layout.table.minX&&x<=layout.table.maxX&&y>=layout.table.minY&&y<=layout.table.maxY;
 });
 return onTable&&isolated()&&meshesApart();
}
// Render-only geoms (contype 0, such as the fixture's copies) never enter the
// contact list, so a part can visibly sit inside one with nothing to detect it.
// Those pairs get a bounding-sphere test instead; colliding pairs use contacts.
function meshesApart(){
 const watched=new Set([...layout.parts.map(part=>part.body),...(layout.fixture?[layout.fixture.body]:[])]);
 const geoms=[];
 for(let g=0;g<model.ngeom;g++){
  const body=model.geom_bodyid[g];
  if(!watched.has(body)||model.geom_group[g]>=3)continue;
  geoms.push({g,body,ghost:model.geom_contype[g]===0&&model.geom_conaffinity[g]===0});
 }
 for(let i=0;i<geoms.length;i++)for(let j=i+1;j<geoms.length;j++){
  const a=geoms[i],b=geoms[j];
  if(a.body===b.body||(!a.ghost&&!b.ghost))continue;
  const gap=Math.hypot(...[0,1,2].map(k=>data.geom_xpos[3*a.g+k]-data.geom_xpos[3*b.g+k]));
  if(gap<model.geom_rbound[a.g]+model.geom_rbound[b.g])return false;
 }
 return true;
}
// An arrangement is only usable if every part rests on the table alone: touching
// the fixture, the robot or another part means the attempt starts inside something.
function isolated(){
 const watched=new Set([...layout.parts.map(part=>part.body),...(layout.fixture?[layout.fixture.body]:[])]);
 let clean=true;
 for(let j=0;j<data.ncon;j++){
  const contact=data.contact.get(j),bodies=Array.from(contact.geom,g=>model.geom_bodyid[g]);
  if(bodies.some(body=>watched.has(body))){
   const others=bodies.filter(body=>!watched.has(body));
   if(!others.length||others.some(body=>body!==layout.table.body))clean=false;
  }
  if(contact.dist<-RESET_PENETRATION_M)clean=false;
  contact.delete();
 }
 return clean;
}
// Starting arrangements shipped with the task. When the file is absent the worker
// generates its own at load, which costs a settling pass.
async function loadResetPool(assets){
 if(!layout||fixedStart!==undefined)return false; // Fixed starts stay bit-exact.
 let file;
 try{
  const response=await fetch(assets+'resets.json');
  if(!response.ok)return false;
  file=await response.json();
 }catch{return false;}
 if(file?.version!==1||!Array.isArray(file.arrangements))return false;
 const owners=goalOwners().length;
 const usable=file.arrangements.filter(entry=>
  Array.isArray(entry?.parts)&&entry.parts.length===layout.parts.length
  &&entry.parts.every(part=>Array.isArray(part)&&part.length===7&&part.every(Number.isFinite))
  &&Array.isArray(entry.goals)&&entry.goals.length===owners
  &&(!layout.fixture||(Array.isArray(entry.fixture?.pos)&&Array.isArray(entry.fixture?.quat))));
 if(!usable.length)return false;
 resetSource='file';
 resetPool=usable.map(entry=>({
  fixture:entry.fixture?{pos:entry.fixture.pos,quat:entry.fixture.quat}:null,
  goals:entry.goals,parts:entry.parts}));
 return true;
}
// Settling costs real time, so it runs once at load. Reset then picks a finished
// arrangement and drops the scene into it.
function buildResetPool(){
 resetPool=[];
 if(!layout||fixedStart!==undefined)return; // Fixed starts stay bit-exact.
 const body=layout.fixture?.body;
 for(let attempt=0;attempt<RESET_ATTEMPTS&&resetPool.length<RESET_POOL;attempt++){
  const plan=planLayout();
  if(!plan)continue;
  applyFixture(plan);
  control.reset(0);
  const robotQpos=data.qpos.slice(0,Math.min(...layout.parts.map(part=>part.address)));
  if(!dropParts(plan,robotQpos))continue;
  resetPool.push({
   fixture:body===undefined?null:{pos:Array.from(model.body_pos.slice(3*body,3*body+3)),quat:Array.from(model.body_quat.slice(4*body,4*body+4))},
   goals:goalOwners().map(owner=>owner.goals?.map(goal=>goal.slice())),
   parts:layout.parts.map(part=>Array.from(data.qpos.slice(part.address,part.address+7)))});
 }
 applyFixture(null);control.reset(0); // Leave the canonical arrangement loaded.
}
// Reset order matters: goals must be in place before control.reset() reads them,
// and part poses must be written after it has restored the canonical state.
function applyResetLayout(entry,index){
 if(entry){
  if(entry.fixture&&layout.fixture!==null){
   model.body_pos.set(entry.fixture.pos,3*layout.fixture.body);
   model.body_quat.set(entry.fixture.quat,4*layout.fixture.body);
  }
  goalOwners().forEach((owner,i)=>{if(entry.goals[i])owner.goals=entry.goals[i].map(goal=>goal.slice());});
 }else applyFixture(null);
 control.reset(index);
 if(entry){
  layout.parts.forEach((part,i)=>data.qpos.set(entry.parts[i],part.address));
  data.qvel.fill(0);
  mj.mj_forward(model,data);
 }
}
function schedule(){
 clearTimeout(timer);const version=++scheduleVersion;
 if(!ready||busy||(paused&&!single))return;
 const delay=deadline-performance.now();
 // Yield to commands without the nested setTimeout(0) minimum delay when
 // simulation is already behind schedule. Positive waits still use a timer.
 if(delay<=0)wake.port2.postMessage(version);
 else timer=setTimeout(()=>{if(version===scheduleVersion)tick();},delay);
}
async function tick(){
 if(busy||!ready||(paused&&!single))return;
 busy=true;single=false;const version=epoch,stage=control.stageIndex??0;
 try{
  operation='Running movement policy';
  const result=await sessions[stage].run({obs:new ort.Tensor('float32',control.observation(),[1,141]),h_in:h,c_in:c});
  if(version!==epoch)return; // Reset invalidates any inference already in flight.
  h=result.h_out;c=result.c_out;operation='Stepping simulation';
  const started=performance.now();control.step(result.mu.data);
  stepMs=stepMs?stepMs*.9+(performance.now()-started)*.1:performance.now()-started;
  if(disturbRemaining>0&&--disturbRemaining===0)clearDisturbance();
  if((control.stageIndex??0)!==stage)resetRnn();
  if(control.succeeded&&!control.failed){
   // Finish withdrawing the hand with the same actor, RNN and physical state.
   // The controller's success is latched; validate the actual final pose too.
   releaseRemaining=releaseRemaining===null?releaseControls:Math.max(0,releaseRemaining-1);
   if(releaseRemaining===0){
    completed=control.distance<=control.successTolerance&&control.fingertipDistances.reduce((a,b)=>a+b,0)/control.fingertipDistances.length>.1;
    if(!completed)control.failed=true;
   }
  }
  if(completed||control.failed)paused=true;
  deadline=Math.max(deadline+1000/60,performance.now()-1000/60);
  emit();
 }catch(error){paused=true;postMessage({type:'error',error:describeError(error,operation)});}
 finally{busy=false;schedule();}
}
// MuJoCo has no runtime flag that turns a free body static: mocap bodies and weld
// equalities are both fixed when the model compiles. Inject one inactive weld per
// part, each anchored to its own mocap body, so a seated part can be welded to the
// world later. An identity relpose holds body1 coincident with body2.
function addFreezeWelds(xml,parts){
 if(!xml.includes('</worldbody>')||!xml.includes('</mujoco>'))throw Error('Unexpected scene layout');
 const anchors=parts.map((_,i)=>`<body name="freeze_anchor_${i}" mocap="true" pos="0 0 0"/>`).join('');
 const welds=parts.map((part,i)=>`<weld name="freeze_${i}" body1="${part}" body2="freeze_anchor_${i}" active="false" relpose="0 0 0 1 0 0 0" anchor="0 0 0" solref="0.005 1" solimp="0.9995 0.9999 0.0001"/>`).join('');
 return xml.replace('</worldbody>',anchors+'</worldbody>').replace('</mujoco>',`<equality>${welds}</equality></mujoco>`);
}
async function initialize({task,assets,mpr,initialPositionOffset,start,disturb,spread,sdf,substep}){
 let sdfPoints,substeps=1;
 if(sdf!==undefined){
  // MuJoCo aborts with 'mjc_SDF: too many contact points' well before 80.
  if(!Number.isInteger(sdf)||sdf<4||sdf>40)throw Error('Invalid SDF sample count');
  sdfPoints=sdf;
 }
 if(substep!==undefined){
  if(!Number.isInteger(substep)||substep<1||substep>8)throw Error('Invalid substep factor');
  substeps=substep;
 }
 if(spread!==undefined){
  if(!Number.isFinite(spread)||spread<0||spread>5)throw Error('Invalid reset spread');
  spreadScale=spread;
 }
 if(disturb!==undefined){
  if(!Number.isFinite(disturb)||disturb<=0||disturb>20)throw Error('Invalid disturbance scale');
  disturbScale=disturb;
 }
 const loading=label=>{operation=label;postMessage({type:'loading',label});};
 loading('Loading task…');
 const response=await fetch(assets+'manifest.json');if(!response.ok)throw Error('Missing scene manifest');const metadata=await response.json();
 sceneMetadata=metadata;
 fixedStart=start;
 if(fixedStart!==undefined&&(!Number.isInteger(fixedStart)||fixedStart<0||fixedStart>=(metadata.initialization_pool?.length??1)))throw Error('Invalid fixed start');
 releaseControls=metadata.release_follow_through_controls??0;
 if(!Number.isInteger(releaseControls)||releaseControls<0||releaseControls>120)throw Error('Invalid release duration');
 if(initialPositionOffset!==undefined){
  if(!Array.isArray(initialPositionOffset)||initialPositionOffset.length!==3||initialPositionOffset.some(v=>!Number.isFinite(v)||Math.abs(v)>.02))throw Error('Invalid diagnostic initial offset');
  delete metadata.initialization_pool; // Explicit diagnostic offsets bypass saved starts.
  metadata.initial_qpos=metadata.initial_qpos.slice();
  const address=metadata.object_qpos_address??29;
  initialPositionOffset.forEach((v,i)=>metadata.initial_qpos[address+i]+=v);
 }
 loading('Preparing simulation…');
 mj=await loadMuJoCo();mj.FS.mkdir('/scene');
 const files=['scene.xml',...Object.keys(metadata.files)];let loaded=0;
 loading(`Loading robot and parts… 0 / ${files.length}`);
 const freezeParts=metadata.stages?.map((_,i)=>['object','second'][i]).filter(Boolean)??[];
 await Promise.all(files.map(async name=>{const r=await fetch(new URL(metadata.asset_sources?.[name]??name,assets));if(!r.ok)throw Error(`Asset ${name}: ${r.status}`);
  const bytes=new Uint8Array(await r.arrayBuffer());
  const patched=name==='scene.xml'&&freezeParts.length?new TextEncoder().encode(addFreezeWelds(new TextDecoder().decode(bytes),freezeParts)):bytes;
  mj.FS.writeFile('/scene/'+name,patched);loading(`Loading robot and parts… ${++loaded} / ${files.length}`);}));
 loading('Preparing physics…');
 model=mj.MjModel.from_xml_path('/scene/scene.xml');
 // Compiled model owns its geometry; release redundant source-file buffers.
 for(const name of files)mj.FS.unlink('/scene/'+name);
 data=new mj.MjData(model);if(mpr)model.opt.disableflags|=131072; // NATIVECCD off
 const tuning=SDF_TUNING[task];
 if(sdfPoints!==undefined||tuning){
  model.opt.sdf_initpoints=sdfPoints??tuning.initpoints;
  if(tuning?.iterations)model.opt.sdf_iterations=tuning.iterations;
 }
 // Finer integration without changing the control interval the policy was trained
 // on: the timestep shrinks and the decimation grows by the same factor.
 if(substeps>1){model.opt.timestep/=substeps;metadata.decimation*=substeps;}
 control=new(task==='fabrica'?FabricaControl:AssemblyControl)(mj,model,data,metadata);
 // Tasks with saved starts randomize by picking one; the rest jitter this pose.
 mj.mj_forward(model,data);prepareLayout();
 if(!await loadResetPool(assets)){loading('Preparing reset arrangements…');buildResetPool();
  resetSource=resetPool.length?'generated':'none';}
 if(fixedStart!==undefined)control.reset(fixedStart);
 ort.env.wasm.numThreads=1;ort.env.wasm.wasmPaths={mjs:new URL('./vendor/ort-wasm-simd-threaded.mjs?v=memory1',import.meta.url).href,wasm:new URL('./vendor/ort-wasm-simd-threaded.wasm',import.meta.url).href};
 const policies=metadata.stages?.map(s=>s.policy)??['policy.onnx'];
 sessions=[];for(const policy of policies){loading(`Loading policy… ${sessions.length+1} / ${policies.length}`);sessions.push(await ort.InferenceSession.create(assets+policy,{executionProviders:['wasm']}));}
 resetRnn();ready=true;
 const geometry={ngeom:model.ngeom,goalBodyId:mj.mj_name2id(model,1,'goal_object'),tableBodyId:mj.mj_name2id(model,1,'table'),fixtureBodyId:mj.mj_name2id(model,1,'assembly_fixture'),partIds:['object','second'].map(n=>mj.mj_name2id(model,1,n))};
 for(const field of ['geom_group','geom_type','geom_size','geom_dataid','geom_rgba','geom_bodyid','mesh_vertadr','mesh_vertnum','mesh_faceadr','mesh_facenum','mesh_vert','mesh_face'])geometry[field]=model[field].slice();
 // MuJoCo uses material RGBA when a geom keeps the default gray RGBA.
 // Resolve into the render-only copy; never change model collision/state data.
 for(let i=0;i<model.ngeom;i++){
  const mat=model.geom_matid[i],offset=4*i;
  if(mat>=0&&geometry.geom_rgba[offset]===.5&&geometry.geom_rgba[offset+1]===.5&&geometry.geom_rgba[offset+2]===.5&&geometry.geom_rgba[offset+3]===1){
   geometry.geom_rgba.set(model.mat_rgba.slice(4*mat,4*mat+4),offset);
  }
 }
 postMessage({type:'ready',metadata,model:geometry,resetArrangements:resetPool.length,resetSource,frame:snapshot()});
}
async function settle(seconds,disableRobotContacts=false){
 if(!paused||busy||single||!completed)throw Error('Settling requires a completed, paused episode');
 if(!Number.isFinite(seconds)||seconds<=0||seconds>10)throw Error('Invalid settling duration');
 busy=true;const version=epoch,targets=data.ctrl.slice(),initialQpos=data.qpos.slice();
 const addresses=control.sequenceMetadata?.stages.map(s=>s.object_qpos_address)??[29];
 const peak=addresses.map(()=>0),contacts=new Map(),contactDetails=new Map();let maxError=0;
 const terminalContacts=[];
 const describeContact=contact=>{
  const ids=Array.from(contact.geom);
  return {geom_ids:ids,geoms:ids.map(g=>mj.mj_id2name(model,5,g)||`geom_${g}`),bodies:ids.map(g=>mj.mj_id2name(model,1,model.geom_bodyid[g])),distance_m:contact.dist};
 };
 for(let j=0;j<data.ncon;j++){const contact=data.contact.get(j);
  if(Array.from(contact.geom).some(g=>model.geom_contype[g]===2))terminalContacts.push(describeContact(contact));
  contact.delete();
 }
 const originalMasks=[];
 if(disableRobotContacts){for(let g=0;g<model.ngeom;g++){if(model.geom_contype[g]===2){originalMasks.push([g,model.geom_contype[g],model.geom_conaffinity[g]]);model.geom_contype[g]=0;model.geom_conaffinity[g]=0;}}}
 const count=Math.round(seconds/model.opt.timestep);
 try{
  for(let i=0;i<count;i++){
   if(version!==epoch)return;
   mj.mj_step(model,data);control.afterSubstep(); // Settling must hold a seated part too.
   for(let j=0;j<addresses.length;j++){const a=addresses[j];peak[j]=Math.max(peak[j],Math.hypot(...[0,1,2].map(k=>data.qpos[a+k]-initialQpos[a+k])));}
   for(let j=0;j<data.ncon;j++){
    const contact=data.contact.get(j);const geoms=Array.from(contact.geom);
    if(geoms.some(g=>model.geom_contype[g]===2)){
     const detail=describeContact(contact),key=JSON.stringify(detail.geoms);contacts.set(key,(contacts.get(key)??0)+1);
     const time=i*model.opt.timestep; // mj_step contacts are evaluated before integration.
     const summary=contactDetails.get(key)??{geom_ids:detail.geom_ids,geoms:detail.geoms,bodies:detail.bodies,first_time_s:time,last_time_s:time,min_distance_m:detail.distance_m};
     summary.last_time_s=time;summary.min_distance_m=Math.min(summary.min_distance_m,detail.distance_m);contactDetails.set(key,summary);
    }
    contact.delete();
   }
   const g=control.geometry(control.metadata.reward_offsets??control.metadata.keypoint_offsets);
   control.distance=Math.max(...g.objectKeys.map((p,j)=>Math.hypot(...p.map((v,k)=>v-g.goalKeys[j][k]))));
   maxError=Math.max(maxError,control.distance);
   if(i%control.metadata.decimation===0){mj.mj_forward(model,data);emit();await new Promise(resolve=>setTimeout(resolve,0));}
  }
  if(version!==epoch)return;
  if(Array.from(data.ctrl).some((v,i)=>v!==targets[i]))throw Error('Held targets changed');
  mj.mj_forward(model,data);emit();
  postMessage({type:'settled',report:{seconds:count*model.opt.timestep,diagnostic_robot_contacts_disabled:disableRobotContacts,peak_translation_m:peak,final_pose_error_m:control.distance,max_pose_error_m:maxError,
   terminal_robot_contacts:terminalContacts,robot_contact_details:Array.from(contactDetails.values()),
   robot_contact_pairs:Array.from(contacts,([key,samples])=>({geoms:JSON.parse(key),samples})),initial_qpos:Array.from(initialQpos),final_qpos:Array.from(data.qpos),
   scope:'Physics continues after terminal success with held actuator targets and no policy inference; success remains latched while pose error is remeasured.'}});
 }finally{for(const [g,type,affinity] of originalMasks){model.geom_contype[g]=type;model.geom_conaffinity[g]=affinity;}busy=false;schedule();}
}
onmessage=async({data:message})=>{
 try{
  if(message.type==='init'){await initialize(message);return;}
  if(!ready)return;
  if(message.type==='settle'){await settle(message.seconds,message.disableRobotContacts===true);return;}
  if(message.type==='reset'){epoch++;paused=true;single=false;releaseRemaining=null;completed=false;clearDisturbance();const count=control.sequenceMetadata?.initialization_pool?.length??1;
   const index=fixedStart??(count>1?((control.startIndex??0)+1+Math.floor(Math.random()*(count-1)))%count:0);
   resetIndex=resetPool.length?(Number.isInteger(message.arrangement)
    ?((message.arrangement%resetPool.length)+resetPool.length)%resetPool.length
    :Math.floor(Math.random()*resetPool.length)):-1;
   applyResetLayout(resetIndex>=0?resetPool[resetIndex]:null,index);
   control.distance=undefined;resetRnn();emit();}
  else if(message.type==='disturb'){if(!paused&&!completed&&!control.failed)applyDisturbance();}
  else if(message.type==='play'){if(!completed&&!control.failed){paused=message.paused;deadline=performance.now();emit();}}
  else if(message.type==='step'){if(!completed&&!control.failed){paused=true;single=true;deadline=performance.now();}}
  schedule();
 }catch(error){postMessage({type:'error',error:describeError(error,operation)});}
};
