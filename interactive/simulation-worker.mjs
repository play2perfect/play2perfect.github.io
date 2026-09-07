import {describeError} from './error-details.mjs';
import loadMuJoCo from './vendor/mujoco.js';
import * as ort from './vendor/ort.wasm.min.mjs';
import {AssemblyControl,FabricaControl} from './controller.mjs';
let mj,model,data,control,sessions,h,c,paused=true,single=false,busy=false,epoch=0,timer,deadline=0,ready=false,scheduleVersion=0;
let fixedStart;
let releaseControls=0,releaseRemaining=null,completed=false;
let operation='Starting demo';
const wake=new MessageChannel();
wake.port1.onmessage=({data:version})=>{if(version===scheduleVersion)tick();};
const empty=()=>new ort.Tensor('float32',new Float32Array(1024),[1,1,1024]);
function resetRnn(){h=empty();c=empty();}
function snapshot(){
 return {paused,control:{resetEpoch:epoch,startIndex:control.startIndex??0,steps:control.steps,stageIndex:control.stageIndex??0,handoffs:control.handoffs??[],successes:control.successes,
   succeeded:completed,failed:control.failed,retract:control.retract,distance:control.distance,goal:control.goal,targets:Array.from(control.targets)},
  data:{qpos:data.qpos.slice(),qvel:data.qvel.slice(),geom_xpos:data.geom_xpos.slice(),geom_xmat:data.geom_xmat.slice(),time:data.time},
  rnnMax:Math.max(...Array.from(h.data,Math.abs),...Array.from(c.data,Math.abs))};
}
function emit(){postMessage({type:'state',frame:snapshot()});}
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
  h=result.h_out;c=result.c_out;operation='Stepping simulation';control.step(result.mu.data);
  if((control.stageIndex??0)!==stage)resetRnn();
  if(control.succeeded&&!control.failed){
   // Finish withdrawing the hand with the same actor, RNN and physical state.
   // The controller's success is latched; validate the actual final pose too.
   releaseRemaining=releaseRemaining===null?releaseControls:Math.max(0,releaseRemaining-1);
   if(releaseRemaining===0){
    completed=control.distance<=.0075&&control.fingertipDistances.reduce((a,b)=>a+b,0)/control.fingertipDistances.length>.1;
    if(!completed)control.failed=true;
   }
  }
  if(completed||control.failed)paused=true;
  deadline=Math.max(deadline+1000/60,performance.now()-1000/60);
  emit();
 }catch(error){paused=true;postMessage({type:'error',error:describeError(error,operation)});}
 finally{busy=false;schedule();}
}
async function initialize({task,assets,mpr,initialPositionOffset,start}){
 const loading=label=>{operation=label;postMessage({type:'loading',label});};
 loading('Loading task…');
 const response=await fetch(assets+'manifest.json');if(!response.ok)throw Error('Missing scene manifest');const metadata=await response.json();
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
 await Promise.all(files.map(async name=>{const r=await fetch(new URL(metadata.asset_sources?.[name]??name,assets));if(!r.ok)throw Error(`Asset ${name}: ${r.status}`);mj.FS.writeFile('/scene/'+name,new Uint8Array(await r.arrayBuffer()));loading(`Loading robot and parts… ${++loaded} / ${files.length}`);}));
 loading('Preparing contacts…');
 model=mj.MjModel.from_xml_path('/scene/scene.xml');
 // Compiled model owns its geometry; release redundant source-file buffers.
 for(const name of files)mj.FS.unlink('/scene/'+name);
 data=new mj.MjData(model);if(mpr)model.opt.disableflags|=131072;
 control=new(task==='fabrica'?FabricaControl:AssemblyControl)(mj,model,data,metadata);
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
 postMessage({type:'ready',metadata,model:geometry,frame:snapshot()});
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
   mj.mj_step(model,data);
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
  if(message.type==='reset'){epoch++;paused=true;single=false;releaseRemaining=null;completed=false;const count=control.sequenceMetadata?.initialization_pool?.length??1;
   const index=fixedStart??(count>1?((control.startIndex??0)+1+Math.floor(Math.random()*(count-1)))%count:0);
   control.reset(index);control.distance=undefined;resetRnn();emit();}
  else if(message.type==='play'){if(!completed&&!control.failed){paused=message.paused;deadline=performance.now();emit();}}
  else if(message.type==='step'){if(!completed&&!control.failed){paused=true;single=true;deadline=performance.now();}}
  schedule();
 }catch(error){postMessage({type:'error',error:describeError(error,operation)});}
};
