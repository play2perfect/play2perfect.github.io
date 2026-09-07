// Simulator-independent browser assembly control. Quaternions: xyzw in metadata.
const f=Math.fround;
const clip=(v,lo,hi)=>Math.min(hi,Math.max(lo,v));
const add=(a,b)=>a.map((v,i)=>v+b[i]);
const sub=(a,b)=>a.map((v,i)=>v-b[i]);
const norm=a=>Math.hypot(...a);
function rotate(q,p){
 const n=norm(q),[x,y,z,w]=q.map(v=>v/n),[a,b,c]=p;
 return [(1-2*(y*y+z*z))*a+2*(x*y-z*w)*b+2*(x*z+y*w)*c,
  2*(x*y+z*w)*a+(1-2*(x*x+z*z))*b+2*(y*z-x*w)*c,
  2*(x*z-y*w)*a+2*(y*z+x*w)*b+(1-2*(x*x+y*y))*c];
}
export function actionTargets(previous,action,ranges){
 return Float32Array.from(previous,(p,i)=>{
  const lo=f(ranges[2*i]),hi=f(ranges[2*i+1]),a=clip(action[i],-1,1);
  const raw=i<7?clip(f(p+f(f(.025)*a)),lo,hi):f(lo+f(f(f(.5)*f(a+1))*f(hi-lo)));
  return clip(f(f(f(.1)*raw)+f(f(.9)*p)),lo,hi);
 });
}
export class AssemblyControl {
 constructor(mj,model,data,metadata){
  Object.assign(this,{mj,model,data,metadata});
  this.palm=mj.mj_name2id(model,1,'link7');
  this.tips=['index','middle','ring','thumb','pinky'].map(v=>mj.mj_name2id(model,1,`palmleft_${v}_DP`));
  const body=mj.mj_name2id(model,1,'goal_object');this.goalId=model.body_mocapid[body];
  this.reset();
 }
 reset(){
  this.mj.mj_resetDataKeyframe(this.model,this.data,0);
  this.data.qpos.set(this.metadata.initial_qpos);this.data.ctrl.set(this.metadata.initial_ctrl);
  if(this.metadata.initial_qvel)this.data.qvel.set(this.metadata.initial_qvel);
  this.targets=Float32Array.from(this.metadata.initial_ctrl);
  this.successes=0;this.near=0;this.episode=0;this.retract=false;this.succeeded=false;this.failed=false;this.steps=0;
  this.setGoal();this.mj.mj_forward(this.model,this.data);
 }
 setGoal(){
  this.goal=this.metadata.goals[Math.min(this.successes,this.metadata.goals.length-1)];
  this.data.mocap_pos.set(this.goal.slice(0,3),3*this.goalId);
  this.data.mocap_quat.set([this.goal[6],...this.goal.slice(3,6)],4*this.goalId);
 }
 bodyPosition(id){return Array.from(this.data.xpos.slice(id*3,id*3+3));}
 geometry(offsets=this.metadata.keypoint_offsets,yaw=0){
  const d=this.data,a=this.metadata.object_qpos_address??29,position=Array.from(d.qpos.slice(a,a+3)),q=[d.qpos[a+4],d.qpos[a+5],d.qpos[a+6],d.qpos[a+3]];
  return {position,q,objectKeys:offsets.map(p=>add(rotate(q,p),position)),
   goalKeys:offsets.map(p=>add(rotate([0,0,Math.sin(yaw/2),Math.cos(yaw/2)],rotate(this.goal.slice(3),p)),this.goal.slice(0,3)))};
 }
 observation(){
  const d=this.data,m=this.model,values=[];
  for(let i=0;i<29;i++){
   const lo=f(m.jnt_range[2*i]),hi=f(m.jnt_range[2*i+1]);
   values.push(f(f(f(2*f(f(d.qpos[i])-lo))/f(hi-lo))-1));
  }
  values.push(...d.qvel.slice(0,29),...this.targets);
  const matrix=d.xmat.slice(9*this.palm,9*this.palm+9),offset=[0,-.02,.16];
  const palm=add(this.bodyPosition(this.palm),[0,1,2].map(i=>offset.reduce((s,v,j)=>s+matrix[3*i+j]*v,0)));
  const q=d.xquat.slice(this.palm*4,this.palm*4+4),g=this.geometry(this.metadata.keypoint_offsets,this.metadata.goal_yaw_bias??0);
  values.push(...palm,q[1],q[2],q[3],q[0],...g.q);
  for(const id of this.tips){
   const r=d.xmat.slice(9*id,9*id+9),p=[.02,.002,0];
   const tip=add(this.bodyPosition(id),[0,1,2].map(i=>p.reduce((s,v,j)=>s+r[3*i+j]*v,0)));
   values.push(...sub(tip,palm));
  }
  for(const p of g.objectKeys)values.push(...sub(p,palm));
  for(let i=0;i<4;i++)values.push(...sub(sub(g.objectKeys[i],g.goalKeys[i]),this.metadata.goal_position_bias));
  values.push(...this.metadata.object_scales);
  if(values.length!==140)throw Error('Observation shape');
  return Float32Array.from([...values.map(v=>clip(v,-10,10)),this.metadata.block_id]);
 }
 step(action){
  this.targets=actionTargets(this.targets,action,this.model.jnt_range);this.data.ctrl.set(this.targets);
  for(let i=0;i<this.metadata.decimation;i++)this.mj.mj_step(this.model,this.data);
  this.mj.mj_forward(this.model,this.data);
  const g=this.geometry(this.metadata.reward_offsets??this.metadata.keypoint_offsets),distance=Math.max(...g.objectKeys.map((p,i)=>norm(sub(p,g.goalKeys[i]))));
  const tips=this.tips.map(id=>norm(sub(this.bodyPosition(id),g.position)));
  this.fingertipDistances=tips;
  this.episode++;this.steps++;this.near=distance<=(this.metadata.insertion_tolerance??.015)?this.near+1:0;
  if(this.near>=(this.metadata.success_steps??10)&&!this.retract){this.successes++;this.episode=0;
   if(this.successes===this.metadata.goals.length)this.retract=true;else this.near=0;
  }
  this.succeeded ||= this.retract&&tips.reduce((a,b)=>a+b,0)/tips.length>.1&&distance<=.0075;
  this.failed ||= g.position[2]<.1||(!this.retract&&Math.max(...tips)>1.5)||this.episode>=601;
  this.distance=distance;this.setGoal();
 }
}

// The runner owns recurrent inference state and must clear it when stageIndex
// changes. This controller never resets physical state during a policy handoff.
export class FabricaControl extends AssemblyControl {
 reset(startIndex=0){
  this.sequenceMetadata??=this.metadata;
  this.stageIndex=0;this.handoffs=[];this.startIndex=startIndex;
  this.metadata={...this.sequenceMetadata,...this.sequenceMetadata.stages[0]};
  const starts=this.sequenceMetadata.initialization_pool;
  if(starts){
   const start=starts[startIndex];
   if(!start||start.part_qpos.length!==this.metadata.initial_qpos.length-29||start.part_qpos.some(v=>!Number.isFinite(v)))throw Error("Invalid Fabrica start");
   this.metadata.initial_qpos=this.metadata.initial_qpos.slice();
   this.metadata.initial_qpos.splice(29,start.part_qpos.length,...start.part_qpos);
  }
  super.reset();
 }
 selectStage(index){
  this.stageIndex=index;
  this.metadata={...this.sequenceMetadata,...this.sequenceMetadata.stages[index]};
  this.successes=0;this.near=0;this.episode=0;this.retract=false;this.succeeded=false;this.failed=false;
  this.setGoal();
 }
 step(action){
  super.step(action);
  if(this.succeeded&&this.stageIndex===0){
   const before=[Array.from(this.data.qpos),Array.from(this.data.qvel),Array.from(this.data.ctrl),Array.from(this.targets)];
   this.selectStage(1);this.handoffs.push(this.steps);
   const after=[this.data.qpos,this.data.qvel,this.data.ctrl,this.targets];
   if(before.some((a,i)=>a.some((v,j)=>v!==after[i][j])))throw Error('Physical Fabrica state changed during handoff');
  }
 }
}
