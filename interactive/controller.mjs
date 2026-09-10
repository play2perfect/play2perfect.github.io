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
const qmul=(a,b)=>[a[3]*b[0]+a[0]*b[3]+a[1]*b[2]-a[2]*b[1],
 a[3]*b[1]-a[0]*b[2]+a[1]*b[3]+a[2]*b[0],
 a[3]*b[2]+a[0]*b[1]-a[1]*b[0]+a[2]*b[3],
 a[3]*b[3]-a[0]*b[0]-a[1]*b[1]-a[2]*b[2]];
const qconj=q=>[-q[0],-q[1],-q[2],q[3]];
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
 // The same tolerance that declares a goal reached must decide final success;
 // a stricter final check turns a legitimate insertion into a failed attempt.
 get successTolerance(){return this.metadata.insertion_tolerance??.0075;}
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
 afterSubstep(){}
 step(action){
  this.targets=actionTargets(this.targets,action,this.model.jnt_range);this.data.ctrl.set(this.targets);
  for(let i=0;i<this.metadata.decimation;i++){this.mj.mj_step(this.model,this.data);this.afterSubstep();}
  this.mj.mj_forward(this.model,this.data);
  const g=this.geometry(this.metadata.reward_offsets??this.metadata.keypoint_offsets),distance=Math.max(...g.objectKeys.map((p,i)=>norm(sub(p,g.goalKeys[i]))));
  const tips=this.tips.map(id=>norm(sub(this.bodyPosition(id),g.position)));
  this.fingertipDistances=tips;
  this.episode++;this.steps++;this.near=distance<=(this.metadata.insertion_tolerance??.015)?this.near+1:0;
  if(this.near>=(this.metadata.success_steps??10)&&!this.retract){this.successes++;this.episode=0;
   if(this.successes===this.metadata.goals.length)this.retract=true;else this.near=0;
  }
  this.succeeded ||= this.retract&&tips.reduce((a,b)=>a+b,0)/tips.length>.1&&distance<=this.successTolerance;
  this.failed ||= g.position[2]<.1||(!this.retract&&Math.max(...tips)>1.5)||this.episode>=601;
  this.distance=distance;this.setGoal();
 }
}

// Fictitious inertia that makes a seated part behave as a kinematic body: the
// solver sees six near-immovable DOFs, so robot contacts push the hand away
// instead of the part. Armature is inertia only; gravity loading is unchanged.
const FROZEN_ARMATURE=1e3;
// The first part is what the second one seats against, so it is held to the
// tighter tolerance even though the sequence as a whole allows 15 mm.
const FIRST_STAGE_TOLERANCE=.0075;
// The runner owns recurrent inference state and must clear it when stageIndex
// changes. This controller never resets physical state during a policy handoff.
export class FabricaControl extends AssemblyControl {
 reset(startIndex=0){
  this.sequenceMetadata??=this.metadata;
  this.releaseFreeze();
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
 get successTolerance(){return this.stageIndex===0?FIRST_STAGE_TOLERANCE:super.successTolerance;}
 // A seated part is held rigid: the second policy may lean on it, but must not
 // knock it back out. Held at physics rate so it never visibly drifts.
 joint(qposAddress){
  for(let j=0;j<this.model.njnt;j++)if(this.model.jnt_qposadr[j]===qposAddress)return j;
  throw Error('No free joint at the object address');
 }
 dofAddress(qposAddress){return this.model.jnt_dofadr[this.joint(qposAddress)];}
 // Welds are injected one per stage, in stage order; verify before trusting that.
 welds(){
  if(this._welds!==undefined)return this._welds;
  const stages=this.sequenceMetadata.stages;
  this._welds=null;
  if(this.model.neq===stages.length){
   const found=stages.map((stage,i)=>{
    const anchor=this.mj.mj_name2id(this.model,1,`freeze_anchor_${i}`);
    const body=this.model.jnt_bodyid[this.joint(stage.object_qpos_address)];
    return anchor>=0&&this.model.eq_obj1id[i]===body?{eq:i,mocap:this.model.body_mocapid[anchor],body}:null;
   });
   if(found.every(Boolean))this._welds=found;
  }
  return this._welds;
 }
 afterSubstep(){
  const frozen=this.frozen;
  if(!frozen||frozen.weld)return; // The weld constraint needs no help.
  this.data.qpos.set(frozen.qpos,frozen.address);
  for(let i=0;i<6;i++)this.data.qvel[frozen.dof+i]=0;
 }
 freezeStage(index){
  const weld=this.welds()?.[index];
  if(weld){
   // Park the anchor on the part's current pose, then switch the weld on.
   this.data.mocap_pos.set(this.data.xpos.slice(3*weld.body,3*weld.body+3),3*weld.mocap);
   this.data.mocap_quat.set(this.data.xquat.slice(4*weld.body,4*weld.body+4),4*weld.mocap);
   this.data.eq_active[weld.eq]=1;this.frozen={weld};
   return;
  }
  // Fallback when the scene carries no welds: fictitious inertia plus a pose latch.
  const address=this.sequenceMetadata.stages[index].object_qpos_address,dof=this.dofAddress(address);
  const armature=Array.from(this.model.dof_armature.slice(dof,dof+6));
  for(let i=0;i<6;i++)this.model.dof_armature[dof+i]=FROZEN_ARMATURE;
  this.frozen={address,dof,armature,qpos:Array.from(this.data.qpos.slice(address,address+7))};
 }
 releaseFreeze(){
  if(!this.frozen)return;
  if(this.frozen.weld)this.data.eq_active[this.frozen.weld.eq]=0;
  else for(let i=0;i<6;i++)this.model.dof_armature[this.frozen.dof+i]=this.frozen.armature[i];
  this.frozen=null;
 }
 selectStage(index){
  this.stageIndex=index;
  this.metadata={...this.sequenceMetadata,...this.sequenceMetadata.stages[index]};
  this.successes=0;this.near=0;this.episode=0;this.retract=false;this.succeeded=false;this.failed=false;
  this.setGoal();
 }
 // The second part mates with the first, so its goals must follow wherever the
 // first actually seated. Its stored goals are absolute world poses, so a first
 // part resting anywhere inside its tolerance leaves the second aiming off the
 // real mating surface -- and a seated part is now rigid, so it cannot be nudged
 // back into line. Identity when the first part seats exactly on its goal.
 compensatedGoals(){
  const stage=this.sequenceMetadata.stages[1];
  if(!stage?.goals)return null;
  const address=this.metadata.object_qpos_address??29,q=this.data.qpos;
  const achievedPosition=Array.from(q.slice(address,address+3));
  const achieved=[q[address+4],q[address+5],q[address+6],q[address+3]];
  const target=this.metadata.goals[this.metadata.goals.length-1];
  const delta=qmul(achieved,qconj(target.slice(3,7)));
  const offset=sub(achievedPosition,rotate(delta,target.slice(0,3)));
  return stage.goals.map(goal=>[...add(rotate(delta,goal.slice(0,3)),offset),...qmul(delta,goal.slice(3,7))]);
 }
 step(action){
  super.step(action);
  if(this.succeeded&&this.stageIndex===0){
   const before=[Array.from(this.data.qpos),Array.from(this.data.qvel),Array.from(this.data.ctrl),Array.from(this.targets)];
   const compensated=this.compensatedGoals();
   this.selectStage(1);
   // Held on the merged metadata only, so the stored goals stay pristine for the
   // next attempt no matter how this one lands.
   if(compensated){this.metadata={...this.metadata,goals:compensated};this.setGoal();}
   this.handoffs.push(this.steps);
   const after=[this.data.qpos,this.data.qvel,this.data.ctrl,this.targets];
   if(before.some((a,i)=>a.some((v,j)=>v!==after[i][j])))throw Error('Physical Fabrica state changed during handoff');
   this.freezeStage(0); // Latches the pose the handoff check just verified unchanged.
  }
 }
}
