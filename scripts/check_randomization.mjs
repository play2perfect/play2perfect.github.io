// Offline test of the actual website WASM worker and exported policies. No simulator substitutions.
// node scripts/check_randomization.mjs <screwing|tight_insertion|fabrica> [rollouts|baseline|zero|race]
import fs from 'node:fs';
import {fileURLToPath,pathToFileURL} from 'node:url';
import assert from 'node:assert/strict';
const root=fileURLToPath(new URL('../interactive/',import.meta.url));
globalThis.fetch=async input=>{const p=String(input);const path=p.startsWith('file:')?fileURLToPath(p):p;return new Response(fs.readFileSync(path),{headers:{'Content-Type':path.endsWith('.wasm')?'application/wasm':'application/octet-stream'}});};
globalThis.onmessage=null;
let state,resolveStep;
globalThis.postMessage=m=>{if(m.type==='error')throw Error(m.error);if(m.frame){state=m.frame;if(resolveStep){const r=resolveStep;resolveStep=null;r();}}};
await import(pathToFileURL(root+'simulation-worker.mjs'));
const task=process.argv[2]??'screwing';
await onmessage({data:{type:'init',task,assets:pathToFileURL(root+'assets/'+task+'/').href,mpr:task!=='tight_insertion',start:0}});
console.log('ready',task);
async function command(data){await onmessage({data});}
async function step(){await new Promise(async r=>{resolveStep=r;await command({type:'step'});});}
const mode=process.argv[3]??'rollouts';
if(mode==='baseline'||mode==='zero'){
if(mode==='zero')await command({type:'randomize',offsets:Array.from({length:task==='fabrica'?2:1},()=>[0,0,0])});
const checkpoints=[];
for(let i=0;i<1600&&!state.control.succeeded&&!state.control.failed;i++){
 if([0,30,100].includes(i))checkpoints.push({steps:i,stage:state.control.stageIndex,parts:Array.from(state.data.qpos.slice(29))});
 await step();
}
console.log(JSON.stringify({task,success:state.control.succeeded,steps:state.control.steps,checkpoints}));
fs.writeFileSync('/tmp/p2p-randomize-'+mode+'-'+task+'.json',JSON.stringify({task,success:state.control.succeeded,steps:state.control.steps,checkpoints},null,2));
process.exit(0);

}
if(mode==='race'){
const initial=state;
const ort=await import(pathToFileURL(root+'vendor/ort.wasm.min.mjs'));
const run=ort.InferenceSession.prototype.run;
let entered,release;
const started=new Promise(r=>entered=r),gate=new Promise(r=>release=r);
ort.InferenceSession.prototype.run=async function(...args){const result=await run.apply(this,args);entered();await gate;return result;};
await command({type:'play',paused:false});await started;
await command({type:'randomize'});
assert.equal(state.paused,false);assert.equal(state.control.steps,0);assert.equal(state.rnnMax,0);
assert.deepEqual(Array.from(state.data.qpos.slice(0,29)),Array.from(initial.data.qpos.slice(0,29)));
assert.deepEqual(state.control.targets,initial.control.targets);
for(const a of (task==='fabrica'?[29,36]:[29])){
 for(const k of [0,1])assert(Math.abs(state.data.qpos[a+k]-initial.data.qpos[a+k])<=.01);
 assert.equal(state.data.qpos[a+2],initial.data.qpos[a+2]);
 const dot=state.data.qpos.slice(a+3,a+7).reduce((s,v,i)=>s+v*initial.data.qpos[a+3+i],0);
 assert(2*Math.acos(Math.min(1,Math.abs(dot)))<=Math.PI/18+1e-6);
}
await command({type:'play',paused:true});release();
await new Promise(r=>setTimeout(r,20));
assert.equal(state.control.steps,0,'Stale inference must not step after teleport');
ort.InferenceSession.prototype.run=run;
await step();assert.equal(state.control.steps,1);
console.log('PASS actual random command, bounds, robot/target preservation, running state, recurrent reset, in-flight inference invalidation:',task);
process.exit(0);

}
assert.equal(mode,'rollouts');
const records=[];
for(const when of [0,30,100])for(let seed=1;seed<=5;seed++){
 await command({type:'reset'});
 for(let i=0;i<when;i++)await step();
 const before=state;
 let rng=Math.imul(seed,0x9e3779b9)>>>0;const random=()=>{rng=(Math.imul(rng,1664525)+1013904223)>>>0;return rng/4294967296;};
 const offsets=Array.from({length:task==='fabrica'?2:1},()=>[(random()*2-1)*.01,(random()*2-1)*.01,(random()*2-1)*Math.PI/18]);
 await command({type:'randomize',offsets});
 assert.deepEqual(Array.from(state.data.qpos.slice(0,29)),Array.from(before.data.qpos.slice(0,29)));
 assert.deepEqual(state.control.targets,before.control.targets);assert.equal(state.rnnMax,0);
 assert.equal(state.control.stageIndex,0);assert.equal(state.control.steps,0);
 let n=0;while(!state.control.succeeded&&!state.control.failed&&n++<2600)await step();
 const row={task,when,seed,offsets,success:state.control.succeeded,failed:state.control.failed,steps:state.control.steps,distance:state.control.distance,preStage:before.control.stageIndex,preTime:before.data.time,preObject:Array.from(before.data.qpos.slice(29)),preRobot:Array.from(before.data.qpos.slice(0,29))};
 records.push(row);console.log(JSON.stringify({task,when,seed,success:row.success,steps:row.steps,distance:row.distance}));
 fs.writeFileSync('/tmp/p2p-randomize-wide-'+task+'.json',JSON.stringify(records,null,2));
}
process.exit(0);
