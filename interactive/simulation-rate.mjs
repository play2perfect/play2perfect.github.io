// Observed simulated seconds per wall second, independent of render FPS.
export class SimulationRate {
 constructor(){this.reset();}
 reset(){this.start=null;this.value=null;}
 sample(simTime,wallMs,paused,epoch){
  if(paused){this.reset();return null;}
  if(!this.start||this.start.epoch!==epoch||simTime<this.start.simTime){
   this.start={simTime,wallMs,epoch};this.value=null;return null;
  }
  const elapsed=wallMs-this.start.wallMs;
  if(elapsed>=1000){
   this.value=(simTime-this.start.simTime)*1000/elapsed;
   this.start={simTime,wallMs,epoch};
  }
  return this.value;
 }
}
