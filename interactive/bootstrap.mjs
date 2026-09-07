// Static loading panel is visible before any renderer/worker dependency loads.
const progress=document.getElementById('loading-progress');
const status=document.getElementById('status');
new MutationObserver(()=>{progress.textContent=status.textContent;}).observe(status,{childList:true,characterData:true,subtree:true});
document.getElementById('loading-retry').onclick=()=>location.reload();
// Catch failures before main.mjs can install its worker error handler, including
// unsupported/lost graphics contexts and missing module dependencies.
import('./main.mjs').catch(error=>{
 document.getElementById('status').textContent='Unable to start the demo';
 document.getElementById('error').textContent='The demo could not initialize. Try reloading or opening it in another browser.';
 const retry=document.getElementById('retry');retry.hidden=false;retry.onclick=()=>location.reload();
 document.getElementById('loading-note').textContent='The demo could not initialize. Try reloading or opening it in another browser.';
 document.getElementById('loading-retry').hidden=false;
 console.error(error);
});
