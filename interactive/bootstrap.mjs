// Catch failures before main.mjs can install its worker error handler, including
// unsupported/lost graphics contexts and missing module dependencies.
import('./main.mjs').catch(error=>{
 document.getElementById('status').textContent='Unable to start the demo';
 document.getElementById('error').textContent='The demo could not initialize. Try reloading or opening it in another browser.';
 const retry=document.getElementById('retry');retry.hidden=false;retry.onclick=()=>location.reload();
 console.error(error);
});
