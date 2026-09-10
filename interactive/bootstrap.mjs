import {describeError} from './error-details.mjs?v=9';
// Static loading panel is visible before any renderer/worker dependency loads.
const progress=document.getElementById('loading-progress');
const status=document.getElementById('status');
new MutationObserver(()=>{progress.textContent=status.textContent;}).observe(status,{childList:true,characterData:true,subtree:true});
document.getElementById('loading-retry').onclick=()=>location.reload();
// Catch failures before main.mjs can install its worker error handler, including
// unsupported/lost graphics contexts and missing module dependencies.
import('./main.mjs?v=9').catch(error=>{
 document.getElementById('status').textContent='Unable to start the demo';
 document.getElementById('error').textContent='The demo could not initialize. Try reloading or opening it in another browser.';
 const retry=document.getElementById('retry');retry.hidden=false;retry.onclick=()=>location.reload();
 document.getElementById('loading-note').textContent=describeError(error,'Loading demo code');
 document.getElementById('loading-retry').hidden=false;
 console.error(error);
});

const copyError=document.getElementById('copy-error');
const loadingRetry=document.getElementById('loading-retry');
new MutationObserver(()=>{copyError.hidden=loadingRetry.hidden;}).observe(loadingRetry,{attributes:true,attributeFilter:['hidden']});
copyError.onclick=async()=>{
 const note=document.getElementById('loading-note');
 const details=`${location.href}\n${navigator.userAgent}\n${note.textContent}`;
 try{await navigator.clipboard.writeText(details);copyError.textContent='Copied';}
 catch{const range=document.createRange();range.selectNodeContents(note);const selection=getSelection();selection.removeAllRanges();selection.addRange(range);copyError.textContent='Select and copy details';}
};
