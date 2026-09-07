// Optional rendering data only. MuJoCo's model and collision assets never change.
export async function loadVisualLod(url){
 const response=await fetch(url);if(!response.ok)throw Error('Unable to load faster-rendering visuals');
 const index=await response.json();if(index.version!==1||!index.meshes)throw Error('Invalid visual mesh index');
 const payload=await fetch(new URL(index.buffer,url));if(!payload.ok)throw Error('Unable to load faster-rendering geometry');
 const buffer=await payload.arrayBuffer(),meshes={};
 const read=(field,Type)=>{
  if(!Array.isArray(field)||field.length!==2)throw Error('Invalid visual attribute');
  const [offset,count]=field;
  if(!Number.isSafeInteger(offset)||!Number.isSafeInteger(count)||offset<0||count<0||offset%4||offset+4*count>buffer.byteLength)throw Error('Visual attribute outside buffer');
  return new Type(buffer,offset,count);
 };
 for(const [id,mesh] of Object.entries(index.meshes)){
  const positions=read(mesh.positions,Float32Array),normals=read(mesh.normals,Float32Array),indices=read(mesh.indices,Uint32Array);
  if(positions.length%3||normals.length!==positions.length||indices.length%3)throw Error('Invalid visual mesh shape');
  meshes[id]={positions,normals,indices};
 }
 return meshes;
}
