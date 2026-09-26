// Node 22+: subscribe to the real main program and validate all six channels.
import { writeFileSync } from 'node:fs';
const host = process.argv[2] || '192.168.1.123';
const output = process.argv[3] || '/tmp/rescue-project-preview.jpg';
const status = await (await fetch(`http://${host}:8080/status`, {signal: AbortSignal.timeout(5000)})).json();
if (!status.health.robot_data_connected) throw new Error(JSON.stringify(status));
console.log('HTTP status:', JSON.stringify(status));
await new Promise((resolve,reject)=>{
 const ws = new WebSocket(`ws://${host}:8765`, 'foxglove.websocket.v1');
 ws.binaryType='arraybuffer';
 const counts = {}, fps=[], timings=[];
 let imageStamp=0n, bytes=0, started=Date.now(), lastSequence=-1;
 const timer=setTimeout(()=>{ws.close();reject(new Error('Timed out waiting for all project channels'));},15000);
 ws.onerror=()=>{clearTimeout(timer);reject(new Error('WebSocket failure'));};
 ws.onmessage=({data})=>{
  try {
   if(typeof data==='string'){
    const msg=JSON.parse(data);
    if(msg.op==='serverInfo' && msg.capabilities.length!==0) throw new Error('Unexpected writable capability');
    if(msg.op==='advertise'){
     const topics=msg.channels.map(c=>c.topic);
     for(const topic of ['/camera/image','/system/health','/detections','/fsm/state','/cmd/motion','/runtime/config'])
      if(!topics.includes(topic)) throw new Error('Missing '+topic);
     ws.send(JSON.stringify({op:'subscribe',subscriptions:msg.channels.map(c=>({id:c.id,channelId:c.id}))}));
     started=Date.now(); console.log('Topics:',topics.join(', '));
    }
    return;
   }
   const b=Buffer.from(data); bytes+=b.length;
   if(b[0]!==1) throw new Error('Invalid data opcode');
   const id=b.readUInt32LE(1), stamp=b.readBigUInt64LE(5), payload=JSON.parse(b.subarray(13));
   counts[id]=(counts[id]||0)+1;
   if(id===1){
    if(stamp<=imageStamp) throw new Error('Non-increasing capture timestamps');
    imageStamp=stamp;
    const jpeg=Buffer.from(payload.data,'base64');
    if(jpeg[0]!==255||jpeg[1]!==216||jpeg.at(-2)!==255||jpeg.at(-1)!==217) throw new Error('Invalid JPEG');
    writeFileSync(output,jpeg);
   } else if(id===2){
    if(!payload.robot_data_connected||!payload.camera_ok) throw new Error('Stale source');
    if(payload.sequence<lastSequence) throw new Error('Sequence moved backwards');
    lastSequence=payload.sequence;fps.push(payload.loop_fps);timings.push(payload.inference_ms);
   } else if(!payload.valid) throw new Error('Invalid structured data');
   if(id===5 && payload.hardware_output_enabled!==0) throw new Error('Unexpected hardware output');
   if(Object.keys(counts).length===6 && counts[1]>=20 && Date.now()-started>=5000){
    clearTimeout(timer);ws.close();
    console.log('Verification:',JSON.stringify({counts,seconds:(Date.now()-started)/1000,receivedKiB:bytes/1024,KiBPerSecond:bytes/1024/((Date.now()-started)/1000),averageLoopFps:fps.reduce((a,b)=>a+b,0)/fps.length,averageInferenceMs:timings.reduce((a,b)=>a+b,0)/timings.length,image:output}));
    resolve();
   }
  } catch(e){clearTimeout(timer);ws.close();reject(e);}
 };
});
await new Promise((resolve,reject)=>{
 const ws=new WebSocket(`ws://${host}:8765`,'foxglove.websocket.v1');
 const timer=setTimeout(()=>{ws.close();reject(new Error('Read-only protocol check timed out'));},5000);
 ws.onopen=()=>ws.send(JSON.stringify({op:'setParameters',parameters:[]}));
 ws.onclose=event=>{clearTimeout(timer);if(event.code===1008){console.log('Parameter writes rejected (1008)');resolve();}else reject(new Error('Unexpected close '+event.code));};
 ws.onerror=()=>{clearTimeout(timer);reject(new Error('Reconnect failed'));};
});
