// Node 22+; verifies both remote endpoints without extra packages.
import { writeFileSync } from 'node:fs';
const host = process.argv[2] || '192.168.1.123';
const output = process.argv[3] || '/tmp/remote-camera.jpg';
const status = await (await fetch(`http://${host}:8080/status`, {signal: AbortSignal.timeout(5000)})).json();
if (!status.camera_ok) throw Error(JSON.stringify(status));
console.log('HTTP status:', status);
const response = await fetch(`http://${host}:8080/stream.mjpg`, {signal: AbortSignal.timeout(5000)});
const reader = response.body.getReader();
let bytes = Buffer.alloc(0);
while (true) {
  const {value, done} = await reader.read();
  if (done) throw Error('Image stream ended');
  bytes = Buffer.concat([bytes, value]);
  const start = bytes.indexOf(Buffer.from([255,216]));
  const end = bytes.indexOf(Buffer.from([255,217]), start + 2);
  if (start >= 0 && end > start) {
    writeFileSync(output, bytes.subarray(start,end+2));
    await reader.cancel();
    console.log('HTTP JPEG saved:', output);
    break;
  }
}
await new Promise((resolve,reject) => {
  const ws = new WebSocket(`ws://${host}:8765`, 'foxglove.websocket.v1');
  ws.binaryType = 'arraybuffer';
  let images=0, health=0, last=0n, started;
  const timer = setTimeout(() => { ws.close(); reject(Error('WebSocket timeout')); },10000);
  ws.onerror = reject;
  ws.onmessage = ({data}) => {
    try {
      if (typeof data === 'string') {
        const msg=JSON.parse(data);
        if (msg.op==='serverInfo' && msg.capabilities.length) throw Error('Unexpected capabilities');
        if (msg.op==='advertise') {
          if (!msg.channels.some(c=>c.schemaName==='foxglove.CompressedImage')) throw Error('Missing image schema');
          ws.send(JSON.stringify({op:'subscribe',subscriptions:[{id:10,channelId:1},{id:11,channelId:2}]}));
          started=Date.now();
        }
        return;
      }
      const b=Buffer.from(data);
      if (b[0]!==1) throw Error('Invalid opcode');
      const id=b.readUInt32LE(1), stamp=b.readBigUInt64LE(5), payload=JSON.parse(b.subarray(13));
      if (id===10) {
        if (stamp<=last) throw Error('Non-increasing timestamp');
        last=stamp;
        const image=Buffer.from(payload.data,'base64');
        if (image[0]!==255 || image[1]!==216) throw Error('Invalid JPEG');
        writeFileSync(output.replace('.jpg','-foxglove.jpg'),image);
        images++;
      } else if (id===11) { if(!payload.camera_ok) throw Error('Camera unhealthy'); health++; }
      if (images>=20 && health>=20) {
        clearTimeout(timer); ws.close();
        console.log('Foxglove received:',{images,health,elapsed_ms:Date.now()-started}); resolve();
      }
    } catch(e) { clearTimeout(timer); ws.close(); reject(e); }
  };
});
await new Promise((resolve,reject) => {
  const ws=new WebSocket(`ws://${host}:8765`,'foxglove.websocket.v1');
  const timer=setTimeout(()=>{ws.close(); reject(Error('Control rejection timeout'));},5000);
  ws.onopen=()=>ws.send(JSON.stringify({op:'setParameters',parameters:[]}));
  ws.onclose=e=>{clearTimeout(timer); if(e.code!==1008) reject(Error(`Unexpected close ${e.code}`)); else {console.log('Control requests rejected; reconnect succeeded');resolve();}};
  ws.onerror=reject;
});
