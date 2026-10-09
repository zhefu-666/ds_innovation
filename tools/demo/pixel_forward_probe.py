#!/usr/bin/env python3
"""Bounded image-only forward probe. No ground-distance decision, no grasp."""
import json,struct,time,subprocess,signal,sys
from pathlib import Path
import demo_serial as s
from demo import demo_lock,load_profile,ROOT

def select(d,locked=None):
    w,h=d['source_width'],d['source_height']
    if (w,h)!=(1280,720):raise ValueError('unexpected image size')
    candidates=[]
    for obj in d['detections']:
        b=obj['box'];x,y,bw,bh=[float(b[k]) for k in ('x','y','width','height')]
        if obj['label']=='ordinary_supply' and obj['confidence']>=.65 and bw>0 and bh>0 and 0<x<x+bw<w and 0<y<y+bh<h:
            candidates.append((bw*bh,obj,(x+bw/2)/w,(y+bh)/h))
    if not candidates:raise ValueError('no eligible ordinary target')
    _,obj,cx,bottom=max(candidates,key=lambda x:x[0]) if locked is None else next((x for x in candidates if x[1]['track_id']==locked),(0,None,0,0))
    if obj is None:raise ValueError('locked target lost')
    if abs(cx-.5)>.08:raise ValueError('target off centre')
    if bottom>=.97:raise ValueError('target at lower image boundary')
    for other in d['detections']:
        if other['track_id']==obj['track_id']:continue
        b=other['box']
        if other['confidence']>=.5 and b['x']<.6*w and b['x']+b['width']>.4*w and b['y']+b['height']>=bottom*h:
            raise ValueError('another detection in lower forward image corridor')
    return obj['track_id'],cx,bottom

def snapshot(path):
    with open(path,'rb') as f:
        hd=f.read(16)
        if hd[:8]!=b'RSTEL001':raise ValueError('invalid snapshot')
        n,j=struct.unpack('<II',hd[8:]);
        if not 0<n<1048576:raise ValueError('metadata length')
        d=json.loads(f.read(n))
    age=time.time()-(d['timestamp']['sec']+d['timestamp']['nsec']/1e9)
    if not 0<=age<.2:raise ValueError('stale vision')
    return d

def main():
    if sys.argv[1:]!=['--run','--site-clear','--watchdog-tested','--assume-done-bit']:
        raise ValueError('explicit run/site/watchdog/semantics required')
    profile=load_profile(ROOT/'config/demo_20261007.json')
    path=Path('/dev/shm/rescue-pixel-probe.bin');path.unlink(missing_ok=True)
    cmd=[profile['binary'],'--demo-mode','recognize','--demo-seconds','15','--no-show','--dry-run','--model',profile['model'],'--rknn-library',profile['rknn_library'],'--camera','0','--width','1280','--height','720','--fps','30','--startup-advance-ms','0','--telemetry','--telemetry-file',str(path)]
    with demo_lock('pixel-forward-probe'):
        with open(ROOT/'telemetry_logs/demo/pixel-forward-perception.log','w') as log:
            child=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT)
            try:
                with s.frame.serial_port('/dev/ttyACM0',115200,False) as fd:
                    parser=s.frame.Feedback();fb=s.initial_feedback(fd,parser)
                    if fb[:3]!=(1,24,45):raise ValueError('session state changed NO_TX')
                    locked=None;count=0;seq=None;until=time.monotonic()+8
                    while count<3:
                        if time.monotonic()>until or child.poll() is not None:raise TimeoutError('vision startup')
                        try:d=snapshot(path);candidate=select(d,locked)
                        except (FileNotFoundError,ValueError):
                            locked=None;count=0;seq=None;time.sleep(.02);continue
                        if d['sequence']!=seq:
                            seq=d['sequence'];locked=candidate[0];count+=1
                        time.sleep(.01)
                    zero=s.packet(0,0,0,24,45);forward=s.packet(.05,0,0,24,45)
                    start=last_rx=time.monotonic()
                    try:
                        while time.monotonic()-start<.5:
                            if child.poll() is not None:raise ValueError('perception stopped')
                            d=snapshot(path);target,cx,bottom=select(d,locked)
                            frames=s.frame.receive(fd,parser,.01)
                            for fb in frames:
                                if fb[:3]!=(1,24,45):raise ValueError('actuator state changed')
                                last_rx=time.monotonic()
                            if time.monotonic()-last_rx>.15:raise TimeoutError('A6 stale')
                            s.frame.send_packet(fd,forward,time.monotonic()+.05)
                            print(json.dumps({'event':'PIXEL_FORWARD','target':target,'cx':cx,'bottom':bottom,'vx':.05,'wz':0,'elapsed_s':time.monotonic()-start}),flush=True)
                            time.sleep(.02)
                    finally:
                        s.frame.send_packet(fd,zero,time.monotonic()+.15);print('ZERO_SENT',flush=True)
            finally:
                child.terminate()
                try:child.wait(timeout=3)
                except subprocess.TimeoutExpired:child.kill();child.wait()
                path.unlink(missing_ok=True)
if __name__=='__main__':
    signal.signal(signal.SIGTERM,lambda *_:sys.exit(130))
    main()
