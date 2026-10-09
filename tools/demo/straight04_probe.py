#!/usr/bin/env python3
"""User-authorized clear-straight test: fixed 0.4 m/s, at most 0.2 s, no rotation.
Not an autonomous approach policy. Keeps session-known frame angle 0/id24/RX45.
"""
import signal,struct,sys,time,json
import demo_serial as s
from demo import demo_lock

def packet(vx):
    if vx not in (0,.4):raise ValueError('Only zero or 0.4 m/s')
    raw=struct.pack('<BffbBh',0x56,vx,0.,0,24,5)
    return raw+struct.pack('<H',s.frame.crc16(raw))

def run(fd,parser,emit=print):
    initial=s.initial_feedback(fd,parser)
    if initial[:3]!=(1,24,45):raise ValueError('Session state differs; NO TX')
    stop=packet(0);forward=packet(.4)
    start=last_rx=time.monotonic();count=0
    try:
        while time.monotonic()-start<.2:
            frames=s.frame.receive(fd,parser,.005)
            for fb in frames:
                if fb[:3]!=(1,24,45):raise ValueError('Feedback changed')
                last_rx=time.monotonic()
            now=time.monotonic()
            if now-last_rx>.10:raise TimeoutError('A6 stale')
            if now-start>=.2:break
            s.frame.send_packet(fd,forward,min(start+.2,now+.02));count+=1
            emit(json.dumps({'vx_command':.4,'wz':0,'id':24,'elapsed_s':now-start}))
            time.sleep(min(.01,max(0,start+.2-time.monotonic())))
    finally:
        s.frame.send_packet(fd,stop,time.monotonic()+.10)
        emit(json.dumps({'event':'ZERO_SENT','elapsed_s':time.monotonic()-start,'packets':count,'physical_speed_measured':False}))

def main():
    if sys.argv[1:]!=['--run','--clear-straight-confirmed','--watchdog-tested']:
        print('PREVIEW NO_TX: .4 m/s, .2 s, nominal .08 m, packet='+packet(.4).hex(' '));return
    signal.signal(signal.SIGTERM,lambda *_:sys.exit(130))
    with demo_lock('straight04-probe'):
        with s.frame.serial_port('/dev/ttyACM0',115200,False) as fd:
            run(fd,s.frame.Feedback(),lambda x:print(x,flush=True))
if __name__=='__main__':main()
