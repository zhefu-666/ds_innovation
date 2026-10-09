#!/usr/bin/env python3
"""Run bounded read-only image/IMU/MCU preview and web bridge together."""
import argparse,subprocess,time,signal,sys,fcntl,os
from pathlib import Path
BASE=Path(__file__).resolve().parent

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--seconds',type=int,default=3600);a=p.parse_args()
 if not 5<=a.seconds<=3600:p.error('seconds must be 5..3600')
 lock=open(BASE/'view.lock','w');fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
 (BASE/'view.pid').write_text(str(os.getpid()))
 children=[]
 def stop(*_):raise KeyboardInterrupt
 signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
 logs=BASE/'logs';logs.mkdir(exist_ok=True)
 end=time.monotonic()+a.seconds
 try:
  with open(logs/'web.log','a') as web,open(logs/'preview.log','a') as preview:
   bridge=subprocess.Popen([sys.executable,str(BASE/'runtime/tools/remote_camera_telemetry/project_bridge.py'),'--snapshot','/dev/shm/rescue-demo-%d.bin'%os.getuid(),'--host','127.0.0.1','--http-port','8080','--ws-port','18765'],stdout=web,stderr=subprocess.STDOUT,start_new_session=True);children.append(bridge)
   time.sleep(.4)
   if bridge.poll() is not None:raise RuntimeError('Web bridge failed; see logs/web.log')
   while end-time.monotonic()>=5:
    seconds=min(60,int(end-time.monotonic()))
    child=subprocess.Popen([str(BASE/'search.sh'),'--preview','--seconds',str(seconds)],stdout=preview,stderr=subprocess.STDOUT,start_new_session=True);children.append(child)
    code=child.wait(timeout=seconds+15)
    if bridge.poll() is not None:raise RuntimeError('Web bridge exited')
    # Preview returns 2 on its normal time budget, inspect its latest output.
    preview.flush();tail=(logs/'preview.log').read_text()[-2000:]
    if code not in (0,2) or (code==2 and 'wall_budget_exhausted' not in tail):raise RuntimeError('Preview failed; see logs/preview.log')
    children.remove(child)
 except KeyboardInterrupt:pass
 finally:
  (BASE/'view.pid').unlink(missing_ok=True)
  for child in reversed(children):
   if child.poll() is None:
    os.killpg(child.pid,signal.SIGTERM)
    try:child.wait(timeout=5)
    except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait()
if __name__=='__main__':main()
