"""Causal postprocessing only. No inference, weight changes, GT, or future frames."""
import copy,math
import numpy as np
class Int8PointSmoother:
 def __init__(self,config):
  self.config=dict(config);self.reset()
 def reset(self):self.points={};self.boxes={};self.last_time=None
 @staticmethod
 def iou(a,b):
  a,b=np.asarray(a),np.asarray(b);inter=np.maximum(np.minimum(a[2:],b[2:])-np.maximum(a[:2],b[:2]),0).prod();union=np.maximum(a[2:]-a[:2],0).prod()+np.maximum(b[2:]-b[:2],0).prod()-inter;return float(inter/max(union,1e-9))
 @staticmethod
 def alpha(cutoff,dt):return -math.expm1(-2*math.pi*cutoff*dt)
 def update(self,detections,timestamp,gain,pad,image_shape):
  if not math.isfinite(timestamp) or gain<=0:raise ValueError('finite timestamp and positive gain required')
  global_reset=None
  if self.last_time is not None and (timestamp<=self.last_time or timestamp-self.last_time>self.config['max_gap_seconds']):self.reset();global_reset='time_discontinuity'
  self.last_time=timestamp;h,w=image_shape[:2];out=[];raw_selected=[];audit=[]
  for cls in (0,1):
   candidates=[d for d in detections if d['cls']==cls and d['score']>=self.config['detection']];reason=global_reset
   if not candidates:
    self.boxes.pop(cls,None)
    for k in range(4):self.points.pop((cls,k),None)
    audit.append({'cls':cls,'selection':'missing','points':[]});continue
   det=max(candidates,key=lambda d:d['score']);raw_selected.append(copy.deepcopy(det));box=np.asarray(det['box'],dtype=float)
   if not np.isfinite(box).all():raise ValueError('nonfinite box')
   if len(candidates)>1:reason='ambiguous_raw_reset'
   elif cls in self.boxes and self.iou(self.boxes[cls],box)<self.config['class_box_reset_iou']:reason='box_discontinuity_raw_reset'
   if reason:
    for k in range(4):self.points.pop((cls,k),None)
   self.boxes[cls]=box.copy();result=copy.deepcopy(det);details=[]
   for slot,p in enumerate(det['kpt']):
    key=(cls,slot);xy=np.array(p[:2],dtype=float);confidence=float(p[2]);info={'slot':slot,'measured_this_frame':bool(confidence>=self.config['visibility']),'predicted_only':False,'cutoff_hz':None,'alpha':None}
    if not np.isfinite(xy).all() or not math.isfinite(confidence):raise ValueError('nonfinite point')
    if confidence<self.config['visibility']:
     self.points.pop(key,None);info.update(status='invalid_invisible',xy=None);details.append(info);continue
    if not(0<=xy[0]<w and 0<=xy[1]<h):
     self.points.pop(key,None);info.update(status='out_of_frame_raw_reset',xy=xy.tolist());details.append(info);continue
    current=xy*gain+np.asarray(pad);prev=self.points.get(key)
    if prev is None:
     filtered=current.copy();velocity=np.zeros(2);info.update(status=reason or 'acquired_raw',xy=xy.tolist())
    else:
     dt=timestamp-prev['t'];raw_velocity=(current-prev['raw'])/dt;da=self.alpha(self.config['derivative_cutoff_hz'],dt);velocity=prev['velocity']+da*(raw_velocity-prev['velocity']);cutoff=min(self.config['max_cutoff_hz'],self.config['min_cutoff_hz']+self.config['beta_hz_per_pixel_per_second']*float(np.linalg.norm(velocity)));a=self.alpha(cutoff,dt);filtered=prev['filtered']+a*(current-prev['filtered']);filtered_original=(filtered-np.asarray(pad))/gain;result['kpt'][slot][:2]=filtered_original.tolist();info.update(status='measured_smoothed',xy=filtered_original.tolist(),cutoff_hz=cutoff,alpha=a)
    if reason=='ambiguous_raw_reset':self.points.pop(key,None)
    else:self.points[key]={'raw':current.copy(),'filtered':filtered.copy(),'velocity':velocity.copy(),'t':timestamp}
    details.append(info)
   out.append(result);audit.append({'cls':cls,'candidate_count':len(candidates),'selection':'highest_score_unchanged','reset':reason,'points':details})
  return {'raw_selected':raw_selected,'gated':copy.deepcopy(raw_selected),'filtered':out,'audit':audit,'global_reset':global_reset}
