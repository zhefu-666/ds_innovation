"""Four-output two-expert A decoder; frozen training-derived router."""
import cv2
import numpy as np

def letterbox(image):
    h,w=image.shape[:2];gain=min(640/h,640/w);nw,nh=round(w*gain),round(h*gain)
    px,py=(640-nw)/2,(640-nh)/2
    image=cv2.resize(image,(nw,nh),interpolation=cv2.INTER_LINEAR) if (nw,nh)!=(w,h) else image
    image=cv2.copyMakeBorder(image,round(py-.1),round(py+.1),round(px-.1),round(px+.1),cv2.BORDER_CONSTANT,value=(114,114,114))
    return cv2.cvtColor(image,cv2.COLOR_BGR2RGB)[None],gain,[px,py]

def decode(outputs,gain,pad,shape,router):
    boxes,scores,xy,visibility=[np.asarray(x,dtype=np.float32)[0].T for x in outputs]
    assert boxes.shape==(8400,4) and scores.shape==(8400,2) and xy.shape==(8400,16) and visibility.shape==(8400,4)
    assert all(np.isfinite(x).all() for x in outputs)
    cls=scores.argmax(1);score=scores.max(1);ids=np.where(score>.5)[0]
    corners=np.concatenate((boxes[:,:2]-boxes[:,2:]/2,boxes[:,:2]+boxes[:,2:]/2),axis=1)
    order=ids[np.argsort(-score[ids],kind='stable')][:30000];keep=[]
    while len(order) and len(keep)<300:
        i=order[0];keep.append(int(i));rest=order[1:]
        if not len(rest):break
        intersect=np.maximum(np.minimum(corners[i,2:],corners[rest,2:])-np.maximum(corners[i,:2],corners[rest,:2]),0).prod(1)
        ai=np.maximum(corners[i,2:]-corners[i,:2],0).prod();ar=np.maximum(corners[rest,2:]-corners[rest,:2],0).prod(1)
        overlap=intersect/(ai+ar-intersect+1e-7)
        order=rest[(cls[rest]!=cls[i]) | (overlap<=.45)]
    h,w=shape[:2];bounds=corners[keep].copy();bounds[:,[0,2]]=(bounds[:,[0,2]]-pad[0])/gain;bounds[:,[1,3]]=(bounds[:,[1,3]]-pad[1])/gain
    bounds[:,[0,2]]=bounds[:,[0,2]].clip(0,w);bounds[:,[1,3]]=bounds[:,[1,3]].clip(0,h)
    counts=[int(sum(cls[i]==c for i in keep)) for c in (0,1)];weight=0.;area=0.;reason='missing_or_ambiguous'
    if counts==[1,1]:
        area=float(((bounds[:,2]-bounds[:,0])*(bounds[:,3]-bounds[:,1])/(h*w)).max())
        weight=float(np.clip((router['near_min']-area)/(router['near_min']-router['far_max']),0,1));reason='scale'
    out=[]
    for j,i in enumerate(keep):
        points=xy[i,:8].reshape(4,2)*(1-weight)+xy[i,8:].reshape(4,2)*weight
        points[:,0]=((points[:,0]-pad[0])/gain).clip(0,w);points[:,1]=((points[:,1]-pad[1])/gain).clip(0,h)
        kp=np.concatenate((points,visibility[i,:,None]),axis=1)
        out.append({'cls':int(cls[i]),'score':float(score[i]),'box':bounds[j].tolist(),'kpt':kp.tolist()})
    return out,{'weight_far':weight,'max_area_fraction':area,'counts':counts,'reason':reason}
