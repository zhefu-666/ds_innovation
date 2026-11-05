import numpy as np
from host_decode import decode

def decode_raw(outputs,gain,pad,shape,router):
    boxes,scores,base,far,vis=[np.asarray(x,dtype=np.float32) for x in outputs]
    assert [list(x.shape) for x in (boxes,scores,base,far,vis)]==[[1,4,8400],[1,2,8400],[1,8,8400],[1,8,8400],[1,4,8400]]
    grid=[];strides=[]
    for size,stride in [(80,8),(40,16),(20,32)]:
        y,x=np.meshgrid(np.arange(size),np.arange(size),indexing='ij');grid.append(np.stack([x.ravel(),y.ravel()],axis=0));strides.append(np.full(size*size,stride))
    grid=np.concatenate(grid,axis=1).astype(np.float32);stride=np.concatenate(strides).astype(np.float32)
    def xy(raw):
        pts=raw.reshape(1,4,2,8400)
        return ((pts*2+grid[None,None,:,:])*stride[None,None,None,:]).reshape(1,8,8400)
    return decode([boxes,scores,np.concatenate([xy(base),xy(far)],axis=1),vis],gain,pad,shape,router)
