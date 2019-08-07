import pytowr 
import os # change the path here does not work
import numpy as np
os.environ["LD_LIBRARY_PATH"] = "/home/yangcy/programs/towr/towr/build"+":"+os.environ["LD_LIBRARY_PATH"]
# terrain = lambda x,y: 0.0 # the terrain functiona
terrain = lambda x,y: 0.0 if x < 1 else 0.3
pos,cost,varDict = pytowr.run(2.,0.,0.01, terrain,{}) # target x, target y, time scale of the return list

print("cost",cost)

def raw2numpy(b, m,n):
    return np.frombuffer(b)

# The keys in VarDict are the names of towrvariables (轨迹变量的名字)
# The values in the VarDict are tripule(bytes, m,n) should go through raw2numpy first
varDict = {k:raw2numpy(*v) for k,v in varDict.items()}

import pickle as pkl
with open("tmpVardict.pkl","wb") as f:
    pkl.dump(varDict,f)
