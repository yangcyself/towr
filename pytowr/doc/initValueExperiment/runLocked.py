import sys
sys.path.append("../../") # to the pytowr location
import pytowr 
import os # change the path here does not work
import numpy as np
import pickle as pkl
# os.environ["LD_LIBRARY_PATH"] = "/home/yangcy/programs/towr/towr/build"+":"+os.environ["LD_LIBRARY_PATH"]

def raw2numpy(b, m,n):
    return np.frombuffer(b)

terrain = lambda x,y: 0.0 if x < 1 else 0.3
initDict = pytowr.initValues(2.,0.,0.01, terrain,None)

with open("tmpVardict.pkl","rb") as f:
    varDict  = pkl.load(f)

baselin = raw2numpy(*initDict["base-lin"]) # the returned arrays need raw2numpy to translate from raw bytes to numpy 
baselin = np.copy(baselin)

for i in range(len(baselin)): # assign presearched value of Px, Py to initvalues
    if(i%6==0 or i%6==1): # the Px,Py 
        baselin[i] = varDict["base-lin"][i] 

pos,cost,varDict = pytowr.run(2.,0.,0.01, terrain,None,{"base-lin":baselin}) # target x, target y, time scale of the return list

print("cost",cost)


# if __name__ == "__main__":
    # import pickle as pkl
    # with open("tmpVardict.pkl","wb") as f:
    #     # pkl.dump(pos,f)
    #     pkl.dump(varDict,f)
