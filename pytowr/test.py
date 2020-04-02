import pytowr 
import os # change the path here does not work
import numpy as np

os.environ["LD_LIBRARY_PATH"] = "/home/dada/catkin_ws/src/towr/towr/build"+":"+os.environ["LD_LIBRARY_PATH"]
# terrain = lambda x,y: 0.0 # the terrain functiona
# terrain = lambda x,y: 0.0 if x < 1 else 0.3
terrain = lambda x,y: 10.0 if (0.8<x<1.2 and -0.2<y<0.2) else 0.0
# def terrain(x,y):
#     if(0.8<x<1.2 and -0.2<y<0.2):
#     # if(0.7<x<1.3 and -0.3<y<0.3):
#         # print("#####CALLEDME#######")
#         return 10
#     else:
#         return 0
pos,cost,varDict = pytowr.run(1,2,1.5,0,0.01, terrain,None,{}) # target x, target y, time scale of the return list
print("cost",cost)
def showtime(i):
    a = pos[i]
    print("time: ",a[0] ,"body: ",a[1],a[2])
    foots = a[3]
    for f,p in enumerate(foots):
        print( f, p ) # (position of foot point, is_contacting_ground)
def raw2numpy(b, m,n):
    return np.frombuffer(b)
varDict = {k:raw2numpy(*v) for k,v in varDict.items()}

if __name__ == "__main__":
    # import pickle as pkl
    # with open("tmpVardict.pkl","wb") as f:
    #     # pkl.dump(pos,f)
    #     pkl.dump(varDict,f)
    import matplotlib.pyplot as plt
    basex = [p[1][0] for p in pos]
    basey = [p[1][1] for p in pos]
    plt.plot(basex,basey)
    for i in range(6):
        eex = [p[3][i][0][0] for p in pos]
        eey = [p[3][i][0][1] for p in pos]
        plt.plot(eex,eey)
    
    plt.show()
    # for i in range(6):
    #     del(varDict["ee-motion_%d"%i])
    #     del(varDict["ee-force_%d"%i])
    # p,c,v =  pytowr.run(2.,0.,0.01, terrain,varDict)
    # print("new cost:",c)