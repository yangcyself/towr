import pytowr 
import os # change the path here does not work
os.environ["LD_LIBRARY_PATH"] = "/home/yangcy/programs/towr/towr/build"+":"+os.environ["LD_LIBRARY_PATH"]
# terrain = lambda x,y: 0.0 # the terrain functiona
terrain = lambda x,y: 0.0 if x < 1 else 0.3
pos,cost = pytowr.run(2.,0.,0.01, terrain) # target x, target y, time scale of the return list
print("cost",cost)
def showtime(i):
    a = pos[i]
    print("time: ",a[0] ,"body: ",a[1],a[2])
    foots = a[3]
    for f,p in enumerate(foots):
        print( f, p ) # (position of foot point, is_contacting_ground)

if __name__ == "__main__":
    import pickle as pkl
    with open("pytowrDump.pkl","wb") as f:
        pkl.dump(pos,f)