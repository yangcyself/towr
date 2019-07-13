import pytowr 
import os # change the path here does not work
os.environ["LD_LIBRARY_PATH"] = "/home/yangcy/programs/towr/towr/build"+":"+os.environ["LD_LIBRARY_PATH"]
pos = pytowr.run(5.,0.,0.1)
def showtime(i):
    a = pos[i]
    print("time: ",a[0] ,"body: ",a[1],a[2])
    foots = a[3]
    for f,p in enumerate(foots):
        print( f, p )