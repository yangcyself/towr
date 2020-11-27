import pytowr 
import os # change the path here does not work
import numpy as np
import math

os.environ["LD_LIBRARY_PATH"] = "/home/dada/catkin_ws/src/towr/towr/build"+":"+os.environ["LD_LIBRARY_PATH"]

# terrain = lambda x,y: 0.0 # the terrain functiona
# terrain = lambda x,y: 0.0 if x < 1 else 0.3
terrain = lambda x,y: 0.2 if (0.4<x<0.8) else 0.0
# def terrain(x,y):
#     if 0.75<x<1.25 and -1<y<1:
#         return 0.05
#     elif 1.25<=x<1.75 and -1 < y < 1:
#         return 0.1
#     elif 1.75<=x<2.25 and -1 < y < 1:
#         return 0.15
#     elif 2.25<=x<2.75 and -1<y<1:
#         return 0.2
#     elif 2.75<=x<3.05 and -1<y<1:
#         return 0.25
#     else:
#         return 0

pos,math.cost,varDict = pytowr.run(0,0.8,0,0,0.05, terrain,None,{}) # target x, target y, time scale of the return list

print("math.cost",math.cost)
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
    import pickle as pkl
    with open("tmpVardict.pkl","wb") as f:
        # pkl.dump(pos,f)
        pkl.dump(varDict,f)
    import matplotlib.pyplot as plt
    basex = [p[1][0] for p in pos]
    basey = [p[1][1] for p in pos]
    basez = [p[1][2] for p in pos]
    plt.plot(basex,basey)

    basea = [p[2][0] for p in pos]
    baseb = [p[2][1] for p in pos]
    basec = [p[2][2] for p in pos]
    # plt.plot(basea)
    # plt.plot(baseb)
    # plt.plot(basec)
    plt.show()
    
    # plt.plot(basex,basey)
    eex = list()
    eey = list()
    eez = list()
    for i in range(6):
        eex.append([p[3][i][0][0] for p in pos])
        eey.append([p[3][i][0][1] for p in pos])
        eez.append([p[3][i][0][2] for p in pos])
        
        # plt.plot(eex,eey)
    print(eex)

    x_nominal_1 = 0.528
    x_nominal_2 = 0
    y_nominal_1 = 0.304
    y_nominal_2 = 0.609
    z_nominal = -0.6
    x_position_1 = 0.131
    x_position_2 = 0.274
    x_position_3 = 0.182
    x_position_4 = 0.092
    y_position_1 = 0.076
    y_position_2 = 0.052
    y_position_3 = 0.211
    y_position_4 = 0.152
    y_position_5 = 0.264
    z_position_0 = 0.117

    nominalLA =  [x_nominal_1,   y_nominal_1, z_nominal]
    nominalLB =  [x_nominal_2,   y_nominal_2, z_nominal]
    nominalLC =  [x_nominal_1,   y_nominal_1, z_nominal]
    nominalRA =  [x_nominal_1,  -y_nominal_1, z_nominal]
    nominalRB =  [x_nominal_2,  -y_nominal_2, z_nominal]
    nominalRC =  [-x_nominal_1,  -y_nominal_1, z_nominal]


    rootLA =  [[x_position_1, y_position_1  ,0], [x_position_2, y_position_2  ,z_position_0], [x_position_3, y_position_3  ,z_position_0]]

    rootLB =  [[0,  y_position_4 ,0], [x_position_4 , y_position_5  ,z_position_0], [-x_position_4  , y_position_5  ,z_position_0]]

    rootLC =  [[-x_position_1, y_position_1,0], [-x_position_3 ,y_position_3  ,z_position_0], [-x_position_2, y_position_2  ,z_position_0]]

    rootRA =  [[x_position_1, -y_position_1  ,0], [x_position_3, -y_position_3 ,z_position_0], [x_position_2, -y_position_2 ,z_position_0]]

    rootRB =  [[0, -y_position_4  ,0], [-x_position_4, -y_position_5   ,z_position_0], [x_position_4, -y_position_5  ,z_position_0]]

    rootRC = [[-x_position_1, -y_position_1     ,0], [-x_position_2, -y_position_2 ,z_position_0], [-x_position_3, -y_position_3  ,z_position_0]]

    l1 = [[],[],[]]
    l2 = [[],[],[]]
    l3 = [[],[],[]]
    l4 = [[],[],[]]
    l5 = [[],[],[]]
    l6 = [[],[],[]]
    for i in range(len(basex)):
        x = basea[i]
        y = baseb[i]
        z = basec[i]
        M = np.array([[math.cos(y)*math.cos(z), math.cos(z)*math.sin(x)*math.sin(y) - math.cos(x)*math.sin(z), math.sin(x)*math.sin(z) + math.cos(x)*math.cos(z)*math.sin(y)], [math.cos(y)*math.sin(z), math.cos(x)*math.cos(z) + math.sin(x)*math.sin(y)*math.sin(z), math.cos(x)*math.sin(y)*math.sin(z) - math.cos(z)*math.sin(x)], [-math.sin(y), math.cos(y)*math.sin(x), math.cos(x)*math.cos(y)]])
        for j in range(3):
            p = np.dot(M,np.array([[rootLA[j][0] + basex[i]],[rootLA[j][1] + basey[i]],[rootLA[j][2] + basez[i]]]))
            l1[j].append(((p[0][0] - eex[0][i]) ** 2 + (p[1][0] - eey[0][i]) ** 2 + (p[2][0] - eez[0][i]) ** 2) ** 0.5)
            p = np.dot(M,np.array([[rootLB[j][0] + basex[i]],[rootLB[j][1] + basey[i]],[rootLB[j][2] + basez[i]]]))
            l2[j].append(((p[0][0] - eex[1][i]) ** 2 + (p[1][0] - eey[1][i]) ** 2 + (p[2][0] - eez[1][i]) ** 2) ** 0.5)
            p = np.dot(M,np.array([[rootLC[j][0] + basex[i]],[rootLC[j][1] + basey[i]],[rootLC[j][2] + basez[i]]]))
            l3[j].append(((p[0][0] - eex[2][i]) ** 2 + (p[1][0] - eey[2][i]) ** 2 + (p[2][0] - eez[2][i]) ** 2) ** 0.5)
            p = np.dot(M,np.array([[rootRA[j][0] + basex[i]],[rootRA[j][1] + basey[i]],[rootRA[j][2] + basez[i]]]))
            l4[j].append(((p[0][0] - eex[3][i]) ** 2 + (p[1][0] - eey[3][i]) ** 2 + (p[2][0] - eez[3][i]) ** 2) ** 0.5)
            p = np.dot(M,np.array([[rootRB[j][0] + basex[i]],[rootRB[j][1] + basey[i]],[rootRB[j][2] + basez[i]]]))
            l5[j].append(((p[0][0] - eex[4][i]) ** 2 + (p[1][0] - eey[4][i]) ** 2 + (p[2][0] - eez[4][i]) ** 2) ** 0.5)
            p = np.dot(M,np.array([[rootRC[j][0] + basex[i]],[rootRC[j][1] + basey[i]],[rootRC[j][2] + basez[i]]]))
            l6[j].append(((p[0][0] - eex[5][i]) ** 2 + (p[1][0] - eey[5][i]) ** 2 + (p[2][0] - eez[5][i]) ** 2) ** 0.5)

    for i in range(3):
        plt.plot(l1[i])
        plt.plot(l2[i])
        plt.plot(l3[i])
        plt.plot(l4[i])
        plt.plot(l5[i])
        plt.plot(l6[i])
    plt.show()
    # for i in range(6):
    #     del(varDict["ee-motion_%d"%i])
    #     del(varDict["ee-force_%d"%i])
    # p,c,v =  pytowr.run(2.,0.,0.01, terrain,varDict)
    # print("new math.cost:",c)
    # i = len(pos)
    # showtime(i)