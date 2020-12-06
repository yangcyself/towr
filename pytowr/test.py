import pytowr 
import os # change the path here does not work
import numpy as np
import math

os.environ["LD_LIBRARY_PATH"] = "/home/dada/catkin_ws/src/towr/towr/build"+":"+os.environ["LD_LIBRARY_PATH"]

# terrain = lambda x,y: 0.0 # the terrain functiona
# terrain = lambda x,y: 0.0 if x < 1 else 0.3
# terrain = lambda x,y: 0.3 if (0.7<x<2) else 0.0
def terrain(x,y):
    if 0.75<x<1.25 and -1<y<1:
        return 0.1
    elif 1.25<=x<1.75 and -1 < y < 1:
        return 0.2
    elif 1.75<=x<2.25 and -1 < y < 1:
        return 0.25
    else:
        return 0

pos,math.cost,varDict = pytowr.run(0,2.0,0,0.25,0.05, terrain,None,{}) # target x, target y, time scale of the return list

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
    # plt.plot(basex,basey)
    plt.plot(basez)

    basea = [p[2][0] for p in pos]
    baseb = [p[2][1] for p in pos]
    basec = [p[2][2] for p in pos]
    # plt.plot(basea)
    # plt.plot(baseb)
    # plt.plot(basec)
    plt.show()
    
    plt.plot(basex,basey)
    eex = list()
    eey = list()
    eez = list()
    contact = list()
    for i in range(6):
        eex.append([p[3][i][0][0] for p in pos])
        eey.append([p[3][i][0][1] for p in pos])
        eez.append([p[3][i][0][2] for p in pos])
        contact.append([p[3][i][1] for p in pos])
        
        # plt.plot(eex,eey)
        plt.plot(eex[i],eey[i])

    plt.show()
    for i in range(6):
        plt.plot(eez[i])
    plt.show()

    x_nominal_1 = 0.528
    x_nominal_2 = 0
    y_nominal_1 = 0.304
    y_nominal_2 = 0.609
    z_nominal = -0.5
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
    nominalLC =  [-x_nominal_1,   y_nominal_1, z_nominal]
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
            p = np.dot(M.T,np.array([[eex[0][i] - basex[i]],[eey[0][i] - basey[i]],[eez[0][i] - basez[i]]]))
            l1[j].append(((p[0][0] - rootLA[j][0]) ** 2 + (p[1][0] - rootLA[j][1]) ** 2 + (p[2][0] - rootLA[j][2]) ** 2) ** 0.5)
            p = np.dot(M.T,np.array([[eex[1][i] - basex[i]],[eey[1][i] - basey[i]],[eez[1][i] - basez[i]]]))
            l2[j].append(((p[0][0] - rootLB[j][0]) ** 2 + (p[1][0] - rootLB[j][1]) ** 2 + (p[2][0] - rootLB[j][2]) ** 2) ** 0.5)
            p = np.dot(M.T,np.array([[eex[2][i] - basex[i]],[eey[2][i] - basey[i]],[eez[2][i] - basez[i]]]))
            l3[j].append(((p[0][0] - rootLC[j][0]) ** 2 + (p[1][0] - rootLC[j][1]) ** 2 + (p[2][0] - rootLC[j][2]) ** 2) ** 0.5)
            p = np.dot(M.T,np.array([[eex[3][i] - basex[i]],[eey[3][i] - basey[i]],[eez[3][i] - basez[i]]]))
            l4[j].append(((p[0][0] - rootRA[j][0]) ** 2 + (p[1][0] - rootRA[j][1]) ** 2 + (p[2][0] - rootRA[j][2]) ** 2) ** 0.5)
            p = np.dot(M.T,np.array([[eex[4][i] - basex[i]],[eey[4][i] - basey[i]],[eez[4][i] - basez[i]]]))
            l5[j].append(((p[0][0] - rootRB[j][0]) ** 2 + (p[1][0] - rootRB[j][1]) ** 2 + (p[2][0] - rootRB[j][2]) ** 2) ** 0.5)
            p = np.dot(M.T,np.array([[eex[5][i] - basex[i]],[eey[5][i] - basey[i]],[eez[5][i] - basez[i]]]))
            l6[j].append(((p[0][0] - rootRC[j][0]) ** 2 + (p[1][0] - rootRC[j][1]) ** 2 + (p[2][0] - rootRC[j][2]) ** 2) ** 0.5)

    for i in range(3):
        plt.plot(l1[i])
        plt.plot(l2[i])
        plt.plot(l3[i])
        plt.plot(l4[i])
        plt.plot(l5[i])
        plt.plot(l6[i])
    plt.show()

    # with open("rod.txt","w") as f:
    #     for i in range(len(l1[0])):
    #         for j in range(3):
    #             f.write(str(l1[j][i]) + ' ')
    #         for j in range(3):
    #             f.write(str(l2[j][i]) + ' ')
    #         for j in range(3):
    #             f.write(str(l3[j][i]) + ' ')
    #         for j in range(3):
    #             f.write(str(l4[j][i]) + ' ')
    #         for j in range(3):
    #             f.write(str(l5[j][i]) + ' ')
    #         for j in range(3):
    #             f.write(str(l6[j][i]) + ' ')
    #         f.write('\r\n')
    # for i in range(6):
    #     del(varDict["ee-motion_%d"%i])
    #     del(varDict["ee-force_%d"%i])
    # p,c,v =  pytowr.run(2.,0.,0.01, terrain,varDict)
    # print("new math.cost:",c)
    # i = len(pos)
    # showtime(i)