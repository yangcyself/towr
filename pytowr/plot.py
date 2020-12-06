import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation

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

fig = plt.figure(1)
ax = fig.add_subplot(1, 1, 1, projection='3d')  # 指定三维空间做图

for i in range(3):
    ax.plot([rootLA[i][0], rootLB[i][0], rootLC[i][0], rootRC[i][0], rootRB[i][0], rootRA[i][0], rootLA[i][0]], [rootLA[i][1], rootLB[i][1], rootLC[i][1], rootRC[i][1], rootRB[i][1], rootRA[i][1], rootLA[i][1]], [rootLA[i][2], rootLB[i][2], rootLC[i][2], rootRC[i][2], rootRB[i][2], rootRA[i][2], rootLA[i][2]], color='b',label='line')
    ax.plot([nominalLA[0], rootLA[i][0]], [nominalLA[1], rootLA[i][1]], [nominalLA[2], rootLA[i][2]], color='r',label='line')
    ax.plot([nominalLB[0], rootLB[i][0]], [nominalLB[1], rootLB[i][1]], [nominalLB[2], rootLB[i][2]], color='r',label='line')
    ax.plot([nominalLC[0], rootLC[i][0]], [nominalLC[1], rootLC[i][1]], [nominalLC[2], rootLC[i][2]], color='r',label='line')
    ax.plot([nominalRC[0], rootRC[i][0]], [nominalRC[1], rootRC[i][1]], [nominalRC[2], rootRC[i][2]], color='r',label='line')
    ax.plot([nominalRB[0], rootRB[i][0]], [nominalRB[1], rootRB[i][1]], [nominalRB[2], rootRB[i][2]], color='r',label='line')
    ax.plot([nominalRA[0], rootRA[i][0]], [nominalRA[1], rootRA[i][1]], [nominalRA[2], rootRA[i][2]], color='r',label='line')
ax.axis('off')
plt.show()