import os
import sys
import ruamel.yaml
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


YAMLpath = "sp_ws/src/state_estimation_rai/state_estimation_rai_problems/config/grasped_object_sim/ros.yaml"
configFile = "config.csv"
resultFile = "result.csv"

def setPositionMean(x,z):
    yaml = ruamel.yaml.YAML()
    with open(YAMLpath) as fIn:
        list_doc = yaml.load(fIn)
    list_doc['grasped_object_0']['state_initial_distribution']['position'][0] = x
    list_doc['grasped_object_0']['state_initial_distribution']['position'][2] = z
    with open(YAMLpath, "w") as fOut:
        yaml.dump(list_doc, fOut)

def setRotMean(yr):
    yaml = ruamel.yaml.YAML()
    with open(YAMLpath) as fIn:
        list_doc = yaml.load(fIn)
    list_doc['grasped_object_0']['state_initial_distribution']['angle_axis_orientation'][3] = yr
    with open(YAMLpath, "w") as fOut:
        yaml.dump(list_doc, fOut)



if __name__ == "__main__":

    testNum = 1000
    testStamp = np.random.rand()

    ## Plot Setting ##
    ####################################
    axis1 = plt.subplot(1,2,1)
    axis2 = plt.subplot(1,2,2)

    range1 = 0.02
    axis1.set_title('Initial Guess Distribution: Position')
    axis1.set_xlabel('x pos [m]')
    axis1.set_ylabel('z pos [m]')
    axis1.set_xlim([0.001-range1, 0.001+range1])
    axis1.set_ylim([-0.005-range1, -0.005+range1])
    axis1.set_aspect('equal', adjustable='box')


    axis2.set_title('Initial Guess Distribution: Rotation')
    axis2.set_xlabel('y rotation [degree]')
    axis2.set_ylabel('y rotation distribution')
    axis2.set_xlim([-11, +3])
    axis2.set_yticks([])
    ylim2 = 23.5
    axis2.set_ylim([0,ylim2])

    ## Initial Position ##
    ####################################
    Var_pos = 0.005
    meansX = np.random.normal(0.001,0.004,testNum)
    meansZ = np.random.normal(-0.005,0.004,testNum)
    
    for i in range(testNum):
        if((meansX[i]-0.001)**2 + (meansZ[i]+0.005)**2<0.008**2):
            if np.random.rand()>0.2:
                continue
        circle = plt.Circle((meansX[i],meansZ[i]), Var_pos, color='red', edgecolor=None, alpha=0.05)
        circle.set_zorder(0)
        axis1.add_patch(circle)


    pos = axis1.scatter(meansX,meansZ, s=5, color='black',alpha=0.8)
    pos.set_zorder(1)

    gt = plt.Circle((0.001,-0.005), 0.001/3, color='blue', edgecolor=None, alpha=0.8)
    gt.set_zorder(2)
    axis1.add_patch(gt)

    axis1.legend(handles=[pos,circle,gt],labels=['initial pos guess mean','initial pos guess distribution (1$\sigma$)', 'ground truth'],loc='best')

    # ## Initial Rotation ##
    # ####################################
    Var_rot = 5
    meansY = np.random.normal(-4,2,testNum)
    N, bins, patches = axis2.hist(meansY,bins=200, color='red', alpha=0.5)

    zeros = np.zeros(testNum)
    y_rots = axis2.scatter(meansY,zeros, s=15, color='black',alpha=0.8)
    y_rots.set_zorder(5)

    rgt = axis2.plot([-4,-4],[0,ylim2],c='b')

    axis2.legend(handles=[y_rots,Rectangle((0,0),1,1,color='red', alpha=0.5),Rectangle((0,0),1,1,color='blue')],labels=['initial rot guess mean','initial rot guess mean distribution', 'ground truth'],loc='best')

    plt.savefig('test.pdf') # make it clear!!!!!!!!!!!!!
    plt.show()




    # for i in range(testNum):
    #     os.system("roslaunch state_estimation_rai_problems grasped_object_sim.launch")




