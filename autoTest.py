import os
import ruamel.yaml
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import math
import time

ifNewData = True
ifSim = False # remember to check contact sim switch in sim.yaml!!
ifFast = False
edge = 4

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

    ## Ground Truth ##
    ####################################
    x_pos = 0.001
    z_pos = -0.005
    y_r = -4    

    ## Plot Setting ##
    ####################################
    figure = plt.gcf() # get current figure
    figure.set_size_inches(16, 7)
    axis1 = plt.subplot(1,2,1)
    axis2 = plt.subplot(1,2,2)

    range1 = 0.02
    axis1.set_title('Initial Guess Distribution: Position')
    axis1.set_xlabel('x pos [m]')
    axis1.set_ylabel('z pos [m]')
    axis1.set_aspect('equal', adjustable='box')
    axis1.grid(color='grey', linestyle='-', linewidth=1)


    axis2.set_title('Initial Guess Distribution: Rotation')
    axis2.set_xlabel('y rotation [degree]')
    axis2.set_ylabel('y rotation distribution')
    ylim2 = 8
    
    axis2.set_ylim([0,ylim2])
    axis2.set_yticks([])

    ## Initial Position ##
    ####################################
    Var_pos = 0.005

    meansX = np.linspace(-0.010+x_pos, 0.010+x_pos, edge).tolist()
    meansZ = np.linspace(-0.010+z_pos, 0.010+z_pos, edge).tolist()
    meansXZ = np.zeros([edge*edge,2])
    
    
    for i in range(edge):
        for j in range(edge):
            meansXZ[i*edge+j][0] = meansX[i]
            meansXZ[i*edge+j][1] = meansZ[j]
            circle = plt.Circle((meansX[i],meansZ[j]), Var_pos, color='red', edgecolor=None, alpha=0.05)
            circle.set_zorder(0)
            axis1.add_patch(circle)

    for i in range(edge):
        pos = axis1.scatter(meansX[i]*np.ones(edge),meansZ, s=5, color='black',alpha=0.8)
        pos.set_zorder(1)

    gt = plt.Circle((x_pos,z_pos), 0.001/3, color='blue', edgecolor=None, alpha=0.8)
    gt.set_zorder(5)
    axis1.add_patch(gt)

    ## Initial Rotation ##
    ####################################
    Var_rot = 5

    if(ifNewData):
        meansY = np.random.uniform(y_r-5,y_r+5,edge**2)
    else:
        meansY = np.loadtxt(configFile,delimiter = ",",skiprows = 1, usecols=2)

    N, bins, patches = axis2.hist(meansY,bins=100, color='black')
    
    rgt = axis2.plot([y_r,y_r],[0,ylim2],c='b', alpha=0.8)

    ## Save figure ##
    ####################################    
    plt.savefig('prior.png', dpi=200)
    
    ## Write down initial guesses ##
    ####################################
    if(ifNewData):    
        pd.DataFrame({'meansX':meansXZ[:,0], 'meansZ':meansXZ[:,1], 'meansY':meansY}).to_csv(configFile, index=False, float_format='%.5f')

    ## Start Auto Test ##
    ####################################
    if(ifNewData):
        meansY = (meansY/180*math.pi).tolist()
        pd.DataFrame({'x_dev':[], 'z_dev':[], 'y_rot':[]}).to_csv(resultFile, index=False, float_format='%.5f')
        os.system("rosclean purge -y")
        
        for i in range(edge):
            for j in range(edge):
                setPositionMean(meansX[i],meansZ[j])
                setRotMean(meansY[i*edge+j])
                if(ifSim):
                    os.system("roslaunch state_estimation_rai_problems grasped_object_sim.launch rviz:=false")
                elif(ifFast):
                    os.system("roslaunch state_estimation_rai_problems grasped_object_real_fast.launch rviz:=false")
                else:
                    os.system("roslaunch state_estimation_rai_problems grasped_object_real.launch rviz:=false")
                print("start sleeping")
                time.sleep(5)
                print("sleeping end")
                
    ## Data Analysis ##
    #################################### 
    results = np.loadtxt(resultFile,delimiter = ",",skiprows = 1)

    if(ifSim):
        axis1.set_title('[Sim] Estimation Distribution: Position')
        axis2.set_title('[Sim] Estimation Distribution: Rotation')
    elif(ifFast):
        axis1.set_title('[Real Fast mode] Estimation Distribution: Position')
        axis2.set_title('[Real Fast mode] Estimation Distribution: Rotation')
    else:
        axis1.set_title('[Real] Estimation Distribution: Position')
        axis2.set_title('[Real] Estimation Distribution: Rotation')

    
    for i in range(edge*edge):
        try:
            if((results[i,0]-x_pos)**2 + (results[i,1]-z_pos)**2 > 0.005**2):
                pos_est_fail = axis1.scatter(results[i,0], results[i,1], s=100, color='red')
                pos_old = axis1.scatter(meansXZ[i,0], meansXZ[i,1], s=100, color='red', marker='X')
                pos_est_fail.set_zorder(3)
                pos_old.set_zorder(3)
            else:
                pos_est = axis1.scatter(results[i,0], results[i,1], s=25, color='green')
                pos_est.set_zorder(3)
        except:
            pass
    
    axis1.legend(handles=[pos,circle,gt,pos_est],
                 labels=['initial pos guess mean','initial pos guess distribution (1$\sigma$)', 'ground truth','pos estimation result'],
                 loc='best')


    results[:,2] = results[:,2]/math.pi*180
    for i in range(edge*edge):
        try:
            y_degree = results[i,2]
            if(y_degree>-1 or y_degree<-7):
                rot_est = axis2.plot([y_degree,y_degree],[0,ylim2],c='red', alpha=0.8)
            else:
                rot_est = axis2.plot([y_degree,y_degree],[0,ylim2],c='green', alpha=0.8)
        except:
            pass

    Handles = [Rectangle((0,0),1,1,color='black'),
               Rectangle((0,0),1,1,color='blue', alpha=0.8),
               Rectangle((0,0),1,1,color='green', alpha=0.8)]
    Labels = ['initial rot guess mean', 'ground truth','rot estimation result']
    axis2.legend(handles=Handles,labels=Labels,loc='best')
    
    plt.savefig('posterior.png', dpi=200)
    plt.show()