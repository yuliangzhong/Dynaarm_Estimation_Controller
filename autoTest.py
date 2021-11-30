import os
import time
import sys
import ruamel.yaml

path = "sp_ws/src/state_estimation_rai/state_estimation_rai_problems/config/grasped_object_sim/ros.yaml"
def setPositionMean(x,z):
    yaml = ruamel.yaml.YAML()
    with open(path) as fIn:
        list_doc = yaml.load(fIn)
    list_doc['grasped_object_0']['state_initial_distribution']['position'][0] = x
    list_doc['grasped_object_0']['state_initial_distribution']['position'][2] = z
    with open(path, "w") as fOut:
        yaml.dump(list_doc, fOut)

def setRotMean(yr):
    yaml = ruamel.yaml.YAML()
    with open(path) as fIn:
        list_doc = yaml.load(fIn)
    list_doc['grasped_object_0']['state_initial_distribution']['angle_axis_orientation'][3] = yr
    with open(path, "w") as fOut:
        yaml.dump(list_doc, fOut)


setPositionMean(5,6)
setRotMean(9)



# os.system("roslaunch state_estimation_rai_problems grasped_object_sim.launch")




