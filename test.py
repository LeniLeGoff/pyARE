#! /bin/usr/python3

import sys
import pybullet as pb
import are_robot_utils as aru
import time
import pybullet_data
#import robot_sensors as rs
import math

physicsClient = pb.connect(pb.DIRECT)#or p.DIRECT for non-graphical version
pb.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
pb.setGravity(0,0,-10)
planeId = pb.loadURDF("plane.urdf")
start_pos = [0,0,0.05]
start_orient = pb.getQuaternionFromEuler([0,0,0.5])
robot_id, joint_ids, wheel_ids, proximity_sensors = aru.load_robot_URDF(sys.argv[1],start_pos,start_orient)

# Define the cube size and position
cube_size = 1.0  # 1 meter
cube_half_extents = [cube_size / 2] * 3  # Half extents for the collision shape
cube_position = [3, 0, cube_size / 2]  # Position the cube above the plane

# Create collision shape
collision_shape_id = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=cube_half_extents)

# Create visual shape
visual_shape_id = pb.createVisualShape(pb.GEOM_BOX, halfExtents=cube_half_extents, rgbaColor=[1, 0, 0, 1])  # Red color

# Create the cube
cube_id = pb.createMultiBody(baseMass=1,  # Mass of the cube
                             baseCollisionShapeIndex=collision_shape_id,
                             baseVisualShapeIndex=visual_shape_id,
                             basePosition=cube_position)

aru.set_joints_position(robot_id,joint_ids,[math.pi/4]*len(joint_ids),[0.5]*len(joint_ids))

sim_time = 0
sum_ctrl_freq = 0
while sim_time < 10:    
    #robot function refresh
    if(abs(sum_ctrl_freq - 0.1) < 0.001):
        sum_ctrl_freq = 0
        prox_outputs = [prox_sensor.detect() for prox_sensor in proximity_sensors]
        print(prox_outputs)
        
        current_joints_states = pb.getJointStates(robot_id,joint_ids)
        joint_positions = []
        for joint_state in current_joints_states:
            if joint_state[0] < -math.pi/4 + 0.01 and joint_state[0] > -math.pi/4 - 0.01:
                joint_positions.append(math.pi/4)
            elif joint_state[0] < math.pi/4 + 0.01 and joint_state[0] > math.pi/4 - 0.01:
                joint_positions.append(-math.pi/4)
        if(len(joint_positions) == len(joint_ids)):
            print(joint_positions)
            aru.set_joints_position(robot_id,joint_ids,joint_positions,[0.5]*len(joint_ids))
        aru.set_wheels_velocity(robot_id,wheel_ids,[10]*len(wheel_ids),10)
    
    #simulation refresh
    pb.stepSimulation() 
    time.sleep(0.02)
    sim_time += 0.02
    sum_ctrl_freq += 0.02

rob_pos, rob_orient = pb.getBasePositionAndOrientation(robot_id)
print(rob_pos,rob_orient)
pb.disconnect()