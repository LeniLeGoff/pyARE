#! /bin/usr/python3

import pybullet as pb
import math
import are_sensors as sensors

def load_robot_URDF(file_path: str, 
             start_position: list, 
             start_orientation: list):
    ''' load urdf file of ARE robot
        Arguments:
            Path to the urdf file of the robot
            starting position and orientation in world frame
        Return:
            robot id
            list of the joints ids
            list of the wheel ids
            list of the proximity sensors ids
            list of the IR sensors ids
            '''
    robot_id = pb.loadURDF(file_path,start_position, start_orientation)
    joint_ids = []
    wheel_ids = []
    proximity_senors = []
    ir_sensors = [] #todo
    for joint_id in range( pb.getNumJoints(robot_id)):
        joint_info = pb.getJointInfo(robot_id,joint_id)
        joint_name = joint_info[1].decode("utf-8")
        link_name = joint_info[12].decode("utf-8")
        if(joint_name.split("t")[0] == "Join"):
            pb.changeDynamics(robot_id,joint_info[0],
                         jointLowerLimit=-math.pi/2,
                         jointUpperLimit=math.pi/2)
            joint_ids.append(joint_id)
        elif(joint_name.split("_")[0] == "Motor"):
            wheel_ids.append(joint_id)
        elif(link_name.split("r")[0] == "P_Senso"):
            print(joint_info)
            proximity_senors.append(sensors.ProximitySensor(robot_id,joint_id,joint_info[14],1.0,10))
            
    return robot_id, joint_ids, wheel_ids, proximity_senors

def set_joint_position(robot_id, 
                       joint_id: int, 
                       position: float, 
                       max_velocity: float):
    ''' set joint target position with a max velocity 
        robot: pybullet body id
        id: id of the joint
        position: target position
        max velocity'''
    pb.setJointMotorControl2(robot_id,joint_id,controlMode=pb.POSITION_CONTROL,
                             targetPosition=position,maxVelocity=max_velocity)

def set_joints_position(robot_id,
                        joint_ids: list,
                        positions: list,
                        max_forces: float):
    ''' set joints target position with a max velocity '''
    pb.setJointMotorControlArray(robot_id,joint_ids,controlMode=pb.POSITION_CONTROL,
                                 targetPositions=positions,forces=max_forces)


def set_wheel_velocity(robot_id , 
                       wheel_id: int, 
                       velocity: float, 
                       max_force: float):
    ''' set wheel target velocity with max force '''
    pb.setJointMotorControl2(robot_id,wheel_id,controlMode=pb.VELOCITY_CONTROL,
                            targetVelocity=velocity,force=max_force)


def set_wheels_velocity(robot_id,
                        wheels_ids: list,
                        velocities: list,
                        max_force: float):
    ''' set wheels target velocity with max force '''
    pb.setJointMotorControlArray(robot_id,wheels_ids,controlMode=pb.VELOCITY_CONTROL,
                                 targetVelocities=velocities,forces=[max_force]*len(velocities))