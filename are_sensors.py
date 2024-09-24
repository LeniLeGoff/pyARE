#! /bin/usr/python3
import pybullet as pb
import numpy as np

class ProximitySensor:
    ''' Class to represent a proximity sensor for a robot using raycasting.'''
    def __init__(self, robot_id, parent_id: int, origin_coord: list, origin_orient: list, ray_length: float = 0.2, ray_count: int = 5, elevation_segments: int = 3,cone_base_radius: float = 0.5):
        ''' Constructor for the ProximitySensor class. It initializes the sensor with raycasting parameters.'''
        self.parent_id = parent_id
        self.robot_id = robot_id
        
        parent_pos, parent_orient = pb.getLinkState(self.robot_id, self.parent_id)[:2]

        self.origin = parent_pos
        self.origin_orient = parent_orient

        self.ray_length = ray_length
        self.ray_count = ray_count
        self.elevation_segments = elevation_segments
        self.cone_base_radius = cone_base_radius

        # Get the parent link's position and orientation

        # # Transform the origin to the parent link frame
        # self.origin_world_pos, self.origin_world_orient = pb.multiplyTransforms(parent_pos, parent_orient, self.origin, self.origin_orient)

    def detect(self):
        ''' Returns the closest object detected by the sensor using raycasting.'''
        parent_pos, parent_orient = pb.getLinkState(self.robot_id, self.parent_id)[:2]
        self.origin = parent_pos
        self.origin_orient = parent_orient

        # Clear previous debug lines
        pb.removeAllUserDebugItems()
    
        ray_starts = []
        ray_ends = []

        # Cast rays in a conical pattern around the origin
        for e in range(self.elevation_segments):
            elevation_angle = (np.pi / 2) * ((e+1) / (self.elevation_segments - 1))  # Elevation angle from 0 to 90 degrees
            for i in range(self.ray_count):
                azimuth_angle = (2 * np.pi / self.ray_count) * (i+1)  # Azimuth angle around the z-axis
                local_direction = [
                    self.cone_base_radius * np.cos(elevation_angle) * np.cos(azimuth_angle),
                    self.cone_base_radius * np.cos(elevation_angle) * np.sin(azimuth_angle),
                    -self.ray_length
                ]
                # Transform the local direction to world coordinates
                direction = pb.multiplyTransforms([0, 0, 0], self.origin_orient, local_direction, [0, 0, 0, 1])[0]
                ray_end = [self.origin[0] + direction[0], self.origin[1] + direction[1], self.origin[2] + direction[2]]
                
                ray_starts.append(self.origin)
                ray_ends.append(ray_end)
                #pb.addUserDebugLine(self.origin, ray_end, lineColorRGB=[0, 1, 0], lineWidth=1.0)


                # Perform ray test
        ray_results = pb.rayTestBatch(ray_starts, ray_ends)#
        distance = 0
        min_distance = 1000
        hit_position = [0, 0, 0]
        for ray_result in ray_results:
            hit_object_id, hit_fraction = ray_result[0], ray_result[2]
            
            if hit_object_id != -1:
                distance = hit_fraction * self.ray_length
                if distance < min_distance:
                    min_distance = distance
                    hit_position = ray_result[3]

                # Add debug line for visualization
        if(min_distance < 1000):
            pb.addUserDebugLine(self.origin, hit_position, lineColorRGB=[1, 0, 0], lineWidth=1.0)

        return min_distance