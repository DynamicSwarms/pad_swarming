import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.time import Time, Duration


from pad_management_interfaces.srv import PadCircleBehaviour


from dataclasses import dataclass

from typing import Dict

import numpy as np
from rcl_interfaces.msg import (
    ParameterDescriptor,
    SetParametersResult,
    FloatingPointRange,
)

import math

@dataclass
class Agent: 
    name: str
    position: "list[float]"
    last_time: Time

NUM_FORMATION_POINTS = 32

class PadLandCircle(Node):

    def __init__(self):
        super().__init__("pad_circle")
        self.declare_parameter(
            "radius",
            value=1.75,
            descriptor=ParameterDescriptor(
                floating_point_range=[
                    FloatingPointRange(from_value=0.5, to_value=3.0),
                ]
            ),
        )

        self.declare_parameter(
            "seperation_factor",
            value=0.5,
            descriptor=ParameterDescriptor(
                floating_point_range=[
                    FloatingPointRange(from_value=0.0, to_value=1.0),
                ]
            ),
        )

        self.declare_parameter(
            "circular_speed",
            value=0.2,
            descriptor=ParameterDescriptor(
                floating_point_range=[
                    FloatingPointRange(from_value=0.0, to_value=1.0),
                ]
            ),
        )



        self.agents : Dict[str, Agent] = {}

        self.callback_group = MutuallyExclusiveCallbackGroup()
        
        self.create_service(srv_type=PadCircleBehaviour, srv_name="pad_circle", callback=self.calculate_behaviour, callback_group=self.callback_group)
        self.create_timer(0.2, self.clear_dict)

        self.clockwise = True

    def clear_dict(self):
        decay_duration = Duration(seconds=0, nanoseconds=0.3 * 1e9) 
        now = self.get_clock().now()
        decay_time = now - decay_duration

        for name in list(self.agents.keys()):
            if self.agents[name].last_time < decay_time: 
                del self.agents[name]



    def calculate_behaviour(self, request: PadCircleBehaviour.Request, response: PadCircleBehaviour.Response):
        name = request.name
        position = [request.position.x, request.position.y, request.position.z]
        now = self.get_clock().now()

        if name in self.agents.keys(): 
            self.agents[name].position = position
            self.agents[name].last_time = now
        else: 
            self.agents[name] = Agent(name=name, position=position, last_time=now)


        response.target.x, response.target.y, response.target.z = self.match_formation(name)
        return response
     
    def match_formation(self, name: str):
        """
        Matches a name (an agent) to our formation	
            
        Uses: formation: list of points on shape (its best to have 32 or more), needs to be closed shape best with no intersections
              agents: list of agents with positions
              name:   an agent name to calculate for
        Idea:  Estimates "angle" in the shape for each copter (linear interpolate of two closest)
                Adjust Angle of copter for no collisions and circular movement
                Interpolate between the closest two points for the final position
        Returns: A target for the copter with the given name
            
        """
        formation = [np.array(point) for point in self.formation]
        positions = [np.array(agent.position) for agent in self.agents.values()]
        
        SEPERATION_FACTOR = self.seperation_factor 
        CIRCULAR_SPEED = self.circular_speed 
        if not self.clockwise: CIRCULAR_SPEED *= -1

        CNT = len(formation)
        STEP_SIZE = np.pi * 2 /CNT
        STEPS = [STEP_SIZE * i for i in range(CNT)]


        angles = []     #Estimated angle of each crazyflie in the shape 
        unit_pos = []   #Position on unit_circle according to the angle
        
        for i, cf_pos in enumerate(positions): 
            # Estimate angle in shape
            dst = [np.linalg.norm(cf_pos - pos) for pos in formation] #Distances of Crazyflie to all formation points
            closest = np.argmin(dst) # closest formation point index
            second = ((closest + 1) % CNT) if dst[(closest + 1)%CNT] <= dst[(closest - 1)%CNT] else (closest - 1)%CNT # closest neighbour index of closest
            # We could have used the two closest, the hope is to have better stability on intersection points

            angles.append(self.circular_interpolate(STEPS[closest], STEPS[second], dst[closest], dst[second])) # The shape angle we have
            unit_pos.append(np.array([np.sin(angles[i]), np.cos(angles[i])])) #only needed for angle difference calculation
        
        # Get index of agent to match to calculate for
        i = list(self.agents.keys()).index(name)

        # Adjust Angle and calculate Target
        a_dfrcs = [] # Angular difference to other crazyflies -> used for seperation calculation
        for j, _cf_pos in enumerate(positions):
            if i == j: continue
            angle = self.get_angle(unit_pos[i], unit_pos[j]) # 0 ... 2 pi
            if angle > np.pi:
                angle = - (2 * np.pi - angle) 
            a_dfrcs.append(angle) ## -pi ... pi 

        separation_urge = 0
        if len(positions) != 1:
            clst_neg =  min([ a      if a > 0 else np.pi for a in a_dfrcs ])
            clst_pos = -min([ abs(a) if a < 0 else np.pi for a in a_dfrcs ])
            separation_urge = (clst_pos + clst_neg) * SEPERATION_FACTOR            
        
        #         old_pos          move         seperation
        angle =  angles[i] + CIRCULAR_SPEED +  separation_urge
        dif = [abs(self.abs_angle_diff(angle, step)) for step in STEPS] #Anglular difference to all available points on shape
        i, j = np.argsort(dif)[:2]  # index of smallest and second smallest angular difference
        
        #interpolate between the nearest two points now in 3d
        p = ( 1/(dif[i]+dif[j]) ) * (formation[i] * dif[j] + formation[j] * dif[i])
        
        return [p[0], p[1], p[2]] #3d


    @property
    def formation(self) -> "list[float]":
        radius = self.get_parameter("radius").get_parameter_value().double_value
        for a in range(NUM_FORMATION_POINTS):
            v = math.radians(360.0 / float(NUM_FORMATION_POINTS) * a)
            yield [radius * math.sin(v), radius * math.cos(v), 0.0]
    
    @property
    def seperation_factor(self) -> float: 
        return self.get_parameter("seperation_factor").get_parameter_value().double_value
    
    @property
    def circular_speed(self) -> float: 
        return self.get_parameter("circular_speed").get_parameter_value().double_value
            
    def circular_interpolate(self, a_0, a_1, w_0, w_1):
        """ 
        interpolates an angle between a_0 and a_1, which is circular between 0 and 2 pi
        the weights (w) the interpolation factors smaller factor -> closer to the given angle
        """
        s = w_0 + w_1
        f_0 = w_0 / s
        f_1 = w_1 / s

        e_1 = 1 / s * (np.sin(a_0) * f_1 + np.sin(a_1) * f_0)
        e_2 = 1 / s * (np.cos(a_0) * f_1 + np.cos(a_1) * f_0)
        ang = np.arctan2(e_1, e_2)
        return ang if ang > 0 else np.pi * 2 - abs(ang)
        ## mean angle see here: https://rosettacode.org/wiki/Averages/Mean_angle

    def get_angle(self, v, w):
        """
        Returns the angle between two vectors in clockwise direction
        In radians 0 ...  2 * pi

        Calculates the dot product between v and w -> angle between 0 and pi
        In order to get direction, we calculate the dotproduct between a 90° clockwise rotated v and w
        """
        l_v, l_w = np.linalg.norm(v), np.linalg.norm(w)
        if l_v == 0 or l_w == 0: return 0
        norm_v = v / l_v
        norm_w = w / l_w
        dot = np.dot(norm_v, norm_w)
        angle = np.arccos(min(1, dot))  # betw. 0 and 180 ° (0 and pi)

        _rad_ang = np.deg2rad(90)
        rot_matrix = np.array([[np.cos(_rad_ang), -np.sin(_rad_ang)], [np.sin(_rad_ang), np.cos(_rad_ang)]])
        rot_w = np.dot(rot_matrix, norm_w)  # rotated by 90 deg left
        dot = np.dot(rot_w, norm_v)
        angle_2 = np.arccos(min(1, dot))

        if angle_2 < np.pi / 2:
            return angle  # angle + np.pi
        else:
            return 2 * np.pi - angle  # np.pi -  angle
        
    def abs_angle_diff(self, a, b):
        """
        Returns the absolute difference of two angles, accounting for circular bahaviour
        """
        d = abs(a - b)
        if d < np.pi:
            return d
        else:
            return 2 * np.pi - d



def main(): 
    rclpy.init()

    beh = PadLandCircle()
    try: 
        rclpy.spin(beh)
        rclpy.try_shutdown()
    except KeyboardInterrupt: 
        rclpy.try_shutdown()
        exit()
if __name__ =="__main__":
    main()
