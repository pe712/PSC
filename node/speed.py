from math import radians, log

class speed_computation:
    """
    Compute speed to prevent to go the fastest possible while avoiding emergency brakes. 
    """
    def __init__(self, max_velocity, turn_velocity, deceleration_dist=1.0, high_angle=radians(30), v_high_angle=radians(20), deceleration_power=0.5):
        """
        Args:
            MAX_VELOCITY (float): Desired velocity if no obstacle and car going straigth
            TURN_VELOCITY (float): Desired velocity when the car is turning with an angle teta>=V_HIGH_ANGLE
            HIGH_ANGLE (float): For turn angle teta>=HIGH_ANGLE, the car is slowing down from MAX_VELOCITY to TURN_VELOCITY
            V_HIGH_ANGLE (float): For turn angle teta>=V_HIGH_ANGLE is at speed TURN_VELOCITY
            DECELARATION_POWER (float): Constant used for the computation of deceleration based on distance
        """
        # data from test:
        self.REF_MAX_VELOCITY = 2.5 # for deceleration_power = 0.5
        self.MAX_VELOCITY=max_velocity
        self.TURN_VELOCITY=turn_velocity
        self.HIGH_ANGLE=high_angle
        self.V_HIGH_ANGLE=v_high_angle
        self.DECELERATION_DIST = deceleration_dist
        self.DECELARATION_POWER=deceleration_power
    
    def speed(self, angle, dist):
        """
        Computes desired speed. 
        
        The racecar slows down when steering and when it is close to an obstacle. 

        Args:
            angle (float): turn angle (radians)
            dist (float): distance to goal = distance measured by lidar in direction angle

        Returns:
            float: desired velocity
        """
        if (abs(angle)<radians(20)):
            factor_angle = 1
        elif (abs(angle)>radians(30)):
            factor_angle = self.TURN_VELOCITY/self.MAX_VELOCITY
        else:
            # angle is between 20 and 30, affine function
            factor_angle = 1 + (abs(angle)-radians(20))/(radians(30)-radians(20))*(-1+self.TURN_VELOCITY/self.MAX_VELOCITY)
        factor_dist = self.distance_factor(dist)
        return min(self.REF_MAX_VELOCITY * factor_angle * factor_dist, self.MAX_VELOCITY * factor_angle)

    def distance_factor(self, dist):
        normalized_dist = dist/self.DECELERATION_DIST
        return normalized_dist**self.DECELARATION_POWER 
