from math import radians

class speed_factor:
    
    def __init__(self, MAX_VELOCITY, TURN_VELOCITY, HIGH_ANGLE=radians(30), V_HIGH_ANGLE=radians(20), DECELARATION_POWER=0.6) -> None:
        """

        Args:
            MAX_VELOCITY (float): Desired velocity if no obstacle and car going straigth
            TURN_VELOCITY (float): Desired velocity when the car is turning with an angle teta>=V_HIGH_ANGLE
            HIGH_ANGLE (float): For turn angle teta>=HIGH_ANGLE, the car is slowing down from MAX_VELOCITY to TURN_VELOCITY
            V_HIGH_ANGLE (float): For turn angle teta>=V_HIGH_ANGLE is at speed TURN_VELOCITY
            DECELARATION_POWER (float): Constant used for the computation of deceleration based on distance
        """
        self.MAX_VELOCITY=MAX_VELOCITY
        self.TURN_VELOCITY=TURN_VELOCITY
        self.HIGH_ANGLE=HIGH_ANGLE
        self.V_HIGH_ANGLE=V_HIGH_ANGLE
        self.DECELARATION_POWER=DECELARATION_POWER
    
    def speed(self, angle, dist):
        """
        Computes desired speed

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
        factor_dist = dist**self.DECELARATION_POWER/(dist**self.DECELARATION_POWER+1)
        return self.MAX_VELOCITY * factor_angle * factor_dist
    