#!/usr/bin/env python
import roslib
roslib.load_manifest('f1tenth_simulator')
import rospy



def main():
    rospy.init_node("wall_following", anonymous=True)
    rospy.spin()

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
