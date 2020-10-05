#!/usr/bin/python3

import rospy

from nav_msgs.msg import Odometry


class KalmanFilter:

    def __init__(self):
        rospy.init_node('kalman_filter')

        self.sub = rospy.Subscriber('odom', Odometry, self.on_odom, queue_size=10)
        self.pub = rospy.Publisher('odom_filtered', Odometry, queue_size=10)
    
    def run(self):
        rospy.spin()
    
    def on_odom(self, msg):
        rospy.loginfo(msg)
        self.pub.publish(msg)


def main():
    kalman_filter = KalmanFilter()
    kalman_filter.run()


if __name__ == '__main__':
    main()
