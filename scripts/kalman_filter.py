#!/usr/bin/python


import rospy
import numpy as np
from numpy.linalg import inv
from nav_msgs.msg import Odometry


class KalmanFilter:

    def __init__(self):
        rospy.init_node('kalman_filter')
        self.sub = rospy.Subscriber('/husky_velocity_controller/odom', Odometry, self.on_odom, queue_size=10)
        self.pub = rospy.Publisher('odom_filtered', Odometry, queue_size=10)

    def predict(self, X_t1, P_t1, F_t, Q_t ):
        X_t = F_t.dot(X_t1)
        # P_t = np.diag(np.diag(F_t.dot(P_t1).dot(F_t.transpose()))) + Q_t
        P_t = F_t.dot(P_t1).dot(F_t.transpose()) + Q_t
        return X_t, P_t

    def update(self, X_t, P_t, Z_t, R_t, H_t):
        K = P_t.dot(H_t.transpose()).dot(np.linalg.inv(H_t.dot(P_t).dot(H_t.transpose()) + R_t))
        print(K.shape)
        X_t = X_t + K.dot(Z_t - H_t.dot(X_t))
        P_t = P_t - K.dot(H_t).dot(P_t)
        return X_t, P_t


    def run(self):
        rospy.spin()

    def on_odom(self, msg):
        X = np.array([[0], [0], [0], [1], [1], [1]])
        Z = np.array([[msg.pose.pose.position.x], [msg.pose.pose.position.y], [msg.pose.pose.position.z],
                      [msg.pose.pose.orientation.x], [msg.pose.pose.orientation.y], [msg.pose.pose.orientation.z]])
        P = np.array(msg.pose.covariance).reshape(6, 6)
        F = np.array([[1, 0, 0.05, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0],
                     [0, 0, 1, 0, 0, 0],
                     [0, 0, 0, 1, 0, 0],
                     [0, 0, 0, 0, 1, 0],
                     [0, 0, 0, 0, 0, 1]])
        Q = np.identity(6)
        H = np.eye(6)
        R = np.identity(6) * 0.2
        X_t, P_t = self.predict(X, P, F, Q)
        X_t, P_t = self.update(X_t, P_t, Z, R, H)

        msg.pose.pose.position.x = X_t[0, 0]
        msg.pose.pose.position.y = X_t[1, 0]
        msg.pose.pose.position.z = X_t[2, 0]
        msg.pose.pose.orientation.x = X_t[3, 0]
        msg.pose.pose.orientation.y = X_t[4, 0]
        msg.pose.pose.orientation.z = X_t[5, 0]
        msg.pose.covariance = list(np.reshape(P_t, 36))
        rospy.loginfo(msg)
        self.pub.publish(msg)

        return X_t, P_t


def main():
    kalman_filter = KalmanFilter()
    kalman_filter.run()


if __name__ == '__main__':
    main()
