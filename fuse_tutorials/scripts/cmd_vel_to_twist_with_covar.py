#!/usr/bin/env python
# Lucas Walter
# turn a cmd_vel Twist into a TwistWithCovarianceStamped

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistWithCovarianceStamped


class TwistConvert(object):
    def __init__(self):
        self.frame_id = rospy.get_param('~frame_id', 'base_link')
        # TODO(lucasw) dynamic reconfigure
        self.covar = rospy.get_param('~covar', 0.01)
        self.pub = rospy.Publisher('twist_with_covar', TwistWithCovarianceStamped, queue_size=3)
        self.sub = rospy.Subscriber('cmd_vel', Twist, self.callback, queue_size=3)

    def callback(self, twist):
        msg = TwistWithCovarianceStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = rospy.Time.now()
        msg.twist.twist = twist

        for i in range(len(msg.twist.covariance)):
            msg.twist.covariance[i] = 0.0

        for i in range(6):
            msg.twist.covariance[i * 6 + i] = self.covar

        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('twist_convert')
    node = TwistConvert()
    rospy.spin()
