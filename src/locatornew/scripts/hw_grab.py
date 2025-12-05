import rospy
import tf
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion
from robot_controller import Robot


class hw_grab:
    def __init__(self):
        self.robot = Robot()
        self.tf_listener = tf.TransformListener()
        rospy.init_node('block_tf_listener', anonymous=True)
        self.target_frame = "world"
        self.source_frame = "block_top_center"
        self.arm = "S"
        self.speed = 0.1
        self.acce = 0.1
        self.offset = [0.0,0.0,0.0]

        self.end_pub = rospy.Publisher("isend", bool, queue_size=1)
        rospy.Subscriber("isbegin", bool, self.feedback)

    def get_block_tf(self):
        (trans, rot) = self.tf_listener.lookupTransform(
                    self.target_frame, 
                    self.source_frame, 
                    rospy.Time(0),
                    rospy.Duration(1.0)  # 超时1秒，避免卡死
                )
        x, y, z = trans
        qx, qy, qz, qw = rot
        point = [x, y, z]
        roll, pitch, yaw = euler_from_quaternion([qx, qy, qz, qw])
        pose = [point[0], point[1], point[2], roll, pitch, yaw]
        rospy.INFO(pose)
        self.robot.moveP(self.speed, self.acce, pose, self.arm, self.offset)
        self.end_pub.publish(True)

    def feedback(self, msg):
        rospy.sleep(1)
        self.get_block_tf()
    

if __name__ == '__main__':
    grab = hw_grab()
    rospy.spin()