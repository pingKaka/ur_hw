import rospy
from fake_command import run

class hw_move:
    def __init__(self):
        self.speed = 0.1
        self.acce = 0.1
        self.resetpoint = "reset"

        self.begin_pub = rospy.Publisher("isbegin", bool, queue_size=1)
        rospy.Subscriber("isend", bool, self.feedback)

    def reset(self):
        rospy.sleep(0.5)
        run("A S PTP J "+f"{self.speed} {self.acce} {self.resetpoint}")
        rospy.INFO("Reset Completed")
        self.begin_pub.publish(True)

    def place(self):
        rospy.sleep(0.5)
        run("actiongroup")
        rospy.INFO("Place Completed")

    def feedback(self,msg):
        self.place()
        self.reset()

if __name__ == '__main__':
    move = hw_move()
    move.reset()
    rospy.spin()