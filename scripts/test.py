#!/usr/bin/env python3
import rospy
from std_msgs.msg import Header
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from lio_sam.msg import context_info  # Replace with your actual message type

class PathPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('path_publisher', anonymous=True)

        # Create publishers for each robot's Path message
        self.path_pub_robot1 = rospy.Publisher('/robot1_path', Path, queue_size=10)
        self.path_pub_robot2 = rospy.Publisher('/robot2_path', Path, queue_size=10)
        self.path_pub_rover1 = rospy.Publisher('/rover1_path', Path, queue_size=10)

        # Subscribe to context_info messages from both robots
        rospy.Subscriber('/robot1/solid/context_info', context_info, self.callback_robot1)
        rospy.Subscriber('/robot2/solid/context_info', context_info, self.callback_robot2)
        rospy.Subscriber('/rover1/solid/context_info', context_info, self.callback_rover1)

        # Initialize Path messages for both robots
        self.path_robot1 = Path()
        self.path_robot1.header.frame_id = "map"  # Set the frame_id (e.g., "map")

        self.path_robot2 = Path()
        self.path_robot2.header.frame_id = "map"  # Set the frame_id (e.g., "map")

        self.path_rover1 = Path()
        self.path_rover1.header.frame_id = "map"  # Set the frame_id (e.g., "map")

    def add_pose_to_path(self, path, pose_x, pose_y, pose_z):
        # Create a PoseStamped message
        pose_stamped = PoseStamped()
        pose_stamped.header = Header()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose.position.x = pose_x
        pose_stamped.pose.position.y = pose_y
        pose_stamped.pose.position.z = pose_z
        pose_stamped.pose.orientation.x = 0.0  # Default orientation (modify as needed)
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = 0.0
        pose_stamped.pose.orientation.w = 1.0

        # Append the new pose to the Path
        path.header.stamp = rospy.Time.now()
        path.poses.append(pose_stamped)

    def callback_robot1(self, msg):
        # Extract pose from robot1 and add to its path
        self.add_pose_to_path(self.path_robot1, msg.poseX, msg.poseY, msg.poseZ)

        # Publish robot1's Path
        self.path_pub_robot1.publish(self.path_robot1)

    def callback_robot2(self, msg):
        # Extract pose from robot2 and add to its path
        self.add_pose_to_path(self.path_robot2, msg.poseX, msg.poseY, msg.poseZ)

        # Publish robot2's Path
        self.path_pub_robot2.publish(self.path_robot2)

    def callback_rover1(self, msg):
        # Extract pose from robot2 and add to its path
        self.add_pose_to_path(self.path_rover1, msg.poseX, msg.poseY, msg.poseZ)

        # Publish robot2's Path
        self.path_pub_rover1.publish(self.path_rover1)


if __name__ == '__main__':
    try:
        path_publisher = PathPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
