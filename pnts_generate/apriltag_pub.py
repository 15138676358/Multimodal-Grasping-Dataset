#!/usr/bin/env python

import rospy
import cv2
import apriltag
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge, CvBridgeError

class AprilTagDetector:
    '''
    读取并且发布相片中的多个apriltag位置坐标
    '''
    def __init__(self):
        # 初始化节点
        rospy.init_node('apriltag_detector', anonymous=True)
        
        # 创建订阅和发布
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)
        self.pose_pub = rospy.Publisher("/apriltag/poses", PoseArray, queue_size=10)
        
        # 图像转换、Apriltag检测
        self.bridge = CvBridge()
        self.detector = apriltag.Detector()

    def image_callback(self, data):
        try:
            # 将ROS图像消息转换为OpenCV图像
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        
        # 转换为灰度图像、Apriltag检测
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        tags = self.detector.detect(gray)
        
        poses = PoseArray()
        poses.header.stamp = rospy.Time.now()
        poses.header.frame_id = "camera"
        
        for tag in tags:
            pose = Pose()
            pose.position.x = tag.center[0]
            pose.position.y = tag.center[1]
            pose.position.z = tag.center[2]
            
            poses.poses.append(pose) # PoseArray
        
        # 发布PoseArray
        self.pose_pub.publish(poses)

if __name__ == '__main__':
    try:
        AprilTagDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
