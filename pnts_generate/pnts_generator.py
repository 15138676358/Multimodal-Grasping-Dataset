#!/usr/bin/env python

import cv2
import rospy
import numpy as np
from numpy.fft import fft2, ifft2, fftshift, ifftshift
from plyfile import PlyData
from geometry_msgs.msg import PoseArray

class AprilTagPositionSubscriber:
    def __init__(self):
        # 初始化节点
        rospy.init_node('apriltag_position_subscriber', anonymous=True)
        
        # 订阅二维码位置话题
        self.pose_sub = rospy.Subscriber("/apriltag/poses", PoseArray, self.pose_callback)
        
        # 初始化点的列表和计数
        self.tag_positions = []
        self.tag_count = 0

    def pose_callback(self, data):
        # 清空之前的列表
        self.tag_positions = []
        
        # 更新点的计数
        self.tag_count = len(data.poses)
        
        # 打印所有二维码的位置，并存储到列表
        for i, pose in enumerate(data.poses):
            position = (pose.position.x, pose.position.y)
            self.tag_positions.append(position)
            rospy.loginfo("Tag {}: Position x: {}, y: {}".format(i, pose.position.x, pose.position.y))
        
        # 打印二维码的总数
        rospy.loginfo("Total number of tags: {}".format(self.tag_count))
        rospy.loginfo("Tag positions: {}".format(self.tag_positions))

def find_center_and_sample(num_points, points, step_size=0.1, threshold=0.2): # 采样步长 step_size，碰撞检测阈值 threshold
    distance_sums = np.zeros(num_points)
    if num_points == 2:
        sampled_points = []
        # to do 1
        p1, p2 = points[0], points[1]
        vector = p2 - p1
        distance = np.linalg.norm(vector)
        num_samples = int(distance // step_size)
        for i in range(0, num_samples + 1):
            sample_point = p1 + (vector * (i / num_samples))
            sampled_points.append(sample_point)
        return sampled_points
    else:
        # 计算每个点到其他点的距离总和
        for i in range(num_points):
            for j in range(num_points):
                if i != j:
                    distance_sums[i] += np.linalg.norm(points[i] - points[j])
        
        # 找到距离总和最小的点的索引
        center_index = np.argmin(distance_sums)
        center_point = points[center_index]
        
        # 在非中心点和中心点之间进行采样
        sampled_points = []
        for i in range(len(points)):
            if i != center_index:
                p1, p2 = points[i], center_point
                vector = p2 - p1
                distance = np.linalg.norm(vector)
                num_samples = int(distance // step_size)
                for i in range(0, num_samples + 1):
                    sample_point = p1 + (vector * (i / num_samples))
                    if np.linalg.norm(sample_point - center_point) < threshold:
                        break
                    sampled_points.append(sample_point)
        
        return sampled_points



if __name__ == '__main__':
    try:
        tag_subscriber = AprilTagPositionSubscriber()
        print(find_center_and_sample(tag_subscriber.tag_count, tag_subscriber.tag_positions))
        rospy.spin()
    except rospy.ROSInterruptException:
        pass