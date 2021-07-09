#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from dUAV_dock.msg import Center  # 自定义消息类型
import os
import cv2
import numpy as np
import time
import dt_apriltags
import math


class Track:

    def __init__(self):
        self.at_detector_48 = dt_apriltags.Detector(families='tagCustom48h12',
                                            nthreads=4,
                                            quad_decimate=2,
                                            quad_sigma=0.0,
                                            refine_edges=1,
                                            decode_sharpening=0.25,
                                            debug=0)
        self.at_detector_36 = dt_apriltags.Detector(families='tag36h11',
                                                    nthreads=4,
                                                    quad_decimate=2,
                                                    quad_sigma=0.0,
                                                    refine_edges=1,
                                                    decode_sharpening=0.25,
                                                    debug=0)
        rospy.Subscriber('iris/usb_cam/image_raw', Image, self.callback)
        self.center_publish = rospy.Publisher('/center', Center, queue_size=1)  # 发布矩形中心

    def callback(self, image):
        img = np.fromstring(image.data, np.uint8)
        img = img.reshape(480, 640, 3)
        self.find(img, image.width, image.height)  # 寻找中心

    def find(self, frame, width, height):
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags_48 = self.at_detector_48.detect(gray)
        tags_36 = self.at_detector_36.detect(gray)
        # print("%d apriltags have been detected."%len(tags))
        if tags_48:
            for tag in tags_48:
                circle_color = (0, 255, 0)
                center_x = int(tag.center[0])
                center_y = int(tag.center[1])
                centerxy = (center_x, center_y)
                self.draw(frame, circle_color, center_x, center_y, tag)
                self.publish(centerxy, width, height)
        elif tags_36:
            for tag in tags_36:
                circle_color = (0, 255, 0)
                center_x = int(tag.center[0])
                center_y = int(tag.center[1])
                centerxy = (center_x, center_y)
                self.draw(frame, circle_color, center_x, center_y, tag)
                self.publish(centerxy, width, height)
        else:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            lower_red = np.array([0, 100, 100])
            upper_red = np.array([10, 255, 255])
            mask = cv2.inRange(hsv, lower_red, upper_red)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            img, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                maxArea = 0
                maxIndex = 0
                for i, c in enumerate(contours):
                    # 舍弃小区域
                    if (cv2.contourArea(contours[i]) < 500):
                        continue
                    # 找出面积最大的区域
                    area = cv2.contourArea(c)
                    if area > maxArea:
                        maxArea = area
                        maxIndex = i
                x, y, w, h = cv2.boundingRect(contours[maxIndex])  # 外接四边形
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
                center_x = int(x + w / 2)
                center_y = int(y + h / 2)
                cv2.circle(frame, (center_x, center_y), 8, (0, 255, 0), -1)  # center
                centerxy = (center_x, center_y)
                self.red_publish(centerxy, width, height)
            else:
                marker_center = Center()
                marker_center.redfind = False
                marker_center.apfind = False
                self.center_publish.publish(marker_center)
        cv2.imshow('w', frame)
        cv2.waitKey(1)

    def compute(self, halflength, tag):
        cameraParams_Intrinsic = [554.382713, 554.382713, 320, 240]  # camera_fx, camera_fy, camera_cx, camera_cy
        camera_matrix = np.array(([554.382713, 0, 640],
                                  [0, 554.382713, 480],
                                  [0, 0, 1.0]), dtype=np.double)
        center_x = int(tag.center[0])
        center_y = int(tag.center[1])

        # PnP解位姿
        object_3d_points = np.array(
            ([-halflength, halflength, 0],
             [halflength, halflength, 0],
             [halflength, -halflength, 0],
             [-halflength, -halflength, 0]),
            dtype=np.double)  # Apriltag coordinates in the World coordinate system
        object_2d_point = np.array(
            (tag.corners[0].astype(int),
             tag.corners[1].astype(int),
             tag.corners[2].astype(int),
             tag.corners[3].astype(int)),
            dtype=np.double)  # Apriltag coordinates in the Image pixel system
        dist_coefs = np.array([0, 0, 0, 0, 0], dtype=np.double)
        found, rvec, tvec = cv2.solvePnP(object_3d_points, object_2d_point, camera_matrix, dist_coefs)
        rotM = cv2.Rodrigues(rvec)[0]
        camera_postion = -np.matrix(rotM).T * np.matrix(tvec)

        # 求出来相机系到二维码系的欧拉角
        thetaZ = math.atan2(rotM[1, 0], rotM[0, 0]) * 180.0 / math.pi
        thetaY = math.atan2(-1.0 * rotM[2, 0], math.sqrt(rotM[2, 1] ** 2 + rotM[2, 2] ** 2)) * 180.0 / math.pi
        thetaX = math.atan2(rotM[2, 1], rotM[2, 2]) * 180.0 / math.pi
        # 目前求出的x,y不是很准，先不用，z还算准
        center_x_pnp = tvec[0]
        center_y_pnp = tvec[1]
        center_z_pnp = tvec[2]
        x_pnp = -1 * center_y_pnp
        y_pnp = -1 * center_x_pnp
        z_pnp = 1 * center_z_pnp

        # 乘这个负数，是因为相机系Z轴朝下，机体系Z轴朝上，所以角度正方向相反
        center_yaw = -1.0 * thetaZ
        centers = (center_x, center_y, center_yaw)
        return float(center_z_pnp), centers

    def draw(self, frame, circle_color, center_x, center_y, tag):
        cv2.circle(frame, (center_x, center_y), 8, circle_color, -1)  # center
        for idx in range(len(tag.corners)):
            cv2.line(frame, tuple(tag.corners[idx - 1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)),
                     circle_color, 8)

    # 发布中心点消息
    def red_publish(self, centers, width, height):
        marker_center = Center()
        marker_center.width = width
        marker_center.height = height
        marker_center.x = centers[0]
        marker_center.y = centers[1]
        marker_center.redfind = True
        marker_center.first_find = True
        self.center_publish.publish(marker_center)

    def publish(self, centers, width, height):
        marker_center = Center()
        marker_center.width = width
        marker_center.height = height
        marker_center.x = centers[0]
        marker_center.y = centers[1]
        marker_center.apfind = True
        marker_center.first_find = True
        self.center_publish.publish(marker_center)


if __name__ == '__main__':
    rospy.init_node('track', anonymous=True)
    print("Tracking start")
    track = Track()
    rospy.spin()