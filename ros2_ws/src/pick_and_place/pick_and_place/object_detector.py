#!/usr/bin/env python3

"""
Detects objects based on their color and provides information 
about their position, dimensions and color.

Author: Elena Oikonomou (adapted to ROS2)
Date:   Fall 2023 / 2025
"""

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import tf_transformations

from gazebo_msgs.srv import GetEntityState
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import Image, CameraInfo
from typing import List, Tuple

from pick_and_place_msgs.msg import DetectedObjectsStamped, DetectedObject


class VisionObjectDetector(Node):
    def __init__(self):
        super().__init__('vision_object_detector')
        
        self.color_ranges = {"blue": [np.array([110,50,50]), np.array([130,255,255])],
                             "green": [np.array([36, 25, 25]), np.array([70, 255,255])],
                             "red": [np.array([0, 100, 100]), np.array([10, 255, 255])],
                             "black": [np.array([0,0,0]), np.array([180, 255, 40])]
                             }                                              # Color ranges in HSV
        self.block_contour_area_threshold = 200                             # Area threshold to detect blocks
        self.blocks_on_workbench = []                                       # Blocks on the workbench
        
        self.T_c_w, self.T_w_c = self.get_camera_homogeneous_transforms()  # Camera transforms
        self.bridge = CvBridge()                                            # ROS-OpenCV bridge
        self.image_height, self.image_width = self.get_image_dimensions()
        self.pin_cam = self.get_pinhole_camera_model()                      # Pinhole camera model
        self.depth_image = self.get_depth_image()
        self.workbench_depth = self.get_workbench_depth()
        
        # Subscribers and publishers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10)
        
        self.detected_objects_pub = self.create_publisher(
            DetectedObjectsStamped,
            '/object_detection',
            10)
        
        # Service client for Gazebo
        self.get_entity_state_client = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        while not self.get_entity_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /gazebo/get_entity_state service...')
    
    def get_image_dimensions(self) -> Tuple[int, int]:
        """Computes the image height and width in pixels."""
        image = self.get_color_image()
        h, w, c = image.shape
        return h, w

    def get_camera_homogeneous_transforms(self) -> Tuple[np.ndarray, np.ndarray]:
        """Returns the homogeneous transform of the camera frame wrt world frame and its inverse."""
        camera_origin = self.get_model_position_from_gazebo("kinect")
        Rot_c_w = np.array([[ 0, -1,  0, 0],
                            [-1,  0,  0, 0],
                            [ 0,  0, -1, 0],
                            [ 0,  0,  0, 1]])
        Transl_c_w = tf_transformations.translation_matrix(camera_origin)
        T_c_w = np.dot(Transl_c_w, Rot_c_w)
        T_w_c = tf_transformations.inverse_matrix(T_c_w)
        return T_c_w, T_w_c
        
    def get_depth_image(self) -> np.ndarray:
        """Returns current depth image in OpenCV format."""
        try:
            image_msg = None
            def depth_callback(msg):
                nonlocal image_msg
                image_msg = msg
            
            sub = self.create_subscription(Image, '/camera/depth/image_raw', depth_callback, 10)
            rclpy.spin_once(self, timeout_sec=10.0)
            self.destroy_subscription(sub)
            
            if image_msg:
                image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='32FC1')
                return image
            return np.zeros((self.image_height, self.image_width))
        except Exception as e:
            self.get_logger().error(f'Failed to get depth image: {e}')
            return np.zeros((480, 640))
    
    def get_color_image(self) -> np.ndarray:
        """Returns current color image in OpenCV format."""
        try:
            image_msg = None
            def color_callback(msg):
                nonlocal image_msg
                image_msg = msg
            
            sub = self.create_subscription(Image, '/camera/color/image_raw', color_callback, 10)
            rclpy.spin_once(self, timeout_sec=10.0)
            self.destroy_subscription(sub)
            
            if image_msg:
                image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
                return image
            return np.zeros((480, 640, 3), dtype=np.uint8)
        except Exception as e:
            self.get_logger().error(f'Failed to get color image: {e}')
            return np.zeros((480, 640, 3), dtype=np.uint8)

    def get_pinhole_camera_model(self) -> PinholeCameraModel:
        """Creates a pinhole camera model from the camera's ROS parameters."""
        pin_cam = PinholeCameraModel()
        try:
            cam_info = None
            def info_callback(msg):
                nonlocal cam_info
                cam_info = msg
            
            sub = self.create_subscription(CameraInfo, '/camera/color/camera_info', info_callback, 10)
            rclpy.spin_once(self, timeout_sec=10.0)
            self.destroy_subscription(sub)
            
            if cam_info:
                pin_cam.fromCameraInfo(cam_info)
        except Exception as e:
            self.get_logger().error(f'Failed to get camera info: {e}')
        
        return pin_cam

    def get_model_position_from_gazebo(self, model: str) -> Tuple[float, float, float]:
        """Returns the position of the model wrt the world frame."""
        request = GetEntityState.Request()
        request.name = model
        request.reference_frame = "world"
        
        try:
            future = self.get_entity_state_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            response = future.result()
            if response and response.success:
                return (response.state.pose.position.x,
                       response.state.pose.position.y,
                       response.state.pose.position.z)
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
        
        return (0.0, 0.0, 1.5)  # Default camera position

    def get_mask(self, hsv: np.ndarray, color: str) -> np.ndarray:
        """Creates a mask of the image that detects the given color."""
        if color not in self.color_ranges.keys():
            raise ValueError('The requested color to mask is not on the list of detectable colors.')
        
        mask = cv2.inRange(hsv, self.color_ranges[color][0], self.color_ranges[color][1])
        return mask

    def get_workbench_depth(self) -> float:
        """Computes the depth of the workbench."""
        image = self.get_color_image()
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = self.get_mask(hsv, "black")
        
        height, width = mask.shape
        for i in range(height):
            for j in range(width):
                if mask[i, j] == 255:
                    depth = self.get_pixel_depth(i, j)
                    return depth
        
        return 1.0  # Default depth

    def get_pixel_depth(self, u: float, v: float) -> float:
        """Returns the value of the pixel in the depth image."""
        if u >= self.image_width:
            u = self.image_width - 1
        if v >= self.image_height:
            v = self.image_height - 1
        
        return self.depth_image[int(v), int(u)]
        
    def compute_mass_center(self, image_array: np.ndarray) -> Tuple[int, int]:
        """Computes the center of mass of the image."""
        M = cv2.moments(image_array)
        cx = int(M['m10']/(M['m00'] + 1e-6))
        cy = int(M['m01']/(M['m00'] + 1e-6))
        return cx, cy

    def get_detected_objects(self, contour_images: List, color: str) -> List[Tuple]:
        """Returns all objects of the given color from the contour images."""
        objects = []
        for image in contour_images:
            area = cv2.contourArea(image)
            if area > self.block_contour_area_threshold:
                cx, cy = self.compute_mass_center(image)
                _, _, w, h = cv2.boundingRect(image)
                depth = self.get_pixel_depth(cx, cy)
                height = self.workbench_depth - depth
                objects.append((cx, cy, w, h, height, color, area))
        return objects
    
    def get_3D_point_from_pixel(self, u: float, v: float) -> Tuple[float, float, float]:
        """Computes the coordinates of the 3D point in the world frame that corresponds to the given pixel."""
        depth = self.get_pixel_depth(u, v)
        height = self.workbench_depth - depth
        
        ray = self.pin_cam.projectPixelTo3dRay((u, v))
        X_ray = ray[0]/ray[2]
        Y_ray = ray[1]/ray[2]
        
        Z = depth + height/2
        X = X_ray*Z 
        Y = Y_ray*Z
        
        X_world, Y_world, Z_world = self.convert_point_from_camera_to_world(X, Y, Z)
        return X_world, Y_world, Z_world

    def convert_point_from_camera_to_world(self, x: float, y: float, z: float) -> Tuple[float, float, float]:
        """Converts point coordinates from camera frame to world frame."""
        X_c = np.array([x, y, z, 1])
        X_world = np.dot(self.T_c_w, X_c)
        return X_world[0], X_world[1], X_world[2]

    def image_callback(self, msg: Image) -> None:
        """Detects objects from images based on color and publishes relevant information."""
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
            self.blocks_on_workbench = []
            for color in ["red", "green", "blue"]:
                contours = self.get_contours(hsv, color)
                detected_objects = self.get_detected_objects(contours, color)
                self.blocks_on_workbench += detected_objects
            
            # Draw bounding boxes
            for cx, cy, w, h, _, color, _ in self.blocks_on_workbench:
                cv2.rectangle(image, (cx-int(w/2),cy-int(h/2)), (cx+int(w/2),cy+int(h/2)), 
                            color=(255, 255, 255), thickness=1)
                cv2.putText(image, f"{color} ({int(cx)}, {int(cy)})", 
                          (int(cx-45), int(cy+30)), cv2.FONT_HERSHEY_SIMPLEX, 
                          fontScale=0.5, color=(255,255,255), thickness=1)
            
            cv2.imshow("Camera View", image)
            cv2.waitKey(1)
            
            self.publish_detected_objects()
        except Exception as e:
            self.get_logger().error(f'Error in image_callback: {e}')
    
    def publish_detected_objects(self) -> None:
        """Publishes information about the detected objects."""
        blocks = DetectedObjectsStamped()
        blocks.header.stamp = self.get_clock().now().to_msg()
        blocks.detected_objects = []
        
        for cx, cy, w, h, height, color, area in self.blocks_on_workbench:
            X_world, Y_world, Z_world = self.get_3D_point_from_pixel(cx, cy)
            width, length = self.get_box_dimensions(cx, cy, w, h)
            
            detected_block = DetectedObject()
            detected_block.x_world = X_world
            detected_block.y_world = Y_world
            detected_block.z_world = Z_world
            detected_block.width = width
            detected_block.length = length
            detected_block.height = height
            detected_block.color = color
            
            blocks.detected_objects.append(detected_block)
        
        self.detected_objects_pub.publish(blocks)

    def get_box_dimensions(self, cx: float, cy: float, w: float, h: float) -> Tuple[float, float]:
        """Computes the cartesian width and length of the box from pixel coordinates."""
        u1 = cx - int(w/2)
        v1 = cy - int(h/2)
        u2 = cx + int(w/2)
        v2 = cy + int(h/2)
        
        x1_world, y1_world, _ = self.get_3D_point_from_pixel(u1, v1)
        x2_world, y2_world, _ = self.get_3D_point_from_pixel(u2, v2)
        length = abs(x1_world - x2_world)
        width = abs(y1_world - y2_world)
        return width, length

    def get_contours(self, hsv: np.ndarray, color: str) -> List:
        """Creates contours of all objects in the given color."""
        mask = self.get_mask(hsv, color)
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contours


def main(args=None):
    rclpy.init(args=args)
    node = VisionObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

