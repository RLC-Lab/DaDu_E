import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image as PILImage
import numpy as np
from lang_sam import LangSAM
from image_geometry import PinholeCameraModel
import tf
import math

from geometry_msgs.msg import Twist
import time

import moveit_commander
import geometry_msgs.msg
import sys


pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def arm_detect(move_group):
    
    # rospy.init_node('move_to_goal', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()


    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = math.radians(83) 
    joint_goal[1] = math.radians(-90)
    joint_goal[2] = math.radians(45)
    joint_goal[3] = math.radians(-90)
    joint_goal[4] = math.radians(-90)
    joint_goal[5] = math.radians(180)


    move_group.go(joint_goal, wait=True)
    move_group.stop()


def send_tf(translation_vector, frame_id, child_frame_id):
    br = tf.TransformBroadcaster()
    br.sendTransform(
        (translation_vector[0], translation_vector[1], translation_vector[2]),
        (0, 0, 0, 1),  # No rotation, only translation
        rospy.Time.now(),
        child_frame_id,
        frame_id
    )

def transform_to_world(translation_vector, transform_listener):
    try:
        (trans, rot) = transform_listener.lookupTransform('/map', '/g_camera_color_optical_frame', rospy.Time(0))
        translation_matrix = np.eye(4)
        translation_matrix[:3, 3] = trans
        rotation_matrix = tf.transformations.quaternion_matrix(rot)
        transformation_matrix = np.dot(translation_matrix, rotation_matrix)

        center_3d_homogeneous = np.append(translation_vector, 1)
        world_center_3d = np.dot(transformation_matrix, center_3d_homogeneous)[:3]

        send_tf(world_center_3d, "world", "center_3d")
        return world_center_3d
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logerr(f"Transform lookup failed: {e}")
        return None

class LangSAMProcessor:
    def __init__(self, camera_model):
        self.model = LangSAM()
        self.camera_model = camera_model

    def detect_obj(self, image, text=None, visualize_box=False, box_filename=None, visualize_mask=False, mask_filename=None):
        masks, boxes, phrases, logits = self.model.predict(image, text)
        if len(masks) == 0:
            return None, None

        seg_mask = np.array(masks[0])
        bbox = np.array(boxes[0], dtype=int)

        if visualize_box:
            self.draw_bounding_box(image, bbox, box_filename)

        if visualize_mask:
            self.draw_mask_on_image(image, seg_mask, mask_filename)

        return seg_mask, bbox

    def draw_bounding_box(self, image, bbox, filename):
        import cv2
        img = np.array(image)
        cv2.rectangle(img, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (255, 0, 0), 2)
        cv2.imwrite(filename, img)

    def draw_mask_on_image(self, image, mask, filename):
        import cv2
        img = np.array(image)
        mask = mask.astype(np.uint8) * 255
        cv2.imwrite(filename, mask)

    def draw_3d_point_on_image(self, image, point_3d, filename):
        import cv2
        img = np.array(image)
        fx, fy = self.camera_model.fx(), self.camera_model.fy()
        cx, cy = self.camera_model.cx(), self.camera_model.cy()
        x, y, z = point_3d
        u = int(fx * x / z + cx)
        v = int(fy * y / z + cy)
        cv2.circle(img, (u, v), 5, (0, 255, 0), -1)
        cv2.imwrite(filename, img)

class RGBDLangSAMNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.camera_model = PinholeCameraModel()
        self.processor = LangSAMProcessor(self.camera_model)

        self.color_sub = rospy.Subscriber("/g_d435/rgb/g_image_raw", Image, self.color_callback)
        self.depth_sub = rospy.Subscriber("/g_d435/depth/g_image_raw", Image, self.depth_callback)
        self.camera_info_sub = rospy.Subscriber("/g_d435/rgb/camera_info", CameraInfo, self.camera_info_callback)

        self.color_image = None
        self.depth_image = None

        self.text_prompt = None
        self.tf_listener = tf.TransformListener()

    def camera_info_callback(self, data):
        self.camera_model.fromCameraInfo(data)

    def color_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.color_image = PILImage.fromarray(cv_image)
        except CvBridgeError as e:
            rospy.logerr(f"Could not convert color image: {e}")

    def depth_callback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr(f"Could not convert depth image: {e}")

    def process_image(self, text_prompt):
        self.text_prompt = text_prompt

        if self.color_image is None or self.depth_image is None:
            print("No image received yet.")
            return None, None, None

        seg_mask, bbox = self.processor.detect_obj(
            self.color_image,
            text=self.text_prompt,
            visualize_box=False,
            visualize_mask=False
        )

        if seg_mask is None or bbox is None:
            print("Object not detected in langsam while adjusting base.")
            return None, None, None

        center_3d = self.compute_center_3d(seg_mask)
        if center_3d is None:
            print("Object detected but failed to compute 3D center.")
            return None, None, None

        world_center_3d = transform_to_world(center_3d, self.tf_listener)
        if world_center_3d is None:
            print("Failed to transform 3D center to world coordinates.")
            return None, None, None

        current_pose = self.get_current_pose()
        if current_pose is None:
            print("Failed to get current robot pose.")
            return None, None, None

        distance, angle = self.calculate_distance_and_angle(current_pose, world_center_3d)
        return distance, angle, current_pose, world_center_3d

    def compute_center_3d(self, seg_mask):
        indices = np.argwhere(seg_mask)
        if len(indices) == 0:
            return None

        depth_values = self.depth_image[indices[:, 0], indices[:, 1]]
        valid_depth_values = depth_values[depth_values > 0]
        valid_depth_values = valid_depth_values[~np.isnan(valid_depth_values)]

        if len(valid_depth_values) == 0:
            return None

        avg_depth = np.mean(valid_depth_values)

        points_3d = []
        for idx in indices:
            depth = self.depth_image[idx[0], idx[1]]
            if depth > 0 and not np.isnan(depth):
                pixel = [idx[1], idx[0]]
                ray = np.array(self.camera_model.projectPixelTo3dRay(pixel))
                point_3d = ray * depth
                points_3d.append(point_3d)

        if len(points_3d) == 0:
            return None

        points_3d = np.array(points_3d)
        avg_3d = np.mean(points_3d, axis=0)

        center_pixel = np.mean(indices, axis=0).astype(int)
        center_ray = np.array(self.camera_model.projectPixelTo3dRay([center_pixel[1], center_pixel[0]]))

        center_3d = center_ray * avg_depth
        return center_3d

    def get_current_pose(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            return np.array(trans), tf.transformations.euler_from_quaternion(rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtractException):
            print("Failed to get current pose")
            return None

    def calculate_distance_and_angle(self, current_pose, target_position):
        current_position, current_orientation = current_pose
        distance = np.linalg.norm(current_position - target_position)
        target_vector = target_position - current_position

        x_distance = target_vector[0]
        y_distance = target_vector[1]

        angle_to_target = np.arctan2(y_distance, x_distance)
        robot_orientation = current_orientation[2]

        angle_difference = np.arctan2(np.sin(angle_to_target - robot_orientation), np.cos(angle_to_target - robot_orientation))

        if angle_difference>0:
            angle_difference = angle_difference + 0.01

        if angle_difference<0:
            angle_difference = angle_difference - 0.01
            
        return distance, angle_difference

    def adjust_base_position(self, base_position, target_position, arm_radius=0.63):
        x_base, y_base = base_position[:2]
        x_target, y_target = target_position[:2]

        delta_x = x_target - x_base
        delta_y = y_target - y_base

        distance = math.sqrt(delta_x**2 + delta_y**2)
        print("distance is: ", distance)

        if distance <= arm_radius-0.2:
            print("***Distance is within UR3 work range***")
            return (0, 0)

        unit_x = delta_x / distance
        unit_y = delta_y / distance

        move_distance = distance - arm_radius

        offset_x = move_distance * unit_x
        offset_y = move_distance * unit_y

        return (offset_x, offset_y)

def turn_angle(angle, angular_speed):

    if angle < 0:
        angular_speed = -abs(angular_speed)

    turn_cmd = Twist()
    turn_cmd.angular.z = angular_speed

    turn_time = abs(angle / angular_speed)
    start_time = rospy.Time.now().to_sec()

    while (rospy.Time.now().to_sec() - start_time) < turn_time:
        pub.publish(turn_cmd)
        # rate.sleep()

    turn_cmd.angular.z = 0
    pub.publish(turn_cmd)

def move_distance(x_distance, y_distance, speed):

    move_cmd = Twist()
    move_cmd.linear.x = speed
    move_cmd.linear.y = speed

    move_time_x = abs(x_distance / speed) if x_distance != 0 else 0
    move_time_y = abs(y_distance / speed) if y_distance != 0 else 0
    move_time = max(move_time_x, move_time_y)

    start_time = rospy.Time.now().to_sec()

    while (rospy.Time.now().to_sec() - start_time) < move_time:
        pub.publish(move_cmd)
        # rate.sleep()

    move_cmd.linear.x = 0
    pub.publish(move_cmd)

def adjust_main(text_prompt, move_group):
    # rospy.init_node('rgbd_lang_sam_node')
    node = RGBDLangSAMNode()
    
    # pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    rospy.sleep(1)  # Allow some time for the ROS topics to start

    # moveit_commander.roscpp_initialize(sys.argv)
    # group_name = "ur3_manipulator" 
    # move_group = moveit_commander.MoveGroupCommander(group_name)

    arm_detect(move_group)

#    time.sleep(10)
    distance, angle, current_pose, world_center_3d = node.process_image(text_prompt)
    if distance is not None and angle is not None:
        new_base_position = node.adjust_base_position(current_pose[0], world_center_3d)
        print("angle, x distance, y distance")
        print(f"{angle} {new_base_position[0]} {new_base_position[1]}")
        turn_angle(angle, 0.5)
        time.sleep(3)
        move_distance(new_base_position[0], new_base_position[1], 0.5)
        # moveit_commander.roscpp_shutdown()
    else:
        print("Processing failed.")

if __name__ == "__main__":
    rospy.init_node('rgbd_lang_sam_node')
    text_prompt='cup'
    adjust_main(text_prompt)
