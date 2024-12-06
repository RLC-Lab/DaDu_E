import os
import math
import copy
import sys

import numpy as np
import open3d as o3d
from PIL import Image
import  PIL

from utils.types import Bbox
from gsnet import AnyGrasp
from graspnetAPI import GraspGroup
from utils.zmq_socket import ZmqSocket
from utils.utils import (
    get_3d_points,
    visualize_cloud_geometries,
    sample_points,
    draw_rectangle,
)
from utils.camera import CameraParameters
from image_processors import LangSAMProcessor

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
import geometry_msgs.msg
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, JointState

from tf.transformations import euler_from_quaternion, quaternion_matrix, quaternion_from_matrix

from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal
import actionlib
import moveit_commander

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion
import tf

def matrix_to_pose_stamped(transform_matrix):
    # Extract rotation matrix and translation vector from the transformation matrix
    rotation_matrix = transform_matrix[:3, :3]
    translation_vector = transform_matrix[:3, 3]

    # Convert the rotation matrix to quaternion
    quaternion = quaternion_from_matrix(transform_matrix)

    # Create a PoseStamped message
    pose_stamped_msg = PoseStamped()
    pose_stamped_msg.pose.position.x = translation_vector[0]
    pose_stamped_msg.pose.position.y = translation_vector[1]
    pose_stamped_msg.pose.position.z = translation_vector[2]
    pose_stamped_msg.pose.orientation.x = quaternion[0]
    pose_stamped_msg.pose.orientation.y = quaternion[1]
    pose_stamped_msg.pose.orientation.z = quaternion[2]
    pose_stamped_msg.pose.orientation.w = quaternion[3]

    return pose_stamped_msg

class CameraObjectHandler:
    def __init__(self, cfgs):
        self.cfgs = cfgs
        self.grasping_model = AnyGrasp(self.cfgs)
        self.grasping_model.load_net()

        self.lang_sam = LangSAMProcessor()

        if self.cfgs.open_communication:
            self.socket = ZmqSocket(self.cfgs)

        rospy.init_node('CameraAnygrasp', anonymous=False)

        # Camera--real
        self.frame_id = "camera_link"
        self.height, self.width = None, None
        
        # self.color_info_topic = "/camera/color/camera_info"
        # self.depth_info_topic = "/camera/aligned_depth_to_color/camera_info"
        # self.intrinsics_topic = "/camera/color/camera_info"
        
        # self.color_topic = "/camera/color/image_raw"
        # self.depth_topic = "/camera/aligned_depth_to_color/image_raw"

        # self.color_info_topic = "/l_camera/color/camera_info"
        # self.depth_info_topic = "/l_camera/aligned_depth_to_color/camera_info"
        # self.intrinsics_topic = "/l_camera/color/camera_info"
        
        # self.color_topic = "/l_camera/color/image_raw"
        # self.depth_topic = "/l_camera/aligned_depth_to_color/image_raw"

        self.color_info_topic = "/g_d435/rgb/camera_info"
        self.depth_info_topic = "/g_d435/depth/camera_info"
        self.intrinsics_topic = "/g_d435/rgb/camera_info"
        
        self.color_topic = "/g_d435/rgb/g_image_raw"
        self.depth_topic = "/g_d435/depth/g_image_raw"
        
        #self.camera_topic = "/vrpn_client_node/franka_base16/pose"
        self.camera_topic = "/vrpn_client_node/cam_grasp/pose_transform"

        # Subscribers
        self.image_sub = rospy.Subscriber(self.color_topic, Image, self.callback_receive_color_image, queue_size=1)
        self.depth_sub = rospy.Subscriber(self.depth_topic, Image, self.callback_receive_depth_image, queue_size=1)
        self.intrinsics_sub = rospy.Subscriber(self.intrinsics_topic, CameraInfo, self.callback_intrinsics, queue_size=1)
        self.camera_sub = rospy.Subscriber(self.camera_topic, PoseStamped, self.callback_camera_pose)

        # Publishers
        self.grasp_pose_pub = rospy.Publisher('/grasp_pose', PoseStamped, queue_size=1)
        self.grasp_pose_above_pub = rospy.Publisher('/grasp_pose_above', PoseStamped, queue_size=1)
        self.grasp_pose_joint_pub = rospy.Publisher('/grasp_pose/joint_space', JointState, queue_size=1)
        self.grasp_pose_above_joint_pub = rospy.Publisher('/grasp_pose_above/joint_space', JointState, queue_size=1)
        # send pose infoormation to /detect_grasps/pose_grasps
        self.pose_pub = rospy.Publisher('/detect_grasps/pose_grasps', Pose, queue_size=1)        
        
        self.init_pose_joint_pub = rospy.Publisher('/init_pose/joint_space', JointState, queue_size=1)

        self.camera_intrinsics = None
        self.camera_pose = None

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.scale = None

        self.color_sensor_state = {'active': False, 'ready': False}
        self.depth_sensor_state = {'active': False, 'ready': False}

    def _active_sensor(self):
        self.color_sensor_state['active'] = True
        self.depth_sensor_state['active'] = True

    def callback_camera_pose(self, data):# Extract quaternion from PoseStamped message
        self.camera_pose = data.pose
        
        # quaternion = (
        #     data.pose.orientation.x,
        #     data.pose.orientation.y,
        #     data.pose.orientation.z,
        #     data.pose.orientation.w
        # )

        # # Convert quaternion to Euler angles in degrees
        # euler_angles = euler_from_quaternion(quaternion)
        # roll, pitch, yaw = euler_angles
        # self.roll, self.pitch, self.yaw = roll * 180 / 3.14159, pitch * 180 / 3.14159, yaw * 180 / 3.14159  

    def callback_intrinsics(self, data):
        self.intrinsics = data
        
        self.height, self.width = self.intrinsics.height, self.intrinsics.width
        self.camera_intrinsics = np.array(self.intrinsics.K).reshape(3, 3)
        self.fx = self.camera_intrinsics[0, 0]
        self.fy = self.camera_intrinsics[1, 1]
        self.cx = self.camera_intrinsics[0, 2]
        self.cy = self.camera_intrinsics[1, 2]
        # self.scale = 1/0.001
        self.scale = 1
        self.intrinsics_sub.unregister()
        print('callback_intrinsics ok!')
    
    def callback_receive_color_image(self, image):
        if not self.color_sensor_state['active']:
            return

        # Get BGR image from data
        self.current_image = np.frombuffer(image.data, dtype=np.uint8).reshape(
            image.height, image.width, -1)       
        

        self.color_sensor_state['active'] = False
        self.color_sensor_state['ready'] = True

        print('callback_receive_color_image ok!')
    
    def callback_receive_depth_image(self, depth):
        """ Callback. Get raw depth from data (Unit: mm). """

        if not self.depth_sensor_state['active']:
            return
        """
            Reference here:
                current_depth = self.cv_bridge.imgmsg_to_cv2(depth, "passthrough")
        """

        # Way 1: works
        if depth.encoding == '16UC1':
            channel = 1
            dtype = np.dtype('uint16')
            dtype = dtype.newbyteorder('>' if depth.is_bigendian else '<')

        # NOTE! not sure
        elif depth.encoding == '32FC1':
            channel = 1
            dtype = np.dtype('float32')
            dtype = dtype.newbyteorder('>' if depth.is_bigendian else '<')

        current_depth = np.frombuffer(depth.data, dtype=dtype).reshape(
            depth.height, depth.width, channel)

        # Way 2: works
        # if depth.encoding == '16UC1':
        #     depth.encoding = "mono16"
        #     current_depth = self.cv_bridge.imgmsg_to_cv2(depth, "mono16")

        # elif depth.encoding == '32FC1':
        #     depth.encoding = "mono16"
        #     current_depth = self.cv_bridge.imgmsg_to_cv2(depth, "mono16")


        # Way 3: works
        # current_depth = self.cv_bridge.imgmsg_to_cv2(
        #     depth, desired_encoding="passthrough")

        # Convert unit from millimeter into meter
        # current_depth = current_depth.astype(float) / 1000.
        current_depth = current_depth.astype(float)

        self.current_depth = current_depth.squeeze(axis=2) \
                             if len(current_depth.shape) >= 3 else current_depth

        self.depth_sensor_state['active'] = False
        self.depth_sensor_state['ready'] = True

        print('callback_receive_depth_image ok!')

    def get_rgbd_images(self):
        self._active_sensor()
        i = 0
        while True:
            if (self.color_sensor_state['ready'] and
                self.depth_sensor_state['ready']):
                color_image = self.current_image
                depth_image = self.current_depth

                # 调试保存图像
                # img = PIL.Image.fromarray(color_image)
                # img.save("./color_img.jpg")

                self.color_sensor_state['ready'] = False
                self.depth_sensor_state['ready'] = False
                return color_image, depth_image

            rospy.sleep(0.1)
            i += 1
            print(i, end='\r')
            if i >= 50:
                print("No image")
                exit()

    def receive_input(self, tries, grasp_object):
        if self.cfgs.open_communication:
            print("\n\nWaiting for data from Robot")
            # Reading color array
            colors = self.socket.recv_array()
            self.socket.send_data("RGB received")

            # Depth data
            depths = self.socket.recv_array()
            # print(np.max(depths), np.min(depths))
            self.socket.send_data("depth received")

            # Camera Intrinsics
            fx, fy, cx, cy, head_tilt = self.socket.recv_array()
            self.socket.send_data("intrinsics received")

            # Object query
            self.query = self.socket.recv_string()
            self.socket.send_data("text query received")
            print(f"Text - {self.query}")

            # action -> ["pick", "place"]
            self.action = self.socket.recv_string()
            self.socket.send_data("Mode received")
            print(f"Manipualtion Mode - {self.action}")
            print(self.socket.recv_string())

            image = Image.fromarray(colors)
        else:
            # head_tilt = -45
            # data_dir = "./example_data/"
            # colors = np.array(Image.open(os.path.join(data_dir, "test_rgb.png")))
            # image = Image.open(os.path.join(data_dir, "test_rgb.png"))
            # depths = np.array(Image.open(os.path.join(data_dir, "test_depth.png")))
            # fx, fy, cx, cy, scale = 306, 306, 118, 211, 0.001
            # if tries == 1:
            #     self.action = str(input("Enter action [pick/place]: "))
            #     self.query = str(input("Enter a Object name in the scence: "))
            # depths = depths * scale


            rospy.sleep(1)
            self.action = "pick"
            # self.query = "bottle" 


            # self.action = str(input("Enter action [pick/place]: "))
            # self.query = str(input("Enter a Object name in the scence: "))
            self.query = grasp_object      
            print("Current input is:", grasp_object)       
            
            head_tilt = 0

            self._active_sensor()
            i = 0

            stop = False
            while not stop:
                if (self.color_sensor_state['ready'] and
                    self.depth_sensor_state['ready']):
                    colors = self.current_image
                    depths = self.current_depth
                    image = PIL.Image.fromarray(colors)

                    fx = self.fx
                    fy = self.fy
                    cx = self.cx
                    cy = self.cy
                    depths = depths / self.scale

                    # if tries == 1:
                    #     self.action = str(input("Enter action [pick/place]: "))
                    #     self.query = str(input("Enter a Object name in the scence: "))                   

                    # 调试保存图像
                    image.save("./color_img.jpg")

                    self.color_sensor_state['ready'] = False
                    self.depth_sensor_state['ready'] = False
                    stop = True

                rospy.sleep(0.1)
                i += 1
                print(i, end='\r')
                if i >= 50:
                    print("No image")
                    exit()

        # Camera Parameters
        colors = colors / 255.0
        head_tilt = head_tilt / 100
        self.cam = CameraParameters(fx, fy, cx, cy, head_tilt, image, colors, depths)

    def manipulate(self, grasp_object):
        """
        Wrapper for grasping and placing

        11 is the maximum number of retries incase of object or grasp detection failure
        Try - 1 -> captures image and centers the robot
        Try - 2 -> captures image and tries to perform action
        If failed:
            Try - 3,4 -> tries in a different camera orientation
        Even then if it fails:
            Try - 5,6,7 -> moves base to left tries again three different camera orientations
        Even then if it fails:
            Try - 8,9,10 -> moves base to the right and agian tries three different camera orientations
        Finally if it fails to detect any pose in above all attempts:
            Try 11 -> If object is detected but anygrasp couldnt find a pose as a last resort
                      the cropped object image is sent to the model.
        In any of the above attempts it is able to succedd it wont perform any further tries
        """

        tries = 1
        retry = True
        while retry and tries <= 11:

            rospy.sleep(0.1)
            self.receive_input(tries, grasp_object)

            # Directory for saving visualisations
            self.save_dir = self.cfgs.environment + "/" + self.query + "/anygrasp/"
            if not os.path.exists(self.save_dir):
                os.makedirs(self.save_dir)
            if self.cfgs.open_communication:
                camera_image_file_name = self.save_dir + "/clean_" + str(tries) + ".jpg"
                self.cam.image.save(camera_image_file_name)
                print(f"Saving the camera image at {camera_image_file_name}")
                np.save(
                    self.save_dir + "/depths_" + str(tries) + ".npy", self.cam.depths
                )

            box_filename = f"{self.cfgs.environment}/{self.query}/anygrasp/object_detection_{tries}.jpg"
            mask_filename = f"{self.cfgs.environment}/{self.query}/anygrasp/semantic_segmentation_{tries}.jpg"
            # Object Segmentaion Mask
            seg_mask, bbox = self.lang_sam.detect_obj(
                self.cam.image,
                self.query,
                visualize_box=True,
                visualize_mask=True,
                box_filename=box_filename,
                mask_filename=mask_filename,
            )

            if bbox is None:
                if self.cfgs.open_communication:
                    print("!!![Didnt detect the object, Trying Again]!!!")
                    tries = tries + 1
                    print(f"Try no: {tries}")
                    data_msg = "No Objects detected, Have to try again"
                    self.socket.send_data([[0], [0], [0, 0, 2], data_msg])
                    if tries == 11:
                        return
                    continue
                    # self.socket.send_data("No Objects detected, Have to try again")
                else:
                

                    print(
                        "!!![bbox is None,Didnt find the Object. Trying to turn left 2 axis]!!!"
                    )
                    
                    # rospy.init_node('rotate_robot')
                    # 创建SimpleActionClient，'move_base'与move_base服务器通信
                    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
                    client.wait_for_server()
                    # 获取当前机器人的方向
                    listener = tf.TransformListener()
                    listener.waitForTransform("/map", "/base_link", rospy.Time(), rospy.Duration(4.0))
                    (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
                    euler = tf.transformations.euler_from_quaternion(rot)
                    # 计算旋转目标角度（将30度转换为弧度）
                    angle_to_rotate = 30 * math.pi / 180  # 30 degrees in radians
                    # 创建新的四元数，代表旋转后的方向
                    goal_orient = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2] + angle_to_rotate)
                    # 创建一个新的目标点
                    goal = MoveBaseGoal()
                    goal.target_pose.header.frame_id = "base_link"
                    goal.target_pose.header.stamp = rospy.Time.now()
                    goal.target_pose.pose.position.x = 0.0
                    goal.target_pose.pose.position.y = 0.0
                    goal.target_pose.pose.position.z = 0.0
                    goal.target_pose.pose.orientation = Quaternion(*goal_orient)
                    print("----------------Get goal----------------")
                    client.send_goal(goal)
                    print("----------------Send turning axis to move_base----------------")


                    # Get current view
                    color_image = self.current_image
                    img_debug = PIL.Image.fromarray(color_image)
                    img_debug.save("./view_debug.jpg")
                    print("Current view is at ./view_debug.jpg")

                    client.wait_for_result()

                    # 检查是否成功
                    if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                        rospy.loginfo("Rotation completed successfully!")
                    else:
                        rospy.logerr("Rotation failed!")

                    
                    retry = False

            bbox_x_min, bbox_y_min, bbox_x_max, bbox_y_max = bbox

            # Center the robot
            if tries == 1 and self.cfgs.open_communication:
                self.center_robot(bbox)
                tries += 1
                continue

            points = get_3d_points(self.cam)
            # print('points: ',points.size)
            # print('seg_mask: ',seg_mask.size)
            # print('bbox: ',bbox.size)

            if self.action == "place":
                retry = not self.place(points, seg_mask)
            else:
                retry = not self.pickup(points, seg_mask, bbox, (tries == 11))

            if retry:
                if self.cfgs.open_communication:
                    print("Trying Again")
                    tries = tries + 1
                    print(f"Try no: {tries}")
                    data_msg = "No poses, Have to try again"
                    self.socket.send_data([[0], [0], [0, 0, 2], data_msg])
                    # self.socketf.send_data("No poses, Have to try again")
                else:
                    print(
                        "Try with another object or tune grasper height and width parameters in demo.py"
                    )
                    retry = False

    def center_robot(self, bbox: Bbox):
        """
        Center the robots base and camera to face the center of the Object Bounding box
        """

        bbox_x_min, bbox_y_min, bbox_x_max, bbox_y_max = bbox

        bbox_center = [
            int((bbox_x_min + bbox_x_max) / 2),
            int((bbox_y_min + bbox_y_max) / 2),
        ]
        depth_obj = self.cam.depths[bbox_center[1], bbox_center[0]]
        print(
            f"{self.query} height and depth: {((bbox_y_max - bbox_y_min) * depth_obj)/self.cam.fy}, {depth_obj}"
        )

        # base movement
        dis = (bbox_center[0] - self.cam.cx) / self.cam.fx * depth_obj
        # print(f"Base displacement {dis}")

        # camera tilt
        tilt = math.atan((bbox_center[1] - self.cam.cy) / self.cam.fy)
        # print(f"Camera Tilt {tilt}")

        if self.cfgs.open_communication:
            data_msg = "Now you received the base and haed trans, good luck."
            self.socket.send_data([[-dis], [-tilt], [0, 0, 1], data_msg])
            # self.socket.send_data("Now you received the base and haed trans, good luck.")

    def place(self, points: np.ndarray, seg_mask: np.ndarray) -> bool:
        points_x, points_y, points_z = points[:, :, 0], points[:, :, 1], points[:, :, 2]
        flat_x, flat_y, flat_z = (
            points_x.reshape(-1),
            -points_y.reshape(-1),
            -points_z.reshape(-1),
        )

        # Removing all points whose depth is zero(undetermined)
        zero_depth_seg_mask = (
            (flat_x != 0) * (flat_y != 0) * (flat_z != 0) * seg_mask.reshape(-1)
        )
        flat_x = flat_x[zero_depth_seg_mask]
        flat_y = flat_y[zero_depth_seg_mask]
        flat_z = flat_z[zero_depth_seg_mask]

        colors = self.cam.colors.reshape(-1, 3)[zero_depth_seg_mask]

        # 3d point cloud in camera orientation
        points1 = np.stack([flat_x, flat_y, flat_z], axis=-1)

        # Rotation matrix for camera tilt
        cam_to_3d_rot = np.array(
            [
                [1, 0, 0],
                [0, math.cos(self.cam.head_tilt), math.sin(self.cam.head_tilt)],
                [0, -math.sin(self.cam.head_tilt), math.cos(self.cam.head_tilt)],
            ]
        )

        # 3d point cloud with upright camera
        transformed_points = np.dot(points1, cam_to_3d_rot)

        # Removing floor points from point cloud
        floor_mask = transformed_points[:, 1] > -1.25
        transformed_points = transformed_points[floor_mask]
        transformed_x = transformed_points[:, 0]
        transformed_y = transformed_points[:, 1]
        transformed_z = transformed_points[:, 2]
        colors = colors[floor_mask]

        pcd1 = o3d.geometry.PointCloud()
        pcd1.points = o3d.utility.Vector3dVector(points1)
        pcd1.colors = o3d.utility.Vector3dVector(colors)

        pcd2 = o3d.geometry.PointCloud()
        pcd2.points = o3d.utility.Vector3dVector(transformed_points)
        pcd2.colors = o3d.utility.Vector3dVector(colors)

        # Projected Median in the xz plane [parallel to floor]
        xz = np.stack([transformed_x * 100, transformed_z * 100], axis=-1).astype(int)
        unique_xz = np.unique(xz, axis=0)
        unique_xz_x, unique_xz_z = unique_xz[:, 0], unique_xz[:, 1]
        px, pz = np.median(unique_xz_x) / 100.0, np.median(unique_xz_z) / 100.0

        x_margin, z_margin = 0.1, 0
        x_mask = (transformed_x < (px + x_margin)) & (transformed_x > (px - x_margin))
        y_mask = (transformed_y < 0) & (transformed_y > -1.1)
        z_mask = (transformed_z < 0) & (transformed_z > (pz - z_margin))
        mask = x_mask & y_mask & z_mask
        py = np.max(transformed_y[mask])
        point = np.array(
            [px, py, pz]
        )  # Final placing point in upright camera co-ordinate system

        geometries = []
        cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=0.1, height=0.04)
        cylinder_rot = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
        cylinder.rotate(cylinder_rot)
        cylinder.translate(point)
        cylinder.paint_uniform_color([0, 1, 0])
        geometries.append(cylinder)

        if self.cfgs.debug:
            visualize_cloud_geometries(
                pcd2,
                geometries,
                save_file=self.save_dir + "/placing.jpg",
                visualize=not self.cfgs.headless,
            )

        point[1] += 0.1
        transformed_point = cam_to_3d_rot @ point
        print(f"Placing point of Object relative to camera: {transformed_point}")

        if self.cfgs.open_communication:
            data_msg = "Now you received the gripper pose, good luck."
            self.socket.send_data(
                [
                    np.array(transformed_point, dtype=np.float64),
                    [0],
                    [0, 0, 0],
                    data_msg,
                ]
            )

        return True

    def pickup(
        self,
        points: np.ndarray,
        seg_mask: np.ndarray,
        bbox: Bbox,
        crop_flag: bool = False,
    ):
        # print('pickup...... ')        
        points_x, points_y, points_z = points[:, :, 0], points[:, :, 1], points[:, :, 2]

        # Filtering points based on the distance from camera
        mask = (points_z > self.cfgs.min_depth) & (points_z < self.cfgs.max_depth)
        points = np.stack([points_x, points_y, points_z], axis=-1)
        points = points[mask].astype(np.float32)
        colors_m = self.cam.colors[mask].astype(np.float32)

        if self.cfgs.sampling_rate < 1:
            points, indices = sample_points(points, self.cfgs.sampling_rate)
            colors_m = colors_m[indices]

        # Grasp Prediction
        # gg is a list of grasps of type graspgroup in graspnetAPI
        xmin = points[:, 0].min()
        xmax = points[:, 0].max()
        ymin = points[:, 1].min()
        ymax = points[:, 1].max()
        zmin = points[:, 2].min()
        zmax = points[:, 2].max()
        # xmin, xmax = -0.30, 0.30
        # ymin, ymax = -0.30, 0.30
        # zmin, zmax = 0.1, 0.8
        lims = [xmin, xmax, ymin, ymax, zmin, zmax]        
        gg, cloud = self.grasping_model.get_grasp(points, colors_m, lims)
        # print('gg: ',len(gg))
        if len(gg) == 0:
            print("No Grasp detected after collision detection!")
            return False

        gg = gg.nms().sort_by_score()
        filter_gg = GraspGroup()

        # Filtering the grasps by penalising the vertical grasps as they are not robust to calibration errors.
        bbox_x_min, bbox_y_min, bbox_x_max, bbox_y_max = bbox
        W, H = self.cam.image.size
        ref_vec = np.array(
            [0, math.cos(self.cam.head_tilt), -math.sin(self.cam.head_tilt)]
        )
        min_score, max_score = 1, -10
        image = copy.deepcopy(self.cam.image)
        img_drw = draw_rectangle(image, bbox)
        for g in gg:
            grasp_center = g.translation
            ix, iy = (
                int(((grasp_center[0] * self.cam.fx) / grasp_center[2]) + self.cam.cx),
                # int(((-grasp_center[1] * self.cam.fy) / grasp_center[2]) + self.cam.cy), # Real camera
                int(((-grasp_center[1] * self.cam.fy) / grasp_center[2]) + self.cam.cy),
            )
            if ix < 0:
                ix = 0
            if iy < 0:
                iy = 0
            if ix >= W:
                ix = W - 1
            if iy >= H:
                iy = H - 1
            # rotation_matrix = g.rotation_matrix
            # cur_vec = rotation_matrix[:, 0]
            # angle = math.acos(np.dot(ref_vec, cur_vec) / (np.linalg.norm(cur_vec)))
            # if not crop_flag:
            #     score = g.score - 0.1 * (angle) ** 4
            # else:
            #     score = g.score
                
            score = g.score

            if not crop_flag:
                if seg_mask[iy, ix]:
                    img_drw.ellipse([(ix - 2, iy - 2), (ix + 2, iy + 2)], fill="green")
                    if g.score >= 0.095:
                        g.score = score
                    min_score = min(min_score, g.score)
                    max_score = max(max_score, g.score)
                    filter_gg.add(g)
                else:
                    img_drw.ellipse([(ix - 2, iy - 2), (ix + 2, iy + 2)], fill="red")
            else:
                g.score = score
                filter_gg.add(g)
        # print('filter_gg: ',len(filter_gg))


        if len(filter_gg) == 0:
            print("!!![No grasp poses detected for this object.Trying to turn lefr 2 axis]!!!")
            # rospy.init_node('rotate_robot')

            client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            client.wait_for_server()

            listener = tf.TransformListener()
            listener.waitForTransform("/map", "/base_link", rospy.Time(), rospy.Duration(4.0))
            (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(rot)

            angle_to_rotate = 15 * math.pi / 180  # 30 degrees in radians
            goal_orient = tf.transformations.quaternion_from_euler(euler[0], euler[1], euler[2] + angle_to_rotate)

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "base_link"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = 0.0
            goal.target_pose.pose.position.y = 0.0
            goal.target_pose.pose.position.z = 0.0
            goal.target_pose.pose.orientation = Quaternion(*goal_orient)
            print("----------------Get goal----------------")
            client.send_goal(goal)
            print("----------------Send goal to move_base----------------")
            client.wait_for_result()
            if len(filter_gg) == 0:
                print("!!![Trying to detect again]!!!")
            else:
                if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo("Rotation completed successfully!")
                else:
                    rospy.logerr("Rotation failed!")
            
            return False

        projections_file_name = (
            self.cfgs.environment + "/" + self.query + "/anygrasp/grasp_projections.jpg"
        )
        image.save(projections_file_name)
        print(f"Saved projections of grasps at {projections_file_name}")
        filter_gg = filter_gg.nms().sort_by_score()

        # print('best_rot: ',filter_gg[0].translation)
        # print('best_trans: ',filter_gg[0].rotation_matrix)

        # publish grasp pose
        T_c_a = np.eye(4)
        T_c_a[:3, :3] = filter_gg[0].rotation_matrix
        T_c_a[:3, 3] = filter_gg[0].translation

        # Y rotation to correct for Anygrasp frame -> franka robot frame
        rotation_matrix_y = np.array([
            [0, 0, 1],
            [0, 1, 0],
            [-1, 0, 0]
        ])
        rotation_matrix_z = np.array([
            [0, 1, 0],
            [-1, 0, 0],
            [0, 0, 1]
        ])
        matrix_orientation =  np.eye(4)
        matrix_orientation[:3, :3] = rotation_matrix_y # @ rotation_matrix_z
        T_c_a = T_c_a @ matrix_orientation

        # Transformation to get the EE gripper at correct pose
        offset_grasp = 0.03
        T_c_a[:3, 3] = T_c_a[:3, 3] + offset_grasp * T_c_a[:3, 2]
        # print("Rotate:", T_c_a)

        # Pose topic creation
        pose_grasp = matrix_to_pose_stamped(T_c_a)
        self.pose_pub.publish(pose_grasp.pose)
        #Add
        # self.callback_pose(pose_grasp.pose)
        print("init grasping pose is :\n", pose_grasp.pose)


        if self.cfgs.debug:
            trans_mat = np.array(
                [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
            )
            cloud.transform(trans_mat)
            grippers = gg.to_open3d_geometry_list()
            filter_grippers = filter_gg.to_open3d_geometry_list()
            for gripper in grippers:
                gripper.transform(trans_mat)
            for gripper in filter_grippers:
                gripper.transform(trans_mat)

            visualize_cloud_geometries(
                cloud,
                grippers,
                visualize=not self.cfgs.headless,
                save_file=f"{self.cfgs.environment}/{self.query}/anygrasp/poses.jpg",
            )
            visualize_cloud_geometries(
                cloud,
                [filter_grippers[0].paint_uniform_color([1.0, 0.0, 0.0])],
                visualize=not self.cfgs.headless,
                save_file=f"{self.cfgs.environment}/{self.query}/anygrasp/best_pose.jpg",
            )
            

        if self.cfgs.open_communication:
            data_msg = "Now you received the gripper pose, good luck."
            self.socket.send_data(
                [
                    filter_gg[0].translation,
                    filter_gg[0].rotation_matrix,
                    [filter_gg[0].depth, crop_flag, 0],
                    data_msg,
                ]
            )        
        # sys.exit()
        
        return True
    
    
    def callback_pose(self,msg):
        # Get pose
        
        print("Reveived message")
        position = msg.position
        orientation = msg.orientation
        print(position,orientation)

        moveit_commander.roscpp_initialize(sys.argv)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()

        group_name = "ur3_manipulator"  # 修改为您的planning group名称
        move_group = moveit_commander.MoveGroupCommander(group_name)


        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation = orientation
        pose_goal.position.x = position.x
        pose_goal.position.y = position.y
        pose_goal.position.z = position.z

        move_group.set_pose_target(pose_goal)
        move_group.set_planning_time(30.0)
        # Strat plannning
        plan = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

        if plan:
            print("Plan success")
        else:
            print("Plan failed")

        # feedback
        current_pose = move_group.get_current_pose().pose
        print("Current pose is:", current_pose)
