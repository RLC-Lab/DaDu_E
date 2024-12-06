import os
import math
import copy
import sys
import time
import json

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
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, JointState

import tf
from tf.transformations import euler_from_quaternion, quaternion_matrix, quaternion_from_matrix

from std_srvs.srv import Empty,EmptyResponse

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
    model_initialized = False  # Static variable to track model initialization

    def __init__(self, cfgs):
        self.cfgs = cfgs
        self.lang_sam = LangSAMProcessor()

        if not CameraObjectHandler.model_initialized:
            self.grasping_model = AnyGrasp(self.cfgs)
            self.grasping_model.load_net()
            CameraObjectHandler.model_initialized = True
        
        if self.cfgs.open_communication:
            self.socket = ZmqSocket(self.cfgs)

        # While running other nodes, we need to disallow the node to exit
        # rospy.init_node('CameraAnygrasp', anonymous=False)

        # Camera--real
        self.frame_id = "camera_link"
        self.height, self.width = None, None

        # rospy.init_node('CameraAnygrasp', anonymous=False)

        self.color_topic = "/camera/color/image_raw"
        self.depth_topic = "/camera/aligned_depth_to_color/image_raw"

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

    def callback_camera_pose(self, data):
        self.camera_pose = data.pose

    def callback_intrinsics(self, data):
        self.intrinsics = data
        
        self.height, self.width = self.intrinsics.height, self.intrinsics.width
        self.camera_intrinsics = np.array(self.intrinsics.K).reshape(3, 3)
        self.fx = self.camera_intrinsics[0, 0]
        self.fy = self.camera_intrinsics[1, 1]
        self.cx = self.camera_intrinsics[0, 2]
        self.cy = self.camera_intrinsics[1, 2]
        self.scale = 1
        # self.scale = 1/0.001
        self.intrinsics_sub.unregister()
        print('callback_intrinsics ok!')
    
    def callback_receive_color_image(self, image):
        if not self.color_sensor_state['active']:
            return

        self.current_image = np.frombuffer(image.data, dtype=np.uint8).reshape(
            image.height, image.width, -1)

        self.color_sensor_state['active'] = False
        self.color_sensor_state['ready'] = True

        print('callback_receive_color_image ok!')
    
    def callback_receive_depth_image(self, depth):
        if not self.depth_sensor_state['active']:
            return

        if depth.encoding == '16UC1':
            channel = 1
            dtype = np.dtype('uint16')
            dtype = dtype.newbyteorder('>' if depth.is_bigendian else '<')
        elif depth.encoding == '32FC1':
            channel = 1
            dtype = np.dtype('float32')
            dtype = dtype.newbyteorder('>' if depth.is_bigendian else '<')

        current_depth = np.frombuffer(depth.data, dtype=dtype).reshape(
            depth.height, depth.width, channel)

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

                self.color_sensor_state['ready'] = False
                self.depth_sensor_state['ready'] = False
                return color_image, depth_image

            rospy.sleep(0.1)
            i += 1
            print(i, end='\r')
            if i >= 50:
                print("No image")
                exit()

    def store_pose_to_param_server(self, pose, param_name):
    rospy.set_param(param_name, {
        'position': {
            'x': pose.position.x,
            'y': pose.position.y,
            'z': pose.position.z
        },
        'orientation': {
            'x': pose.orientation.x,
            'y': pose.orientation.y,
            'z': pose.orientation.z,
            'w': pose.orientation.w
        }
    })

    def receive_input(self, tries, grasp_object):
        if self.cfgs.open_communication:
            print("\n\nWaiting for data from Robot")
            colors = self.socket.recv_array()
            self.socket.send_data("RGB received")

            depths = self.socket.recv_array()
            self.socket.send_data("depth received")

            fx, fy, cx, cy, head_tilt = self.socket.recv_array()
            self.socket.send_data("intrinsics received")

            self.query = self.socket.recv_string()
            self.socket.send_data("text query received")
            print(f"Text - {self.query}")

            self.action = self.socket.recv_string()
            self.socket.send_data("Mode received")
            print(f"Manipualtion Mode - {self.action}")
            print(self.socket.recv_string())

            image = Image.fromarray(colors)
        else:
            rospy.sleep(1)
            # self.action = str(input("Enter action [pick/place]: "))
            self.action = "pick"
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

                    self.color_sensor_state['ready'] = False
                    self.depth_sensor_state['ready'] = False
                    stop = True

                rospy.sleep(0.1)
                i += 1
                print(i, end='\r')
                if i >= 50:
                    print("No image")
                    exit()

        colors = colors / 255.0
        head_tilt = head_tilt / 100
        self.cam = CameraParameters(fx, fy, cx, cy, head_tilt, image, colors, depths)

    def manipulate(self, grasp_object):

        tries = 1
        retry = True
        while retry and tries <= 11:

            rospy.sleep(0.1)
            self.receive_input(tries, grasp_object)

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
                    print("Didnt detect the object, Trying Again")
                    tries = tries + 1
                    print(f"Try no: {tries}")
                    data_msg = "No Objects detected, Have to try again"
                    self.socket.send_data([[0], [0], [0, 0, 2], data_msg])
                    if tries == 11:
                        return
                    continue
                else:
                    print(
                        "Didnt find the Object. Try with another object or tune grasper height and width parameters in demo.py"
                    )
                    retry = False
                    continue

            bbox_x_min, bbox_y_min, bbox_x_max, bbox_y_max = bbox

            if tries == 1:
                print("Centering the robot")
                self.center_robot(bbox)
                tries += 1
                continue

            points = get_3d_points(self.cam)
            print('points: ',points.size)
            print('seg_mask: ',seg_mask.size)
            print('bbox: ',bbox.size)

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
                else:
                    print(
                        "Try with another object or tune grasper height and width parameters in demo.py"
                    )
                    retry = False

    def center_robot(self, bbox: Bbox):

        bbox_x_min, bbox_y_min, bbox_x_max, bbox_y_max = bbox

        bbox_center = [
            int((bbox_x_min + bbox_x_max) / 2),
            int((bbox_y_min + bbox_y_max) / 2),
        ]
        depth_obj = self.cam.depths[bbox_center[1], bbox_center[0]]
        print("Current bbox center is", bbox_center[1], bbox_center[0])
        print(
            f"{self.query} height and depth: {((bbox_y_max - bbox_y_min) * depth_obj)/self.cam.fy}, {depth_obj}"
        )

        dis = (bbox_center[0] - self.cam.cx) / self.cam.fx * depth_obj
        print("Current bbox_center, cx, fx, depth_obj is:", bbox_center[0], self.cam.cx, self.cam.fx, depth_obj)
        print(f"Base displacement {dis}")

        tilt = math.atan((bbox_center[1] - self.cam.cy) / self.cam.fy)
        print(f"Camera Tilt {tilt}")

        if self.cfgs.open_communication:
            data_msg = "Now you received the base and haed trans, good luck."
            self.socket.send_data([[-dis], [-tilt], [0, 0, 1], data_msg])

    def place(self, points: np.ndarray, seg_mask: np.ndarray) -> bool:
        points_x, points_y, points_z = points[:, :, 0], points[:, :, 1], points[:, :, 2]
        flat_x, flat_y, flat_z = (
            points_x.reshape(-1),
            -points_y.reshape(-1),
            -points_z.reshape(-1),
        )

        zero_depth_seg_mask = (
            (flat_x != 0) * (flat_y != 0) * (flat_z != 0) * seg_mask.reshape(-1)
        )
        flat_x = flat_x[zero_depth_seg_mask]
        flat_y = flat_y[zero_depth_seg_mask]
        flat_z = flat_z[zero_depth_seg_mask]

        colors = self.cam.colors.reshape(-1, 3)[zero_depth_seg_mask]

        points1 = np.stack([flat_x, flat_y, flat_z], axis=-1)

        cam_to_3d_rot = np.array(
            [
                [1, 0, 0],
                [0, math.cos(self.cam.head_tilt), math.sin(self.cam.head_tilt)],
                [0, -math.sin(self.cam.head_tilt), math.cos(self.cam.head_tilt)],
            ]
        )

        transformed_points = np.dot(points1, cam_to_3d_rot)

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
        )

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

        T_c_a = np.eye(4)

        return True

    def pickup(
        self,
        points: np.ndarray,
        seg_mask: np.ndarray,
        bbox: Bbox,
        crop_flag: bool = False,
    ):  
        print("init points is: ", points.size)
        print('pickup...... ')        
        points_x, points_y, points_z = points[:, :, 0], points[:, :, 1], points[:, :, 2]

        mask = (points_z > self.cfgs.min_depth) & (points_z < self.cfgs.max_depth)
        points = np.stack([points_x, points_y, points_z], axis=-1)
        points = points[mask].astype(np.float32)

        colors_m = self.cam.colors[mask].astype(np.float32)

        if self.cfgs.sampling_rate < 1:
            points, indices = sample_points(points, self.cfgs.sampling_rate)
            colors_m = colors_m[indices]

        xmin = points[:, 0].min()
        xmax = points[:, 0].max()
        ymin = points[:, 1].min()
        ymax = points[:, 1].max()
        zmin = points[:, 2].min()
        zmax = points[:, 2].max()
        lims = [xmin, xmax, ymin, ymax, zmin, zmax]
        if not any(lims):
            print("RGB-D can't get 3D points, please check RGB-D camera!")

        gg, cloud = self.grasping_model.get_grasp(points, colors_m, lims)

        print('gg: ',len(gg))
        if len(gg) == 0:
            print("No Grasp detected after collision detection!")
            return False

        gg = gg.nms().sort_by_score()
        filter_gg = GraspGroup()

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
                int(((grasp_center[1] * self.cam.fy) / grasp_center[2]) + self.cam.cy),
            )
            if ix < 0:
                ix = 0
            if iy < 0:
                iy = 0
            if ix >= W:
                ix = W - 1
            if iy >= H:
                iy = H - 1

            rotation_matrix = g.rotation_matrix
            cur_vec = rotation_matrix[:, 0]
            angle = math.acos(np.dot(ref_vec, cur_vec) / (np.linalg.norm(cur_vec)))
            if not crop_flag:
                score = g.score - 0.1 * (angle) ** 4
                score = g.score
            else:
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
        print('filter_gg: ',len(filter_gg))
        if len(filter_gg) == 0:
            print(
                "No grasp poses detected for this object try to move the object a little and try again"
            )
            return False

        projections_file_name = (
            self.cfgs.environment + "/" + self.query + "/anygrasp/grasp_projections.jpg"
        )
        image.save(projections_file_name)
        print(f"Saved projections of grasps at {projections_file_name}")
        filter_gg = filter_gg.nms().sort_by_score()

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

        T_c_a = np.eye(4)
        T_c_a[:3, :3] = filter_gg[0].rotation_matrix
        T_c_a[:3, 3] = filter_gg[0].translation

        pose_grasp = matrix_to_pose_stamped(T_c_a)
        print(pose_grasp)
        # while not rospy.is_shutdown():
        self.pose_pub.publish(pose_grasp.pose)
        store_pose_to_param_server(pose_grasp.pose, 'current_pose')
        



        print("Finish publish")
        print("-----Grasping-----")
        
        return True
