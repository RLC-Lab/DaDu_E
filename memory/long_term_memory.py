# import xml.etree.ElementTree as ET
# import json

# # 从gazebo的world中提取结构化的语义地图，保存至.json
# def extract_world_objects(world_file, output_file):

#     tree = ET.parse(world_file)
#     root = tree.getroot()

#     ns = {'sdf': 'http://www.gazebosim.org/schemas'}

#     objects = []

#     for model in root.findall(".//model", ns):
#         model_name = model.get('name', 'unknown_model')
#         model_pose = model.find("pose", ns)
#         model_position = model_pose.text if model_pose is not None else "0 0 0 0 0 0"
#         objects.append({'name': model_name, 'position': model_position})

#         for collision in model.findall(".//collision", ns):
#             collision_name = collision.get('name', 'unknown_collision')
#             collision_pose = collision.find("pose", ns)
#             collision_position = collision_pose.text if collision_pose is not None else "0 0 0 0 0 0"
#             objects.append({'name': collision_name, 'position': collision_position})


#     with open(output_file, 'w') as f:
#         json.dump(objects, f, indent=4)

#     print(f"Objects extracted and saved to {output_file}")

# world_file = '/home/hs/disk1/wangzixuan/catkin_ws/src/husky_ur3_simulator/husky_ur3_gazebo/worlds/HRI_lab.world'  # .world path
# output_file = 'long_term_memory.json'
# extract_world_objects(world_file, output_file)

import xml.etree.ElementTree as ET
import json

class WorldExtractor:
    def __init__(self, world_file, output_file):
        """
        初始化 WorldExtractor 类。
        
        :param world_file: Gazebo .world 文件路径
        :param output_file: 提取结果的 JSON 文件路径
        """
        self.world_file = world_file
        self.output_file = output_file

    def extract_objects(self):
        """
        从 Gazebo 的 .world 文件中提取语义地图信息，并保存为 JSON 格式。
        """
        tree = ET.parse(self.world_file)
        root = tree.getroot()

        ns = {'sdf': 'http://www.gazebosim.org/schemas'}

        objects = []

        for model in root.findall(".//model", ns):
            model_name = model.get('name', 'unknown_model')
            model_pose = model.find("pose", ns)
            model_position = model_pose.text if model_pose is not None else "0 0 0 0 0 0"
            objects.append({'name': model_name, 'position': model_position})

            for collision in model.findall(".//collision", ns):
                collision_name = collision.get('name', 'unknown_collision')
                collision_pose = collision.find("pose", ns)
                collision_position = collision_pose.text if collision_pose is not None else "0 0 0 0 0 0"
                objects.append({'name': collision_name, 'position': collision_position})

        with open(self.output_file, 'w') as f:
            json.dump(objects, f, indent=4)

        print(f"Objects extracted and saved to {self.output_file}")
