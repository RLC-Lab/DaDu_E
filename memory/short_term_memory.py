# import xml.etree.ElementTree as ET
# import json
# import os

# def extract_objects(world_file):
#     """
#     从 .world 文件中提取物体的名称和位置
#     """
#     tree = ET.parse(world_file)
#     root = tree.getroot()

#     ns = {'sdf': 'http://www.gazebosim.org/schemas'}


#     objects = {}
#     for model in root.findall(".//model", ns):
#         model_name = model.get('name', 'unknown_model')
#         pose_element = model.find("pose", ns)
#         position = pose_element.text.split()[:3] if pose_element is not None else ["0", "0", "0"]
#         objects[model_name] = position

#     return objects

# def find_changed_objects(prev_objects, current_objects):
#     """
#     比较前后两次 .world 文件中的物体位置，找出发生变化的物体
#     """
#     changed_objects = {}
#     for name, position in current_objects.items():
#         if name not in prev_objects or prev_objects[name] != position:
#             changed_objects[name] = position
#     return changed_objects

# def update_short_term_memory(changed_objects, short_term_memory_file):
#     """
#     将发生变化的物体累计保存到 short_term_memory.json
#     """
#     if os.path.exists(short_term_memory_file):
#         with open(short_term_memory_file, 'r') as f:
#             short_term_memory = json.load(f)
#     else:
#         short_term_memory = {}

#     short_term_memory.update(changed_objects)
#     with open(short_term_memory_file, 'w') as f:
#         json.dump(short_term_memory, f, indent=4)

#     print(f"Updated short_term_memory.json with {len(changed_objects)} changed objects.")

# def compare_world_files(prev_world_file, current_world_file, short_term_memory_file):
#     """
#     比较前后两次 .world 文件中的物体位置变化并累计保存到 short_term_memory.json
#     """
    
#     prev_objects = extract_objects(prev_world_file)
#     current_objects = extract_objects(current_world_file)

#     changed_objects = find_changed_objects(prev_objects, current_objects)

#     update_short_term_memory(changed_objects, short_term_memory_file)

# prev_world_file = '/home/hs/disk1/wangzixuan/catkin_ws/src/husky_ur3_simulator/husky_ur3_gazebo/worlds/HRI_lab.world'  
# current_world_file = '/home/hs/disk1/wangzixuan/catkin_ws/src/husky_ur3_simulator/husky_ur3_gazebo/worlds/HRI_lab.world'  
# short_term_memory_file = 'short_term_memory.json'

# compare_world_files(prev_world_file, current_world_file, short_term_memory_file)

import xml.etree.ElementTree as ET
import json
import os

class WorldComparator:
    def __init__(self, prev_world_file, current_world_file, short_term_memory_file):
        """
        初始化 WorldComparator 类。
        
        :param prev_world_file: 上一次 .world 文件路径
        :param current_world_file: 当前的 .world 文件路径
        :param short_term_memory_file: 保存短期记忆的 JSON 文件路径
        """
        self.prev_world_file = prev_world_file
        self.current_world_file = current_world_file
        self.short_term_memory_file = short_term_memory_file

    def extract_objects(self, world_file):
        """
        从 .world 文件中提取物体的名称和位置。
        """
        tree = ET.parse(world_file)
        root = tree.getroot()

        ns = {'sdf': 'http://www.gazebosim.org/schemas'}

        objects = {}
        for model in root.findall(".//model", ns):
            model_name = model.get('name', 'unknown_model')
            pose_element = model.find("pose", ns)
            position = pose_element.text.split()[:3] if pose_element is not None else ["0", "0", "0"]
            objects[model_name] = position

        return objects

    def find_changed_objects(self, prev_objects, current_objects):
        """
        比较前后两次 .world 文件中的物体位置，找出发生变化的物体。
        """
        changed_objects = {}
        for name, position in current_objects.items():
            if name not in prev_objects or prev_objects[name] != position:
                changed_objects[name] = position
        return changed_objects

    def update_short_term_memory(self, changed_objects):
        """
        将发生变化的物体累计保存到 short_term_memory.json。
        """
        if os.path.exists(self.short_term_memory_file):
            with open(self.short_term_memory_file, 'r') as f:
                short_term_memory = json.load(f)
        else:
            short_term_memory = {}

        short_term_memory.update(changed_objects)
        with open(self.short_term_memory_file, 'w') as f:
            json.dump(short_term_memory, f, indent=4)

        print(f"Updated {self.short_term_memory_file} with {len(changed_objects)} changed objects.")

    def compare_world_files(self):
        """
        比较前后两次 .world 文件中的物体位置变化并累计保存到 short_term_memory.json。
        """
        prev_objects = self.extract_objects(self.prev_world_file)
        current_objects = self.extract_objects(self.current_world_file)

        changed_objects = self.find_changed_objects(prev_objects, current_objects)

        self.update_short_term_memory(changed_objects)
