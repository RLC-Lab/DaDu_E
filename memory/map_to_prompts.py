# import json
# # 将结构化的语义地图文本转换为指定文本格式，用于prompts的输入
# def json_to_text(json_file, output_file):

#     with open(json_file, 'r') as f:
#         data = json.load(f)

   
#     formatted_lines = []
#     for obj in data:
#         name = obj['name']  
#         position = obj['position'].split()  
#         # 只提取 x 和 y 坐标
#         x, y = position[0], position[1]
#         formatted_lines.append(f"[{name}] [{x} {y}]")

#     with open(output_file, 'w') as f:
#         f.write("\n".join(formatted_lines))

#     print(f"Converted data saved to {output_file}")

# json_file = 'long_term_memory.json'  
# output_file = 'formatted_positions.txt'
# json_to_text(json_file, output_file)

import json

class JsonToTextFormatter:
    def __init__(self, json_file, output_file):
        """
        初始化 JsonToTextFormatter 类。
        
        :param json_file: 输入的 JSON 文件路径
        :param output_file: 输出的文本文件路径
        """
        self.json_file = json_file
        self.output_file = output_file

    def convert_to_text(self):
        """
        将 JSON 格式的语义地图数据转换为指定文本格式并保存。
        """
        with open(self.json_file, 'r') as f:
            data = json.load(f)

        formatted_lines = []
        for obj in data:
            name = obj['name']  
            position = obj['position'].split()  
            # 只提取 x 和 y 坐标
            x, y = position[0], position[1]
            formatted_lines.append(f"[{name}] [{x} {y}]")

        with open(self.output_file, 'w') as f:
            f.write("\n".join(formatted_lines))

        print(f"Converted data saved to {self.output_file}")
