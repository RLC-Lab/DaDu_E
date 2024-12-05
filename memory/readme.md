# long_term_memory.py
从gazebo的.world文件中提取结构化的语义地图，保存至long_term_memory.json

# map_to_prompts.py
将long_term_memory.json中结构化的语义地图转化为prompts中的-MAP- ...... -MAP-部分,将文本结果保存至 formattered_positions.txt

# prompts_map_example.txt
我们目前使用的测试场景的-MAP- -MAP-

# short_term_memory.py
动态添加short_memory的内容（通过ground_truth），将short_memory的结果保存至short_memory.json

# task_memory_pair.py
比较输入的task与short_term_memory.json中的物体的语义相似度，如果相似度大于 0.8，则输出一段有关的memory prompts到short_term_memory_prompts.txt

# 