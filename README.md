<h1 align="center">DaDu-E: Rethinking the Role of Large Language Model in Robotic Computing Pipeline</h1>

[![arXiv](https://img.shields.io/badge/arXiv-Paper-<COLOR>.svg)]([https://arxiv.org/abs/2407.04292](https://arxiv.org/abs/2412.01663))


**This project is working in process, our fully code will be coming soon**

## Building and Installation

### Software Requirement

* ROS Noetic on Ubuntu 20.04
* Gazebo multi-robot simulator, version 11.11.0
* Python 3.9.18
* GCC 10.5.0
* CUDA 11.8

### Installation

* Firstly, you need to install the required packages:

```
conda env create -f environment.yml
```

* Then, you need to install [AnyGrasp](https://github.com/graspnet/anygrasp_sdk) and apply for license from offical repo, details can be found in [AnyGrasp License Registration](https://github.com/graspnet/anygrasp_sdk/blob/main/license_registration/README.md).

* You also need to download the pre-trained models from [LangSAM](https://github.com/luca-medeiros/lang-segment-anything), [LLaMA](https://huggingface.co/meta-llama/Llama-3.1-8B-Instruct),

## Env Formulation

In the eval code, we set 4 actions as **goto**, **pick up**, **place** and **done**, you can follow action list in eval list to execute actions.

### Project Architecture

* In `executor_a.py` file, we setup the computing pipeline
* In `navigation.py` file, we send target pose to move_base and navigate
* In `grasping.py` file, we define grasping functions
* In `grasp_utils.py` file, we define grasping functions, including moving forward and backward before and after grasping for robust grasping.
* In `distance_calculator.py` file, we compute the navigation distance.
* In `./example_data`, we save pics duiring grasping

## Run the codes

* Firstly, init the arm pose, avoid the collision between the arm and the object:

```shell
conda activate dadu-e
python init_detect_arm.py
```

* Meanwhile, prepare grasping init, you should run this script duiring grasping:

```shell
conda activate dadu-e
python test_camera_convert.py
```

* After that, run the pipeline manually:

```shell
conda activate dadu-e
python executor_a.py
```

during this processing, user need to input the target postion  

```
action_type(1_navi 2_pick 3_place 4_done): 
1(manually choose)
param: -4.66216 3.40295 (example postion of a table target)
goto[table][-4.66216 3.40295]
navigatoin
goto[table][-4.66216 3.40295]
-4.66216 3.40295
move_base:  ['-4.66216', '3.40295']
```

Then you can arrive at the target position and select pick up object.

* Or you can run the pipeline automatically:

```shell
conda activate dadu-e
python executor.py
```

During this process, you should replace your OpenAI Key at `api_key`

* deal with navigation planning error:

```shell
rosparam set use_sim_time true
```

## Thanks

Thanks the code of [OK-robot](https://github.com/ok-robot/ok-robot), the code of [AnyGrasp](https://github.com/graspnet/anygrasp_sdk) and the code of [langSAM](https://github.com/luca-medeiros/lang-segment-anything)

If you find our projct useful, please cite our paper:
```
@article{sun2024dadu,
  title={DaDu-E: Rethinking the Role of Large Language Model in Robotic Computing Pipeline},
  author={Wenhao Sun and Sai Hou and Zixuan Wang and Bo Yu and Shaoshan Liu and Xu Yang and Shuai Liang and Yiming Gan and Yinhe Han},
  journal={arXiv preprint arXiv:2412.01663},
  year={2024}
}
```
