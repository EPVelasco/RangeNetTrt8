# Rangenet 
This project is based on the [RangeNetTrt8](https://github.com/Natsu-Akatsuki/RangeNetTrt8) repository, which is based on [rangenet_lib](https://github.com/PRBonn/rangenet_lib).The algorithm is tested on a docker with Ubuntu 20.04, TensortRT8, cdnn 8.2.4 and ROS-Noetic.

## File tree

└── **RangeNetTrt8**   
　└── **Dockerfile**  
　├── **darknet53**   
　└── **libtorch** 

## 方法一：docker（改进ing）

### 依赖

- nvidia driver

- [docker](https://ambook.readthedocs.io/zh/latest/docker/rst/docker-practice.html#docker)
- [nvidia-container2](https://ambook.readthedocs.io/zh/latest/docker/rst/docker-practice.html#id4)

- 创建工作空间

```bash
$ git clone https://github.com/Natsu-Akatsuki/RangeNetTrt8 ~/docker_ws/RangeNetTrt8/src
```

- 下载onnx模型

```bash
$ wget -c https://www.ipb.uni-bonn.de/html/projects/semantic_suma/darknet53.tar.gz -O ~/docker_ws/RangeNetTrt8/src/darknet53.tar.gz
$ cd ~/docker_ws/RangeNetTrt8/src && tar -xzvf darknet53.tar.gz
```

### 安装

- 拉取镜像（镜像大小约为20G，需预留足够的空间）

```bash
$ docker pull registry.cn-hangzhou.aliyuncs.com/gdut-iidcc/rangenet:1.0
```

- 创建容器

```bash
$ cd ~/docker_ws/RanageNetTrt8/src
$ bash script/build_container_rangenet.sh

# 编译和执行
(container root) $ cd /docker_ws/RanageNetTrt8
# 编译前需加CMakelists_v2.txt改名字为CMakelists.txt
(container root) $ catkin_build
(container root) $ source devel/setup.bash

# dem01:
(container root) $ roslaunch rangenet_plusplus rangenet.launch
# 播放包（该模型仅适用于kitti数据集，需自行下载包文件和修改该launch文档）
(container root) $ roslaunch rangenet_plusplus rosbag.launch

# demo2:
# need modify the example/infer.yaml first
(container root) $ ./devel/lib/rangenet_plusplus/infer
```

**NOTE**

首次运行生成TensorRT模型运行需要一段时间

![image-20220330012729619](https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/image-20220330012729619.png)

## 方法二：native PC

### 依赖

- **ros1**
- **nvidia driver**

- **TensorRT 8.2.3**（tar包下载）, **cuda_11.4.r11.4**,  **cudnn 8.2.4**

- apt package and python package

```bash
$ sudo apt install build-essential python3-dev python3-pip apt-utils git cmake libboost-all-dev libyaml-cpp-dev libopencv-dev python3-empy
$ pip install catkin_tools trollius numpy
```

- 创建工作空间

```bash
$ git clone https://github.com/Natsu-Akatsuki/RangeNetTrt8 ~/RangeNetTrt8/src
```

- 下载**onnx**模型

```bash
$ wget -c https://www.ipb.uni-bonn.de/html/projects/semantic_suma/darknet53.tar.gz -O ~/RangeNetTrt8/src/darknet53.tar.gz
$ cd ~/RangeNetTrt8/src && tar -xzvf darknet53.tar.gz
```

- 下载**libtorch**

```bash
$ wget -c https://download.pytorch.org/libtorch/cu113/libtorch-cxx11-abi-shared-with-deps-1.10.2%2Bcu113.zip -O libtorch.zip
$ unzip libtorch.zip
```

TIP: 需导入各种环境变量到`~/.bashrc`

```bash
# example
export PATH="/home/helios/.local/bin:$PATH"
CUDA_PATH=/usr/local/cuda/bin
TENSORRT_PATH=${HOME}/application/TensorRT-8.2.3.0/bin
CUDA_LIB_PATH=/usr/local/cuda/lib64
TENSORRT_LIB_PATH=${HOME}/application/TensorRT-8.2.3.0/lib
PYTORCH_LIB_PATH=${HOME}/application/libtorch/lib
export PATH=${PATH}:${CUDA_PATH}:${TENSORRT_PATH}
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${CUDA_LIB_PATH}:${TENSORRT_LIB_PATH}:${PYTORCH_LIB_PATH}
```

### 安装

- 修改CMakeLists：修改其中的TensorRT, libtorch等依赖库的路径
- 编译和执行

```bash
# 编译和执行
$ cd ~/RanageNetTrt8
$ catkin_build
$ source devel/setup.bash

# dem01:
$ roslaunch rangenet_plusplus rangenet.launch
# 播放包（该模型仅适用于kitti数据集，需自行下载包文件和修改该launch文档）
$ roslaunch rangenet_plusplus rosbag.launch

# demo2:
# need modify the example/infer.yaml first
$ ./devel/lib/rangenet_plusplus/infer
```

<img src="https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/ros.gif" alt="img" style="zoom:67%;" />

**NOTE**

首次运行生成TensorRT模型运行需要一段时间

## Q&A

- 模型解析出问题（查看是否下载的onnx模型是否完整，是否在解压缩时broken了）

> [libprotobuf ERROR google/protobuf/text_format.cc:298] Error parsing text-format onnx2trt_onnx.ModelProto: 1:1: Invalid control characters encountered in text. 
> [libprotobuf ERROR google/protobuf/text_format.cc:298] Error parsing text-format onnx2trt_onnx.ModelProto: 1:14: Message type "onnx2trt_onnx.ModelProto" has no field named "pytorch". Message type "onnx2trt_onnx.ModelProto" has no field named "pytorch"

## Citations

If you use this library for any academic work, please cite the original [paper](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/milioto2019iros.pdf).

```
@inproceedings{milioto2019iros,
  author    = {A. Milioto and I. Vizzo and J. Behley and C. Stachniss},
  title     = {{RangeNet++: Fast and Accurate LiDAR Semantic Segmentation}},
  booktitle = {IEEE/RSJ Intl.~Conf.~on Intelligent Robots and Systems (IROS)},
  year      = 2019,
  codeurl   = {https://github.com/PRBonn/lidar-bonnetal},
  videourl  = {https://youtu.be/wuokg7MFZyU},
}
```

If you use SuMa++, please cite the corresponding [paper](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/chen2019iros.pdf):

```
@inproceedings{chen2019iros, 
  author    = {X. Chen and A. Milioto and E. Palazzolo and P. Giguère and J. Behley and C. Stachniss},
  title     = {{SuMa++: Efficient LiDAR-based Semantic SLAM}},
  booktitle = {Proceedings of the IEEE/RSJ Int. Conf. on Intelligent Robots and Systems (IROS)},
  year      = {2019},
  codeurl   = {https://github.com/PRBonn/semantic_suma/},
  videourl  = {https://youtu.be/uo3ZuLuFAzk},
}
```

## License

Copyright 2019, Xieyuanli Chen, Andres Milioto, Jens Behley, Cyrill Stachniss, University of Bonn.

This project is free software made available under the MIT License. For details see the LICENSE file.
