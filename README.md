# Rangenet 
This project is based on the [RangeNetTrt8](https://github.com/Natsu-Akatsuki/RangeNetTrt8) repository, which is based on [rangenet_lib](https://github.com/PRBonn/rangenet_lib).The algorithm is tested on a docker with Ubuntu 20.04, TensortRT8, cdnn 8.2.4 and ROS-Noetic.

## File tree
└── ~/your_ws   
　└── RangeNetTrt8  
　　├── Dockerfile.txt   
　　├── darknet53   
　　└── libtorch   
  
The hardware I have tested with is a core i7 9th Gen computer with a GTX1660 6GB graphics card. The native operating system is Ubuntu 18.04. But with docker you can test it on any ubuntu system. :)

It is necessary to have [docker](https://www.digitalocean.com/community/tutorials/how-to-install-and-use-docker-on-ubuntu-18-04) installed and the [nvidia-docker2](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)

## Topics
### Suscribed Topics
*~/raw_pointcloud* Input Point Cloud message. ([sensor_msgs/PointCloud2](http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/PointCloud2.html))

### Published Topics
*~/label_pointcloud* Segmented output point cloud. ([sensor_msgs/PointCloud2](http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/PointCloud2.html))

## Requisites
- Download the [dockerfile](https://github.com/EPVelasco/RangeNetTrt8/blob/master/Dockerfile)
- Download onnx model 
```
    cd ~/your_ws/RangeNetTrt8
    wget -c https://www.ipb.uni-bonn.de/html/projects/semantic_suma/darknet53.tar.gz 
    tar -xzvf darknet53.tar.gz
    rm -r darknet53.tar.gz
```
- Download libtorch 
```
    cd ~/your_ws/RangeNetTrt8
    wget -c https://download.pytorch.org/libtorch/cu113/libtorch-cxx11-abi-shared-with-deps-1.10.2%2Bcu113.zip
    unzip libtorch.zip
    rm -r libtorch.zip
```

## Pull docker image
This will take several minutes, get a docker FROM nvidia/cuda:11.4.0-cudnn8-devel-ubuntu20.04
The image size is about 20Gb, need to set aside enough space.
```
    sudo docker build -t randala_net .
```
## Create container
```
    sudo docker run --shm-size=1g --ulimit memlock=-1 --ulimit stack=67108864 --rm -it --name randala_net --net host --gpus all --cpuset-cpus="0" -v ~/:/your_name randala_net
```
## Inside the docker 
Run in docker terminal. It takes a few minutes to get the model.trt file the first time
```
    cd
    cd ros_ws 
    ./devel/lib/rangenet_plusplus/infer
```
When the .trt file is ready, run this command
```
    source devel/setup.bash
    roslaunch rangenet_plusplus rangenet.launch
```
## Rosbag play
You can use the [rosbag](https://drive.google.com/file/d/1Sdh-m30VW-PFf7fCqD-kIljHiUjoJh2_/view?usp=share_link) of KITTI 03 scenario to test the code.
In a new terminal outside of docker put: 

```
    rosbag play '~/your/kitti/rosbag/path/03.bag'  /velodyne_points:=/raw_pointcloud
```

<img src="https://natsu-akatsuki.oss-cn-guangzhou.aliyuncs.com/img/ros.gif" alt="img" style="zoom:67%;" />

## Q&A from original [RangeNetTrt8](https://github.com/Natsu-Akatsuki/RangeNetTrt8)**

- Problems with model parsing (check if the downloaded onnx model is complete and if it is broken when unpacking)

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
