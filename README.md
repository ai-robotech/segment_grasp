# segment_grasp
This demo can segmengt the point cloud, remove the background and compute the grasp position
# 所作工作

## 此程序可以完成，去除点云的平面背景并对物体进行分类，再进行抓取规划

> 主要参考了 https://github.com/yayaneath/GeoGrasp，并在其基础上进行了修改
>
> 程序主要是使用pcl中实现的方法来对点云图像进行处理。
## 环境
> pcl-1.7.2

> ROS Kinetic
## pcl下载安装
> 下载pcl-1.7.2。解压后再文件路径下

```

    mkdir build
    cd build 
    cmake ..
    make -j4
    
```

> 安装后进入工作空间的src，下载代码 

```
git clone https://github.com/yayaneath/GeoGrasp.git
```
## 编译运行
> 编译完成后执行（打开Kinect相机的代码，也可以是其他的命令，能发布点云信息到指定话题即可）

```
roslaunch kinect2_bridge kinect2_bridge.launch
```

> Kinect会不断发布话题消息，点云的话题有/kinect2/sd/points，/kinect2/qhd/points，我们要监听的是后者

```
rosrun geograsp cloud_processor _topic:="/kinect2/qhd/points"
```


##效果
> Kinect所得到的点云图像如下:

<img src="/1.jpg" width="445"> 

> 通过pcl中的分类方式，我们将点云通过RANSAC分为，平面和非平面，并将两者存在不同的点云数据中。只显示物体点云，效果如下:

<img src="/2.jpg" width="445">

> 由于kinect限制，对于表面会有一定反光的物体，生成点云的效果并不佳。我们再对物体进行分类，分类后使用Geograsp使用的方法对每个物体找出最优的抓取点。分类所使用的方法是欧几里得聚类。是通过距离来区分物体。缺点是对于相连的物体两个物体会将其认为是一个物体。且对点云的精度有要求。分类的效果如下

<img src="/3.jpg" width="445">
