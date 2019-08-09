#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>

#include <pcl/filters/passthrough.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <geograsp/GeoGrasp.h>

pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Cloud viewer"));

// callback signature
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr & inputCloudMsg) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::fromROSMsg(*inputCloudMsg, *cloud);//将输入的ros点云，转换成pcl的点云格式（也可以转换回来）

  // Remove NaN values and make it dense
  std::vector<int> nanIndices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, nanIndices);//将NAN的点移除

  // Save to file
  //pcl::io::savePCDFileBinary("original-cloud.pcd", *cloud);

  // Remove background points
  pcl::PassThrough<pcl::PointXYZRGB> ptFilter;//定义一个滤波器
  ptFilter.setInputCloud(cloud);//设置输入的要处理的点云
  ptFilter.setFilterFieldName("z");//需要处理的坐标为z轴
  ptFilter.setFilterLimits(0.0, 1.5);
  ptFilter.filter(*cloud);//进行滤波
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ori(new pcl::PointCloud<pcl::PointXYZRGB>());
  //cloud_ori=cloud;
  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZRGB> sacSegmentator;//定义一个pcl分类器
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);//生成的参数
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);//生成的参数
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPlane(new pcl::PointCloud<pcl::PointXYZRGB>());//用来装平面点云的容器
  sacSegmentator.setOptimizeCoefficients (true);//允许参数优化，默认为false，也可以不设置，效果差不多
  sacSegmentator.setModelType(pcl::SACMODEL_PLANE);//需要分类的模型为平面
  sacSegmentator.setMethodType(pcl::SAC_RANSAC);//方法为RANSAC，也有其他方法
  sacSegmentator.setMaxIterations(100);//迭代次数
  sacSegmentator.setDistanceThreshold(0.01);//最远距离，这里可以理解为，找到地面后，与地面距离在1cm的也都是地面
  sacSegmentator.setInputCloud(cloud);//设置输入的要处理的点云
  sacSegmentator.segment(*inliers, *coefficients);

  // Remove the planar inliers, extract the rest
  pcl::ExtractIndices<pcl::PointXYZRGB> indExtractor;
  indExtractor.setInputCloud(cloud);
  indExtractor.setIndices(inliers);
  indExtractor.setNegative(false);//选择false，提取出来的就是平面

  // Get the points associated with the planar surface
  indExtractor.filter(*cloudPlane);//将提取的点云放到cloudplane容器

  // Remove the planar inliers, extract the rest
  indExtractor.setNegative(true);//选择true，提取出来的就是非平面点云
  indExtractor.filter(*cloud);//将提取的点云放到cloud容器，这里实际上覆盖了原先的点云了
  //cloud=cloud_ori;
  // Creating the KdTree object for the search method of the extraction
  //使用kdtree，用欧几里德算法，寻找紧邻点云，进行分类
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(cloud);

  std::vector<pcl::PointIndices> clusterIndices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ecExtractor;
  ecExtractor.setClusterTolerance(0.01);
  ecExtractor.setMinClusterSize(300);
  //ecExtractor.setMaxClusterSize(25000);
  ecExtractor.setSearchMethod(tree);
  ecExtractor.setInputCloud(cloud);
  ecExtractor.extract(clusterIndices);

  if (clusterIndices.empty()) {
    // Visualize the result
    //一般不会进这个if
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> planeColor(cloudPlane, 0, 255, 0);

    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "Main cloud");
    viewer->addPointCloud<pcl::PointXYZRGB>(cloudPlane, planeColor, "Plane");

    viewer->spinOnce();
  }
  else {
    std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin();
    int objectNumber = 0;

    viewer->removeAllPointClouds();//移除所有显示的点云，不做的话，可能会有奇怪的点进入
    viewer->removeAllShapes();//同上
    //给的初始颜色，如果可以的话，个人还是比较想用好一点的配色
    int color_num1=76;
    int color_num2=180;
    int color_num3=231;
    int color_type=1;
    // Every cluster found is considered an object
    for (it = clusterIndices.begin(); it != clusterIndices.end(); ++it) {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr objectCloud(new pcl::PointCloud<pcl::PointXYZRGB>());//用来存放每一类物体点云

      for (std::vector<int>::const_iterator pit = it->indices.begin(); 
          pit != it->indices.end(); ++pit)
        objectCloud->points.push_back(cloud->points[*pit]);//
      //参数设定
      objectCloud->width = objectCloud->points.size();
      objectCloud->height = 1;
      objectCloud->is_dense = true;

      // Create and initialise GeoGrasp
      GeoGrasp geoGraspPoints;//生成类
      //区分点云
      geoGraspPoints.setBackgroundCloud(cloudPlane);//区分点云
      geoGraspPoints.setObjectCloud(objectCloud);

      // Calculate grasping points
      //计算
       geoGraspPoints.compute();

      // Extract best pair of points
      //找到最优抓取并显示出来，并会打印出来
       GraspConfiguration bestGrasp = geoGraspPoints.getBestGrasp();

      // Visualize the result
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(objectCloud);
      //定义显示物体点云的颜色
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> obj_color(cloudPlane, 
        color_num1, color_num2, color_num3);
      //备选，用来显示平面的颜色
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> planeColor(cloudPlane, 
        0, 155, 0);
      //显示平面的颜色，这样写就是用默认颜色
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> planeRGB(cloudPlane);

      std::string objectLabel = "";
      std::ostringstream converter;

      converter << objectNumber;
      objectLabel += converter.str();
      objectLabel += "-";

      //显示颜色
      //viewer->addPointCloud<pcl::PointXYZRGB>(objectCloud, rgb, objectLabel + "Object");
      viewer->addPointCloud<pcl::PointXYZRGB>(objectCloud, obj_color, objectLabel + "Object");
      //viewer->addPointCloud<pcl::PointXYZRGB>(cloudPlane, planeColor, objectLabel + "Plane");
      //viewer->addPointCloud<pcl::PointXYZRGB>(cloudPlane, planeRGB, objectLabel + "Plane");
      //显示抓手的两点
      viewer->addSphere(bestGrasp.firstPoint, 0.01, 0, 0, 255, objectLabel + "First best grasp point");
      viewer->addSphere(bestGrasp.secondPoint, 0.01, 255, 0, 0, objectLabel + "Second best grasp point");
      //切换颜色的非重要部分，可以根据需求更改
      objectNumber++;
      switch (color_type)
      {
        case 1:
          color_type++;
          color_num1=255;
          color_num2=192;
          color_num3=159;
          break;
        case 2:
          color_type++;
          color_num1=255;
          color_num2=238;
          color_num3=147;
          break;
        case 3:
          color_type++;
          color_num1=226;
          color_num2=219;
          color_num3=190;
          break;
        case 4:
          color_type++;
          color_num1=163;
          color_num2=163;
          color_num3=128;         
          break; 
        case 5:
          color_type=1;
          color_num1=76;
          color_num2=180;
          color_num3=231;
          break;
      }
      //测试颜色显示
      //std::cout << color_num2 << "   " << "\n";
      
    }

    // viewer->spinOnce();
    while (!viewer->wasStopped())
      viewer->spinOnce(100);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "cloud_processor");

  viewer->initCameraParameters();
  viewer->addCoordinateSystem(0.1);

  ros::NodeHandle n("~");
  std::string cloudTopic;
  
  n.getParam("topic", cloudTopic);
//输入的点云是ros的sensor_msgs::PointCloud类型
  ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>(cloudTopic, 1, cloudCallback);

  ros::spin();

  return 0;
}
