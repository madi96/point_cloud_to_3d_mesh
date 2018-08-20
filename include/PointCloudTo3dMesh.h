#ifndef _POINT_CLOUD_TO_3D_MESH_H
#define _POINT_CLOUD_TO_3D_MESH_H

#include <rgbd_utils/rgbd_subscriber.hpp>
#include <rgbd_utils/rgbd_to_pointcloud.h>
#include <math.h>
#include <tf/transform_listener.h>
#include <Eigen/Geometry>  
#include <geometry_msgs/PointStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/impl/mls.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include <sstream>
class PointCloudTo3dMesh  {

public:
  PointCloudTo3dMesh();
  ~PointCloudTo3dMesh();

  void printInfo(std::string data) {std::cout<<"[INFO]::POINT_CLOUD_TO_3D_MESH : "<< data << std::endl;}
  void printWarn(std::string data) {std::cout<<"[WARN]::POINT_CLOUD_TO_3D_MESH : "<< data << std::endl;}
  void printErr(std::string data) {std::cout<<"[ERR]::POINT_CLOUD_TO_3D_MESH : "<< data << std::endl; exit(0);}
  void init();
  void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* junk);
  void planeModelSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
			      pcl::PointCloud<pcl::PointXYZ>::Ptr inlinersCloud, 
			      pcl::PointCloud<pcl::PointXYZ>::Ptr outlinersCloud);

  pcl::PointCloud<pcl::PointNormal>::Ptr normalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud);
  pcl::PolygonMesh fastTriangulation(pcl::PointCloud<pcl::PointNormal>::Ptr cloudNormalsPtr);
  void outliersRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, 
		       pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud);
  void smoothingAndNormalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
				    pcl::PointCloud<pcl::PointNormal>::Ptr movingLeastSquarePointsPtr);
  void transformToCloudOrigin(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, 
						  pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud);
  void get3DMeshFromPointCloud(); 
  void visualizeMesh();
  bool getViewerStatus(){
    return viewerTerminated;
  }


  ros::NodeHandle nh_;

private:
  typedef pcl::PointXYZ PointT;
  pcl::PolygonMesh mesh_;
  const tf::TransformListener listener_;
  bool update_ = false;
  bool viewer_empty_ = true;
  bool viewerTerminated=false;
  std::unique_ptr<rgbd_utils::RGBD_Subscriber> rgbd_sub_;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
  std::unique_ptr<ros::Publisher> point_cloud_pub_;
  int mesh_id_=1;
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
  bool is_finish = false;
};
#endif
