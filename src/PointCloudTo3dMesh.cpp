#include <PointCloudTo3dMesh.h>

PointCloudTo3dMesh::PointCloudTo3dMesh(){
  init();
}

PointCloudTo3dMesh::~PointCloudTo3dMesh(){
  _rgbdSub.reset();
  _pointCloudPub.reset();
  _cloud.reset();
  _viewer.reset();
}


void PointCloudTo3dMesh::init() {

  _cloud.reset(new pcl::PointCloud<PointT>);
      
  XmlRpc::XmlRpcValue params;

  _nh.getParam("/PointCloudTo3dMesh", params);
  std::cout<<"params:"<<params<<std::endl;
  _rgbdSub.reset(new rgbd_utils::RGBD_Subscriber(
                      params["rgbInfoTopic"],
                      params["rgbTopic"],
                      params["depthInfoTopic"],
                      params["depthTopic"],
                      _nh));
    
  _pointCloudPub.reset(new ros::Publisher(_nh.advertise<sensor_msgs::PointCloud2>("PointCloud",5)));

  // Init the viewer
  _viewer.reset( new pcl::visualization::PCLVisualizer ("3D Viewer ('Esc': close viewer; 'u': update mesh; 's': save mesh)"));
  _viewer->setBackgroundColor (0, 0, 0);
  _viewer->addCoordinateSystem (1.0);
  _viewer->initCameraParameters ();
  _viewer->registerKeyboardCallback(&PointCloudTo3dMesh::keyboardEventOccurred, *this);
}


void PointCloudTo3dMesh::keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* junk){

  std::string pressed = event.getKeySym();
  if( event.keyDown() ){
    if (pressed == "Escape"){
      printInfo("Viewer Callback: 'Esc' was pressed -> closing the viewer");
      _viewer->close();
      viewerTerminated=true;

    } else if (pressed == "u"){
      printInfo("Viewer Callback: 'u' was pressed -> updating the 3D mesh");
      _update = true;
      meshID ++;

    }else if (pressed == "s"){
      printInfo("Viewer Callback: 's' was pressed -> saving the 3D mesh");
      pcl::io::saveVTKFile ("3DMeshFromPointCloud.vtk", _mesh);
      pcl::io::savePolygonFileSTL("3DMeshFromPointCloud.stl", _mesh);
	
    }
  }
} 

void PointCloudTo3dMesh::planeModelSegmentation(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
					    pcl::PointCloud<pcl::PointXYZ>::Ptr inlinersCloud, 
					    pcl::PointCloud<pcl::PointXYZ>::Ptr outlinersCloud){
  printInfo("Plane segmentation");
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  std::stringstream ss;
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
    
  // Setting Optimisation coefs: Optional
  seg.setOptimizeCoefficients (true);

  // Setting the model and the method type and the threshold for segmentation: Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.1);

  seg.setInputCloud (inputCloud);
  seg.segment (*inliers, *coefficients);

  if (inliers->indices.size () == 0)
    {
      PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    }

  ss << "Model coefficients: " 
     << coefficients->values[0] << " " 
     << coefficients->values[1] << " "
     << coefficients->values[2] << " " 
     << coefficients->values[3] << " "
     <<"Model inliers: " << inliers->indices.size ();
  printInfo(ss.str());


  // Set the inliners cloud size
  inlinersCloud->points.resize (inliers->indices.size ()); 
  // Fill in the inliners cloud
  for (size_t i = 0; i < inliers->indices.size (); ++i){
    inlinersCloud->points[i].x =    inputCloud->points[inliers->indices[i]].x;
    inlinersCloud->points[i].y =    inputCloud->points[inliers->indices[i]].y;
    inlinersCloud->points[i].z =    inputCloud->points[inliers->indices[i]].z;
  }


  // extract outliners from the cloud
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(inputCloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*outlinersCloud);
    
}

pcl::PointCloud<pcl::PointNormal>::Ptr PointCloudTo3dMesh::normalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud){
  printInfo("Estimating normals");
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> nEstimation;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  // Defining and initiating the KD search tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (inputCloud);

  // Setting normal estimation parameters
  nEstimation.setInputCloud (inputCloud);
  nEstimation.setSearchMethod (tree);
  nEstimation.setKSearch (20);
  nEstimation.compute (*normals);

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloudWithNormals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*inputCloud, *normals, *cloudWithNormals);

  return cloudWithNormals;
}
  
pcl::PolygonMesh PointCloudTo3dMesh::fastTriangulation(pcl::PointCloud<pcl::PointNormal>::Ptr cloudNormalsPtr){
  printInfo("Computing triangles mesh from the point cloud");

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh trianglesMesh;

  // Create KD-search tree
  pcl::search::KdTree<pcl::PointNormal>::Ptr kDTree (new pcl::search::KdTree<pcl::PointNormal>);
  kDTree->setInputCloud (cloudNormalsPtr);


  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.025);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors(150);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  // gp3.setInputCloud (cloud_with_normals);
  gp3.setInputCloud (cloudNormalsPtr);
  gp3.setSearchMethod (kDTree);
  gp3.reconstruct (trianglesMesh);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();

  return trianglesMesh;
}

void PointCloudTo3dMesh::outliersRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud, 
				     pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud){

  printInfo("Removing outliers from the point cloud");

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (inputCloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*filteredCloud);
}

void PointCloudTo3dMesh::smoothingAndNormalEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud,
						  pcl::PointCloud<pcl::PointNormal>::Ptr movingLeastSquarePointsPtr){
  printInfo("Smoothing and computing normals");

  // Init objects
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
  // Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kDTree (new pcl::search::KdTree<pcl::PointXYZ>);

  // PointNormal type: output of the normals calculated by MLS
  pcl::PointCloud<pcl::PointNormal> mls_points;
 
  mls.setComputeNormals (true);

  // Set parameters
  mls.setInputCloud (inputCloud);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (kDTree);
  mls.setSearchRadius (0.03);

  // Reconstruct normals
  mls.process (mls_points);
  //movingLeastSquarePointsPtr.reset(&mls_points, [](pcl::PointCloud<pcl::PointNormal>* ptr){std::cout<<"not deleating the instance"<<std::endl;});
  * movingLeastSquarePointsPtr = mls_points;
}

void PointCloudTo3dMesh::get3DMeshFromPointCloud() {
  printInfo("Getting the 3D mesh from the point cloud");

  pcl::PolygonMesh trianglesMesh;
  pcl::PointCloud<PointT>::Ptr inlinersCloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr outlinersCloud (new pcl::PointCloud<PointT>);

  rgbd_utils::RGBD_to_Pointcloud converter(_rgbdSub->get_depthConstPtr(),
					   _rgbdSub->get_rgbConstPtr(),
					   _rgbdSub->get_rgb_infoConstPtr());
  converter.convert();

  // Publishing the point cloud
  sensor_msgs::PointCloud2 rosCloudMsg=converter.get_pointcloud();
  //pcl::toROSMsg(*(_cloud),cc_msg);
  rosCloudMsg.header = _rgbdSub->get_depth().header;
  //(converter.get_pointcloud()).header = _rgbd_sub->get_depth().header;
  _pointCloudPub->publish(rosCloudMsg);
	
  //compute 3d mesh
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::fromROSMsg(rosCloudMsg,*(cloud));
  if (!(cloud->empty())){
    // Create the filtering object
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZ>);
    // Filter to remove the outliers
    outliersRemoval(cloud, filteredCloud);
      

    // Plane segmentation
    planeModelSegmentation(filteredCloud,inlinersCloud, outlinersCloud);

    //Smoothing and normal estimation
    pcl::PointCloud<pcl::PointNormal>::Ptr movingLeastSquaresPointsPtr(new  pcl::PointCloud<pcl::PointNormal>) ;
    smoothingAndNormalEstimation(outlinersCloud, movingLeastSquaresPointsPtr);

    //Fast triangulation
    trianglesMesh = fastTriangulation(movingLeastSquaresPointsPtr);
  } else   printWarn("The point cloud is empty");  
  _mesh=trianglesMesh;
}

void PointCloudTo3dMesh::visualizeMesh(){
  printInfo("Visualising the 3D mesh");
  _update =false;
  if (_mesh.cloud.width>0 ){
    if (_viewerEmpty){
      _viewerEmpty = false;
    }else{      
      _viewer->removePolygonMesh("Mesh_"+std::to_string(meshID-1),0);
    }
    _viewer->addPolygonMesh(_mesh,"Mesh_"+std::to_string(meshID),0);
    while (!_viewer->wasStopped () && !_update){
      _viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));

    }
  }
    	
}

int main(int argc, char** argv)
{
    std::string node_name="pcTo3dMesh";    
    ros::init(argc, argv, node_name);
    PointCloudTo3dMesh pcTo3dMesh;
    while (ros::ok() && !pcTo3dMesh.getViewerStatus() ) {
      pcTo3dMesh.get3DMeshFromPointCloud();
      pcTo3dMesh.visualizeMesh();      
      usleep(2000);
      ros::spinOnce();
    }

    return 0;
}
