// Disable Error C4996 that occur when using Boost.Signals2.
#ifdef _DEBUG
#define _SCL_SECURE_NO_WARNINGS
#endif
/*
TODO:
1. 转盘在动的时候存下点云
2. 裁剪可视化
3. 结果可视化
4. 确认矩阵变换没有弄错
*/

//#include "main.h"
#include "kinect2_grabber.h"
#include <time.h>
#include <cmath>
#include <pcl/surface/gp3.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h> 
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>

typedef pcl::PointXYZRGBA PointType;


// tr -> in 的坐标系
int ICP(pcl::PointCloud<PointType>::Ptr cloud_in, pcl::PointCloud<PointType>::Ptr cloud_tr, Eigen::Matrix4d& matrix) {
	 
	pcl::PointCloud<PointType>::Ptr cloud_icp(new pcl::PointCloud<PointType>);  // ICP output point cloud  

	*cloud_icp = *cloud_tr;

	// The Iterative Closest Point algorithm  
	pcl::IterativeClosestPoint<PointType, PointType> icp;
	icp.setMaximumIterations(20);
	icp.setInputSource(cloud_icp);
	icp.setInputTarget(cloud_in);
	icp.align(*cloud_icp);

	if (icp.hasConverged())
	{
		std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
		matrix = icp.getFinalTransformation().cast<double>();
	}
	else
	{
		PCL_ERROR("\nICP has not converged.\n");
		system("pause");
		return (-1);
	}

	return 0;
}

// 裁剪掉不需要的部分
void preCutOffClouds(std::vector<pcl::PointCloud<PointType>::Ptr>& cloudQueues, 
					float minX, float maxX, float minY, float maxY, float minZ, float maxZ) {
	int numOfPCL = cloudQueues.size();
	for (size_t i = 0; i < numOfPCL; i++) {
		pcl::PointCloud<PointType>::Ptr Filtered(new pcl::PointCloud<PointType>);
		pcl::CropBox<PointType> boxFilter;
		boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
		boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
		boxFilter.setInputCloud(cloudQueues[i]);
		boxFilter.filter(*Filtered);

		cloudQueues[i] = Filtered;
	}
}

void operateClouds(std::vector<pcl::PointCloud<PointType>::Ptr>& cloudQueues) {
	int numOfPCL = cloudQueues.size();
	std::vector<Eigen::Matrix4d> transformVecs(numOfPCL-1);
	for (size_t i = 1; i < numOfPCL; i++) {
		std::cout << "开始计算变换矩阵" << std::endl;
		int hr = ICP(cloudQueues[i - 1], cloudQueues[i], transformVecs[i - 1]);
		std::cout << "矩阵计算成功" << std::endl;
		if (hr != 0)
			std::cout << "错误，ICP不收敛" << std::endl;
	}
	Eigen::Matrix4d currentMatrix = Eigen::Matrix4d::Identity();
	for (size_t i = 1; i < numOfPCL; i++) {
		currentMatrix = currentMatrix * transformVecs[i - 1];    // ?? 左乘 or 右乘 
		// 变换点云
		pcl::PointCloud<PointType>::Ptr pPointCloudOut(new pcl::PointCloud<PointType>());
		pcl::transformPointCloud(*cloudQueues[i], *pPointCloudOut, currentMatrix);
		cloudQueues[i] = pPointCloudOut;
		/*std::stringstream ss;
		ss << i;
		std::string fileName = ss.str() + ".pcd";
		pcl::io::savePCDFile(fileName, *cloudQueues[i]);
		std::cout << "第" << i << "个点云已经保存成功" << std::endl;*/
	}


}

pcl::PointCloud<PointType>::Ptr combineClouds(std::vector<pcl::PointCloud<PointType>::Ptr>& cloudQueues) {
	pcl::PointCloud<PointType>::Ptr combinedClouds(new pcl::PointCloud<PointType>);
	for (size_t i = 0; i < cloudQueues.size(); i++) {
		*combinedClouds += *cloudQueues[0];
	}
	return combinedClouds;
}

float distanceSqred(PointType* p1, PointType* p2) {

	return pow(p1->x - p2->x, 2) + pow(p1->y - p2->y, 2) + pow(p1->z - p2->z, 2);
}

pcl::PointCloud<PointType>::Ptr sparseCloud(pcl::PointCloud<PointType>::Ptr combinedCloud, float threshold) {
	pcl::PointCloud<PointType>::Ptr sparsedCloud(new pcl::PointCloud<PointType>);
	int minX = 0xFFFFF;
	int maxX = -0xFFFFF;
	int minY = 0xFFFFF;
	int maxY = -0xFFFFF;
	int minZ = 0xFFFFF;
	int maxZ = -0xFFFFF;
	for (size_t i = 0; i < combinedCloud->points.size(); i++) {
		if (combinedCloud->points[i].x < minX)
			minX = floor(combinedCloud->points[i].x);
		if (combinedCloud->points[i].x > maxX)
			maxX = ceil(combinedCloud->points[i].x);
		if (combinedCloud->points[i].y < minY)
			minY = floor(combinedCloud->points[i].y);
		if (combinedCloud->points[i].y > maxY)
			maxY = ceil(combinedCloud->points[i].y);
		if (combinedCloud->points[i].z < minZ)
			minZ = floor(combinedCloud->points[i].z);
		if (combinedCloud->points[i].z > maxZ)
			maxZ = ceil(combinedCloud->points[i].z);
	}

	int numX = (maxX - minX) / threshold;
	int numY = (maxY - minY) / threshold;
	int numZ = (maxZ - minZ) / threshold;
	std::vector<std::vector<int>> gg(10, std::vector<int>(10, 1));
	std::vector< std::vector< std::vector<PointType*> > > grids(numX, std::vector<std::vector<PointType*>>(numY, std::vector<PointType*>(numZ, nullptr)));
	
	for (size_t i = 0; i < combinedCloud->points.size(); i++) {
		int xIndex = (combinedCloud->points[i].x - minX)/threshold;
		int yIndex = (combinedCloud->points[i].y - minY)/threshold;
		int zIndex = (combinedCloud->points[i].z - minZ)/threshold;
		if (xIndex < numX && yIndex < numY && zIndex < numZ) {
			if (grids[xIndex][yIndex][zIndex] == nullptr) {
				grids[xIndex][yIndex][zIndex] = &combinedCloud->points[i];
			}
			else {
				PointType p;
				p.x = minX + xIndex * threshold;
				p.y = minY + yIndex * threshold;
				p.z = minZ + zIndex * threshold;
				float dis1 = distanceSqred(grids[xIndex][yIndex][zIndex], &p);
				float dis2 = distanceSqred(&combinedCloud->points[i], &p);
				grids[xIndex][yIndex][zIndex] = dis1 > dis2 ? &combinedCloud->points[i] : grids[xIndex][yIndex][zIndex];
			}
		}
	}
	for (size_t i = 0; i < numX; i++) {
		for (size_t j = 0; j < numY; j++) {
			for (size_t k = 0; k < numZ; k++) {
				if (grids[i][j][k] != nullptr) {
					sparsedCloud->push_back(*grids[i][j][k]);
				}
			}
		}
	}

	return sparsedCloud;
}

void Reconstruction(pcl::PointCloud<PointType>::Ptr sparsedCloud) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*sparsedCloud, *cloud);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setKSearch(20);
	n.compute(*normals);

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(0.025);

	// Set typical values for the parameters
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(triangles);

	pcl::io::savePLYFile("网格.ply", triangles);
}

int getParameter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float parameter[4]) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(cloud);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

	ne.setSearchMethod(tree);

	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

	// Use all neighbors in a sphere of radius 1cm

	//ne.setRadiusSearch(0.02);
	ne.setKSearch(15);


	std::cout << "开始计算法线" << std::endl;
	ne.compute(*normals);

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);

	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
	std::cout << "法线计算成功" << std::endl;

	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;

	pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);

	pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);


	pcl::ExtractIndices<pcl::PointXYZ> extract;

	// Create the segmentation object for the planar model and set all the parameters

	seg.setOptimizeCoefficients(true);//设置对估计的模型系数需要进行优化

	seg.setModelType(pcl::SACMODEL_NORMAL_PLANE); //设置分割模型

	seg.setNormalDistanceWeight(0.1);//设置表面法线权重系数

	seg.setMethodType(pcl::SAC_RANSAC);//设置采用RANSAC作为算法的参数估计方法

	seg.setMaxIterations(500); //设置迭代的最大次数

	seg.setDistanceThreshold(0.05); //设置内点到模型的距离允许最大值

	seg.setInputCloud(cloud);

	seg.setInputNormals(normals);

	// 开始segment
	seg.segment(*inliers_plane, *coefficients_plane);

	std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

	// Extract the planar inliers from the input cloud

	for (size_t i = 0; i < 4; i++) {
		parameter[i] = coefficients_plane->values[i];
	}

	//extract.setInputCloud(cloud);

	//extract.setIndices(inliers_plane);

	//extract.setNegative(false);


	//// 提取地面
	//extract.filter(*cloud_filtered);


	/*pcl::PCDWriter writer;
	writer.write<pcl::PointXYZ>("3dpoints_ground.pcd", *cloud_filtered, false);

	extract.setNegative(true);
	extract.filter(*cloud_filtered);
	writer.write<pcl::PointXYZ>("3dpoints_object.pcd", *cloud_filtered, false);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
	float a = coefficients_plane->values[0];
	float b = coefficients_plane->values[1];
	float c = coefficients_plane->values[2];
	float d = coefficients_plane->values[3];
	for (size_t i = 0; i < cloud->points.size(); i++) {
		pcl::PointXYZ& p = cloud->points[i];
		float residual = a * p.x + b * p.y + c * p.z + d;
		if (abs(residual) > 0.03)
			cloud_plane->push_back(p);
	}
	pcl::io::savePCDFile("object.pcd", *cloud_plane);*/
	return 0;
}

int filterWithPlane(pcl::PointCloud<PointType>::Ptr cloudIn, pcl::PointCloud<PointType>::Ptr cloudOut, float parameter[4]) {
	float a = parameter[0];
	float b = parameter[1];
	float c = parameter[2];
	float d = parameter[3];
	for (size_t i = 0; i < cloudIn->points.size(); i++) {
		pcl::PointXYZRGBA& p = cloudIn->points[i];
		float residual = a * p.x + b * p.y + c * p.z + d;
		if (abs(residual) > 0.04 && p.x != 0.0 && p.y != 0.0 && p.z != 0.0)
			cloudOut->push_back(p);
	}

	return 0;
}
int filterWithPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut, float parameter[4]) {
	float a = parameter[0];
	float b = parameter[1];
	float c = parameter[2];
	float d = parameter[3];
	for (size_t i = 0; i < cloudIn->points.size(); i++) {
		pcl::PointXYZ& p = cloudIn->points[i];
		float residual = a * p.x + b * p.y + c * p.z + d;
		if (abs(residual) > 0.04 && (p.x != 0.0 || p.y != 0.0 || p.z != 0.0))
			cloudOut->push_back(p);
	}

	return 0;
}

int main1() {
	pcl::PointCloud<PointType>::Ptr cloud_xyzrgb(new pcl::PointCloud<PointType>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile("点云/test.pcd", *cloud_xyzrgb);
	pcl::copyPointCloud(*cloud_xyzrgb, *cloud_xyz);
	std::cout << cloud_xyz->points.size() << std::endl;
	//extractGround(cloud_xyz);
	return 0;
}

void filter() {
	//pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;// 创建滤波器    
	//outrem.setInputCloud(cloudFiltered);              //设置输入点云
	//outrem.setRadiusSearch(0.02);              //设置在0.8半径的范围内找邻近点
	//outrem.setMinNeighborsInRadius(40);       //设置查询点的邻近点集数小于2的删除
	//outrem.filter(*cloud_filtered);//执行条件滤波，存储结果到cloud_filtered
}

int main3() {
	// get parameter
	pcl::PointCloud<pcl::PointXYZ>::Ptr testCloud(new pcl::PointCloud<pcl::PointXYZ>);
	float parameter[4];
	pcl::io::loadPCDFile("点云/0.pcd", *testCloud);
	getParameter(testCloud, parameter);

	std::cout << "参数生成成功" << std::endl;

	pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
	pcl::PointCloud<PointType>::Ptr cloudFiltered(new pcl::PointCloud<PointType>);
	// Create the filtering object
	pcl::io::loadPCDFile("点云/1.pcd", *cloud);
	filterWithPlane(cloud, cloudFiltered, parameter);

	
	pcl::io::savePCDFile("滤波.pcd", *cloudFiltered);

	return 0;
}

int main() {
	// get parameter
	pcl::PointCloud<pcl::PointXYZ>::Ptr testCloud(new pcl::PointCloud<pcl::PointXYZ>);
	float parameter[4];
	pcl::io::loadPCDFile("点云/0.pcd", *testCloud);
	getParameter(testCloud, parameter);

	std::cout << "参数生成成功" << std::endl;

	std::vector<pcl::PointCloud<PointType>::Ptr> cloudQueues;
	for (size_t i = 0; i < 31; i++) {
		pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
		pcl::PointCloud<PointType>::Ptr cloudFiltered(new pcl::PointCloud<PointType>);
		std::stringstream ss;
		ss << i;
		std::string path = "点云/" + ss.str() + ".pcd";
		pcl::io::loadPCDFile(path, *cloud);
		filterWithPlane(cloud, cloudFiltered, parameter);
		cloudQueues.push_back(cloudFiltered);
		std::cout << cloudFiltered->points.size() << std::endl;
	}
	std::cout << "点云读取成功" << std::endl;
	operateClouds(cloudQueues);

	return 0;
}

int main2() {
	// PCL Visualizer
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
		new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
	viewer->setCameraPosition(0.0, 0.0, -2.5, 0.0, 0.0, 0.0);

	pcl::PointCloud<PointType>::ConstPtr cloud;

	std::vector<pcl::PointCloud<PointType>::Ptr> cloudQueues;

	boost::mutex mutex;
	boost::function<void(const pcl::PointCloud<PointType>::ConstPtr&)> function =
		[&cloud, &mutex](const pcl::PointCloud<PointType>::ConstPtr& ptr) {
		boost::mutex::scoped_lock lock(mutex);
		cloud = ptr->makeShared();
	};

	boost::function<void()> addCloudToArrayFunc = 
		[&cloud, &mutex, &cloudQueues]() {
		boost::mutex::scoped_lock lock(mutex);
		pcl::PointCloud<PointType>::Ptr ptr;
		ptr = cloud->makeShared();
		cloudQueues.push_back(ptr);
	};



	// Kinect2Grabber
	boost::shared_ptr<pcl::Grabber> grabber = boost::make_shared<pcl::Kinect2Grabber>();

	// Register Callback Function
	boost::signals2::connection connection = grabber->registerCallback(function);


	// Start Grabber
	grabber->start();
	std::cout << "has been started" << std::endl;
	while (cloud == nullptr) {
		std::cout << "cloud has not been initialized yet" << endl;
	}


	std::cout << "<--------------- kinect初始化成功" << std::endl;

	clock_t startTime, endTime;
	startTime = clock();
	while (true)
	{
		Sleep(2974);
		std::cout << "添加到队列中" << std::endl;
		addCloudToArrayFunc();
		if (cloudQueues.size() == 64)
			break;
	}
	endTime = clock();
	std::cout << "Totle Time : " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << endl;

	// Stop Grabber
	grabber->stop();

	// Disconnect Callback Function
	if (connection.connected()) {
		connection.disconnect();
	}

	// 预处理  去除框外的
	preCutOffClouds(cloudQueues, -0.2, 0.2, -0.6, 0.6, -2.0, 2.0);

	pcl::PointCloud<pcl::PointXYZ>::Ptr testCloud(new pcl::PointCloud<pcl::PointXYZ>);
	float parameter[4];
	pcl::copyPointCloud(*cloudQueues[0], *testCloud);
	getParameter(testCloud, parameter);

	std::cout << "参数生成成功" << std::endl;

	for (size_t i = 0; i < cloudQueues.size(); i++) {
		pcl::PointCloud<PointType>::Ptr cloudFiltered(new pcl::PointCloud<PointType>);
		filterWithPlane(cloudQueues[i], cloudFiltered, parameter);
		cloudQueues[i] = cloudFiltered;
		std::cout << cloudFiltered->points.size() << std::endl;
	}
	operateClouds(cloudQueues);

	pcl::PointCloud<PointType>::Ptr combinedCloud = combineClouds(cloudQueues);
	pcl::PointCloud<PointType>::Ptr sparsedCloud = sparseCloud(combinedCloud, 0.02);

	Reconstruction(sparsedCloud);

	return 0;
}