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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h> 
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
typedef pcl::PointXYZRGBA PointType;


// tr -> in 的坐标系
int ICP(pcl::PointCloud<PointType>::Ptr cloud_in, pcl::PointCloud<PointType>::Ptr cloud_tr, Eigen::Matrix4d& matrix) {
	 
	pcl::PointCloud<PointType>::Ptr cloud_icp(new pcl::PointCloud<PointType>);  // ICP output point cloud  

	*cloud_icp = *cloud_tr;

	// The Iterative Closest Point algorithm  
	pcl::IterativeClosestPoint<PointType, PointType> icp;
	icp.setMaximumIterations(30);
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
		int hr = ICP(cloudQueues[i - 1], cloudQueues[i], transformVecs[i - 1]);
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
		std::stringstream ss;
		ss << i;
		std::string fileName = ss.str() + ".pcd";
		pcl::io::savePCDFile(fileName, *cloudQueues[i]);
		std::cout << "第" << i <<  "个点云已经保存成功" << std::endl;
	}


}

int main1() {
	std::vector<pcl::PointCloud<PointType>::Ptr> cloudQueues;
	for (size_t i = 0; i < 3; i++) {
		pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
		std::stringstream ss;
		ss << i;
		std::string path = "点云/" + ss.str() + ".pcd";
		pcl::io::loadPCDFile(path, *cloud);
		cloudQueues.push_back(cloud);
		std::cout << cloud->points.size() << std::endl;
	}
	operateClouds(cloudQueues);

	return 0;
}

int main() {
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
	//preCutOffClouds(cloudQueues, -0.2, 0.2, -0.6, 0.6, -2.0, 2.0);

	// 收集完成 开始处理
	//operateClouds(cloudQueues);


	return 0;
}