#pragma once
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
	std::vector<Eigen::Matrix4d> transformVecs(numOfPCL - 1);
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
	}


}

void refineTransform(std::vector<pcl::PointCloud<PointType>::Ptr>& cloudQueues) {
	int numOfPCL = cloudQueues.size();
	std::vector<Eigen::Matrix4d> transformVecs(numOfPCL);
	for (size_t i = 0; i < numOfPCL; i++) {
		std::cout << "开始计算变换矩阵" << std::endl;
		int hr = ICP(cloudQueues[14], cloudQueues[i], transformVecs[i]);
		std::cout << "矩阵计算成功" << std::endl;
		if (hr != 0)
			std::cout << "错误，ICP不收敛" << std::endl;
	}
	for (size_t i = 0; i < numOfPCL; i++) {
		// 变换点云
		pcl::PointCloud<PointType>::Ptr pPointCloudOut(new pcl::PointCloud<PointType>());
		pcl::transformPointCloud(*cloudQueues[i], *pPointCloudOut, transformVecs[i]);
		cloudQueues[i] = pPointCloudOut;
		std::stringstream ss;
		ss << i;
		std::string fileName = ss.str() + ".pcd";
		pcl::io::savePCDFile(fileName, *cloudQueues[i]);
		std::cout << "第" << i << "个点云已经保存成功" << std::endl;
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