// udpTest.cpp : Defines the entry point for the console application.
//

#include <stdio.h> 
#include <iostream>
#include <winsock2.h> 
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

#include "kinect2_grabber.h"
#pragma comment(lib,"ws2_32.lib")  

typedef pcl::PointXYZRGBA PointType;
//void k() {
//	// PCL Visualizer
//	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
//		new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
//	viewer->setCameraPosition(0.0, 0.0, -2.5, 0.0, 0.0, 0.0);
//
//	// Point Cloud
//	pcl::PointCloud<PointType>::ConstPtr cloud;
//
//	boost::mutex mutex;
//	boost::function<void(const pcl::PointCloud<PointType>::ConstPtr&)> function =
//		[&cloud, &mutex](const pcl::PointCloud<PointType>::ConstPtr& ptr) {
//		boost::mutex::scoped_lock lock(mutex);
//
//		cloud = ptr->makeShared();
//	};
//
//	boost::function<void(const pcl::PointCloud<PointType>::ConstPtr&)> saveFunction =
//		[&cloud, &mutex](const pcl::PointCloud<PointType>::ConstPtr& ptr) {
//		mutex.unlock();
//		boost::mutex::scoped_lock lock(mutex);
//		pcl::io::savePCDFile("resultFile.pcd", *cloud);
//
//		return 0;
//	};
//
//
//	// Kinect2Grabber
//	boost::shared_ptr<pcl::Grabber> grabber = boost::make_shared<pcl::Kinect2Grabber>();
//
//	// Register Callback Function
//	boost::signals2::connection connection = grabber->registerCallback(function);
//
//	boost::signals2::signal<void(const pcl::PointCloud<PointType>::ConstPtr&)> saveSignal;
//	saveSignal.connect(saveFunction);
//
//	// Start Grabber
//	grabber->start();
//
//	while (cloud == nullptr) {
//		std::cout << "cloud has not been initialized yet" << endl;
//	}
//
//	{
//		mutex.unlock();
//		saveSignal(cloud);
//	}
//
//	std::cout << "has been saved";
//
//	while (!viewer->wasStopped()) {
//		// Update Viewer
//		viewer->spinOnce();
//
//		boost::mutex::scoped_try_lock lock(mutex);
//		if (lock.owns_lock() && cloud) {
//			// Update Point Cloud
//			if (!viewer->updatePointCloud(cloud, "cloud")) {
//				viewer->addPointCloud(cloud, "cloud");
//			}
//		}
//	}
//
//	// Stop Grabber
//	grabber->stop();
//
//	// Disconnect Callback Function
//	if (connection.connected()) {
//		connection.disconnect();
//	}
//
//}

int main(int argc, char* argv[])
{
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
	//	new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
	//viewer->setCameraPosition(0.0, 0.0, -2.5, 0.0, 0.0, 0.0);

	//pcl::PointCloud<PointType>::ConstPtr cloud;

	//boost::mutex mutex;
	//boost::function<void(const pcl::PointCloud<PointType>::ConstPtr&)> function =
	//	[&cloud, &mutex](const pcl::PointCloud<PointType>::ConstPtr& ptr) {
	//	boost::mutex::scoped_lock lock(mutex);

	//	cloud = ptr->makeShared();
	//};

	//boost::shared_ptr<pcl::Grabber> grabber = boost::make_shared<pcl::Kinect2Grabber>();

	//boost::signals2::connection connection = grabber->registerCallback(function);

	//grabber->start();

	//while (cloud == nullptr) {
	//	std::cout << "cloud has not been initialized yet" << endl;
	//}

	///*{
	//	mutex.unlock();
	//	saveSignal(cloud);
	//}*/

	////while (!viewer->wasStopped()) {
	////	viewer->spinOnce();

	////	boost::mutex::scoped_try_lock lock(mutex);
	////	if (lock.owns_lock() && cloud) {
	////		// Update Point Cloud
	////		if (!viewer->updatePointCloud(cloud, "cloud")) {
	////			viewer->addPointCloud(cloud, "cloud");
	////		}
	////	}
	////}


	WSADATA wsaData;
	WORD sockVersion = MAKEWORD(2, 2);
	if (WSAStartup(sockVersion, &wsaData) != 0)
	{
		return 0;
	}

	SOCKET serSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (serSocket == INVALID_SOCKET)
	{
		printf("socket error !");
		return 0;
	}

	sockaddr_in serAddr;
	serAddr.sin_family = AF_INET;
	serAddr.sin_port = htons(8888);
	serAddr.sin_addr.S_un.S_addr = INADDR_ANY;
	if (bind(serSocket, (sockaddr *)&serAddr, sizeof(serAddr)) == SOCKET_ERROR)
	{
		printf("bind error !");
		closesocket(serSocket);
		return 0;
	}

	sockaddr_in remoteAddr;
	int nAddrLen = sizeof(remoteAddr);
	while (true)
	{
		char recvData[255];
		int ret = recvfrom(serSocket, recvData, 255, 0, (sockaddr*)&remoteAddr, &nAddrLen);
		if (ret > 0)
		{
			recvData[ret] = 0x00;
			printf("接受到一个连接：%s \r\n", inet_ntoa(remoteAddr.sin_addr));
			printf(recvData);
		}

		/*char temp[10000000];
		memset(temp, 0, sizeof(temp));

		memcpy(temp, cloud.get(), sizeof(*cloud));*/

											

		//sendto(sock, temp, sizeof(person), 0, (SOCKADDR*)&addSer, sizeof(SOCKADDR));
		const char * sendData = "一个来自服务端的UDP数据包\n";
		//sendto(serSocket, sendData, strlen(sendData), 0, (sockaddr *)&remoteAddr, nAddrLen);
		sendto(serSocket, sendData, strlen(sendData), 0, (sockaddr *)&remoteAddr, nAddrLen);

	}
	std::cout << "has closed" << std::endl;
	closesocket(serSocket);
	WSACleanup();



	/*grabber->stop();

	if (connection.connected()) {
		connection.disconnect();
	}*/
	return 0;
}

