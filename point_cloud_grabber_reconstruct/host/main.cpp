// Disable Error C4996 that occur when using Boost.Signals2.
#ifdef _DEBUG
#define _SCL_SECURE_NO_WARNINGS
#endif
//

//#include "main.h"
#include "kinect2_grabber.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <boost/asio.hpp>
#include "transform.h"
typedef pcl::PointXYZRGBA PointType;


struct PointCloudBuffers
{
	typedef boost::shared_ptr<PointCloudBuffers> Ptr;
	std::vector<short> points;    // x y z r g b
	//std::vector<unsigned char> rgb;
};

void
CopyPointCloudToBuffers(pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, PointCloudBuffers& cloud_buffers)
{
	const size_t nr_points = cloud->points.size();

	cloud_buffers.points.resize(nr_points * 6);
	//cloud_buffers.rgb.resize(nr_points * 3);

	const pcl::PointXYZ  bounds_min(-0.9, -0.8, 1.0);
	const pcl::PointXYZ  bounds_max(0.9, 3.0, 3.3);

	size_t j = 0;
	float oriToThisZ;
	float oriToThisX;
	float theta;
	readConfig("transform.xml", oriToThisX, oriToThisZ, theta);
	theta = theta / 180 * 3.141592653;
	for (size_t i = 0; i < nr_points; ++i)
	{

		const pcl::PointXYZRGBA& point = cloud->points[i];

		if (point.x == 0.0 && point.y == 0.0 && point.z == 0.0)
			continue;

		/*if (!pcl_isfinite(point.x) ||
			!pcl_isfinite(point.y) ||
			!pcl_isfinite(point.z))
			continue;

		if (point.x < bounds_min.x ||
			point.y < bounds_min.y ||
			point.z < bounds_min.z ||
			point.x > bounds_max.x ||
			point.y > bounds_max.y ||
			point.z > bounds_max.z)
			continue;*/


		/*
		在这里做矩阵变换
		*/
		float new_x = -sin(theta)*point.z + cos(theta)*point.x + oriToThisX;
		float new_z = cos(theta)*point.z + sin(theta)*point.x + oriToThisZ;



		const int conversion_factor = 500; 
		
		cloud_buffers.points[j * 6 + 0] = static_cast<short> (new_x*conversion_factor);         // float to short   
		cloud_buffers.points[j * 6 + 1] = static_cast<short> (point.y*conversion_factor);
		cloud_buffers.points[j * 6 + 2] = static_cast<short> (new_z*conversion_factor);
		cloud_buffers.points[j * 6 + 3] = static_cast<short> (point.r);
		cloud_buffers.points[j * 6 + 4] = static_cast<short> (point.g);
		cloud_buffers.points[j * 6 + 5] = static_cast<short> (point.b);

		
		/*cloud_buffers.rgb[j * 3 + 0] = point.r;
		cloud_buffers.rgb[j * 3 + 1] = point.g;
		cloud_buffers.rgb[j * 3 + 2] = point.b;*/

		j++;
	}

	cloud_buffers.points.resize(j * 6);
	//cloud_buffers.rgb.resize(j * 3);
}

void intToByte(int i, char * stream, int size = 4)
{
	memset(stream, 0, sizeof(char)*size);
	stream[0] = (char)(0xff & i);
	stream[1] = (char)((0xff00 & i) >> 8);
	stream[2] = (char)((0xff0000 & i) >> 16);
	stream[3] = (char)((0xff000000 & i) >> 24);
	return;
}



int main( int argc, char* argv[] )
{
    // PCL Visualizer
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
        new pcl::visualization::PCLVisualizer( "Point Cloud Viewer" ) );
    viewer->setCameraPosition( 0.0, 0.0, -2.5, 0.0, 0.0, 0.0 );

    pcl::PointCloud<PointType>::ConstPtr cloud;
	auto host = socketHost::HostSock();


    boost::mutex mutex;
    boost::function<void( const pcl::PointCloud<PointType>::ConstPtr& )> function =
        [&cloud, &mutex]( const pcl::PointCloud<PointType>::ConstPtr& ptr ){
            boost::mutex::scoped_lock lock( mutex );
            cloud = ptr->makeShared();
			std::cout << "cloud size is :" << cloud->points.size() << std::endl;
			//mutex.unlock();
        };

	boost::function<void()> saveFunction =
		[&cloud, &mutex, &host]() {
		std::cout << "before lock" << std::endl;
			boost::mutex::scoped_lock lock(mutex);
			std::cout << "save has been called" << std::endl;

			char recvData[255];
			
			
			int ret = recvfrom(host.cf.serSocket, recvData, 255, 0, (sockaddr*)&host.cf.remoteAddr, &host.cf.nAddrLen);
			if (ret > 0)
			{
				recvData[ret] = 0x00;
				printf("接受到一个连接：%s \r\n", inet_ntoa(host.cf.remoteAddr.sin_addr));
				printf(recvData);
				printf("正在准备发送数据长度\n");
			}
			

			PointCloudBuffers::Ptr buffers_to_send = PointCloudBuffers::Ptr(new PointCloudBuffers);
			CopyPointCloudToBuffers(cloud, *buffers_to_send);
			unsigned int n_points = buffers_to_send->points.size() / 6;


			/*发送数据包总长度*/
			char sendBufferSize[255];
			unsigned int totalByte = buffers_to_send->points.size() * sizeof(short);
			intToByte(totalByte, sendBufferSize);
			sendto(host.cf.serSocket, sendBufferSize, sizeof(unsigned int), 0, (sockaddr *)&host.cf.remoteAddr, host.cf.nAddrLen);
			
			
			ret = recvfrom(host.cf.serSocket, recvData, 255, 0, (sockaddr*)&host.cf.remoteAddr, &host.cf.nAddrLen);
			if (ret > 0)
			{
				recvData[ret] = 0x00;
				printf("接受到一个连接：%s \r\n", inet_ntoa(host.cf.remoteAddr.sin_addr));
				printf(recvData);
				printf("正在准备发送数据\n");
			}

			std::cout << "point cloud size is " << n_points << std::endl;

			/*unsigned int nr_points = cloud->points.size();
			std::cout << nr_points << std::endl;*/
			/*if (nr_points > 0)
			{
				
				boost::asio::write(host.socket, boost::asio::buffer(&buffers_to_send->points.front(), nr_points * 3 * sizeof(short)));
				boost::asio::write(host.socket, boost::asio::buffer(&buffers_to_send->rgb.front(), nr_points * 3 * sizeof(unsigned char)));
				std::cout << "data has been sent" << std::endl;
			}*/

			unsigned int numByte = 60000;
			unsigned int leftBytes = buffers_to_send->points.size() * sizeof(short);
			unsigned int sendTimes = ceil(buffers_to_send->points.size() * sizeof(short) / 60000.0);
			char * temp = new char[numByte];
			for (size_t i = 0; i < sendTimes; i++) {
				unsigned int len = numByte;
				if (leftBytes < numByte) {
					len = leftBytes;
				}
				std::cout << "len is " << len << std::endl;
				memset(temp, 0, len);
				memcpy(temp, buffers_to_send->points.data()+i*numByte/2, len);
				sendto(host.cf.serSocket, temp, len, 0, (sockaddr *)&host.cf.remoteAddr, host.cf.nAddrLen);
				leftBytes -= numByte;
				std::cout << "package " << i << " has been sent" << std::endl;
			}
			free(temp);
			/*unsigned int numByte = 60000;
			char * temp = new char[numByte+sizeof(numByte)];
			memset(temp, 0, numByte);
			intToByte(numByte, temp);
			memcpy(temp+sizeof(numByte), buffers_to_send->points.data(), numByte);*/
			//memcpy(temp+sizeof(numByte), &(buffers_to_send->points[0]), numByte);

			/*unsigned int numByte = buffers_to_send->points.size() * sizeof(short);
			char * temp = new char[numByte+sizeof(numByte)];
			memset(temp, 0, numByte + sizeof(numByte));
			memcpy(temp, &numByte, sizeof(numByte));
			memcpy(temp+sizeof(numByte), &(buffers_to_send->points[0]), numByte);*/

		

			//mutex.unlock();
		};


    // Kinect2Grabber
    boost::shared_ptr<pcl::Grabber> grabber = boost::make_shared<pcl::Kinect2Grabber>();

    // Register Callback Function
    boost::signals2::connection connection = grabber->registerCallback( function );

	

    // Start Grabber
    grabber->start();
	std::cout << "has been started" << std::endl;
	while (cloud == nullptr) {
		std::cout << "cloud has not been initialized yet" << endl;
	}
	
	//{
	//	mutex.unlock();
	//	saveSignal(cloud);
	//}
	//
	//std::cout << "has been saved";


	
	host.registerCallback(saveFunction);
	host.start();

    while( !viewer->wasStopped() ){
        // Update Viewer
        viewer->spinOnce();

        boost::mutex::scoped_try_lock lock( mutex );
        if( lock.owns_lock() && cloud ){
            // Update Point Cloud
            if( !viewer->updatePointCloud( cloud, "cloud" ) ){
                viewer->addPointCloud( cloud, "cloud" );
            }
        }
    }
	
	//WSADATA wsaData;
	//WORD sockVersion = MAKEWORD(2, 2);
	//if (WSAStartup(sockVersion, &wsaData) != 0)
	//{
	//	return 0;
	//}

	//SOCKET serSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	//if (serSocket == INVALID_SOCKET)
	//{
	//	printf("socket error !");
	//	return 0;
	//}

	//sockaddr_in serAddr;
	//serAddr.sin_family = AF_INET;
	//serAddr.sin_port = htons(8888);
	//serAddr.sin_addr.S_un.S_addr = INADDR_ANY;
	//if (bind(serSocket, (sockaddr *)&serAddr, sizeof(serAddr)) == SOCKET_ERROR)
	//{
	//	printf("bind error !");
	//	closesocket(serSocket);
	//	return 0;
	//}

	//sockaddr_in remoteAddr;
	//int nAddrLen = sizeof(remoteAddr);
	//while (true && !viewer->wasStopped())
	//{
	//	char recvData[255];
	//	int ret = recvfrom(serSocket, recvData, 255, 0, (sockaddr*)&remoteAddr, &nAddrLen);
	//	if (ret > 0)
	//	{
	//		recvData[ret] = 0x00;
	//		printf("接受到一个连接：%s \r\n", inet_ntoa(remoteAddr.sin_addr));
	//		printf(recvData);
	//	}

	//	//char temp[10000];   // dynamic
	//	//memset(temp, 0, sizeof(temp));

	//	//memcpy(temp, cloud.get(), sizeof(*cloud));

	//	//sendto(serSocket, temp, sizeof(temp), 0, (sockaddr *)&remoteAddr, nAddrLen);
	//	const char * sendData = "一个来自服务端的UDP数据包\n";
	//	sendto(serSocket, sendData, strlen(sendData), 0, (sockaddr *)&remoteAddr, nAddrLen);

	//	boost::mutex::scoped_try_lock lock(mutex);
	//	if (lock.owns_lock() && cloud) {
	//		if (!viewer->updatePointCloud(cloud, "cloud")) {
	//			viewer->addPointCloud(cloud, "cloud");
	//		}
	//	}

	//}
	//std::cout << "has closed" << std::endl;
	//closesocket(serSocket);
	//WSACleanup();




    // Stop Grabber
    grabber->stop();
    
    // Disconnect Callback Function
    if( connection.connected() ){
        connection.disconnect();
    }

    return 0;
}
