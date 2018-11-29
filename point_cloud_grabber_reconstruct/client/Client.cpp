#include "Client.h"



Client::Client()
{
	typedef boost::signals2::signal<void()> Signal;
	cloudSignal = new Signal();
	ips = readConfig("../config.xml");
	std::cout << ips[0] << std::endl;
	numHost = ips.size();
}

void Client::initVariable(std::string ip, int port=8888) {
	sclient = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	sin.sin_family = AF_INET;
	sin.sin_port = htons(port);
	sin.sin_addr.S_un.S_addr = inet_addr(ip.c_str());
	len = sizeof(sin);
}

pcl::PointCloud<PointType>::ConstPtr Client::getDataRecved() {
	return pointCloudRecved;
}


boost::signals2::connection Client::registerCallback(const boost::function<void()> & callback)
{
	boost::signals2::connection ret = cloudSignal->connect(callback);
	return (ret);
}

void Client::connectToHost() {

	WORD socketVersion = MAKEWORD(2, 2);
	WSADATA wsaData;
	if (WSAStartup(socketVersion, &wsaData) != 0)
	{
		return;
	}
	

	while (true) {
		
		pointCloudRecved = boost::make_shared<pcl::PointCloud<PointType>>();


		for (size_t i = 0; i < numHost; i++) {

			initVariable(ips[i]);
			
			const char * sendData = "来自客户端的数据包.\n";
			/*  第一次请求  获取总数据包长度  */
			sendto(sclient, sendData, strlen(sendData), 0, (sockaddr *)&(sin), len);

			char bufferSizeRaw[255];
			unsigned int bufferSize;
			int ret = recvfrom(sclient, bufferSizeRaw, 255, 0, (sockaddr *)&(sin), &len);
			if (ret > 0) {
				bufferSize = bytesToInt(bufferSizeRaw);
				std::cout << "total size is " << bufferSize << std::endl;
			}

			pcl::PointCloud<PointType>::Ptr cloudTemp(new pcl::PointCloud<PointType>);
			cloudTemp->width = bufferSize / (12);  // 6*sizeof(short)
			cloudTemp->height = 1;
			cloudTemp->points.resize(cloudTemp->width*cloudTemp->height);

			/* 第二次请求  获取所有数据包 */
			sendto(sclient, sendData, strlen(sendData), 0, (sockaddr *)&(sin), len);

			char recvRaw[60010];
			short recvData[32020];
			//short * recvData = new short[bufferSize/2];
			unsigned int leftBytes = bufferSize;

			unsigned int headPointer = 0;
			unsigned int offset = 0;


			/* 增加接收缓冲区的大小 防止丢包*/
			/*  TODO: 需要增加timeout*/
			int recvBufSize = 2000000;
			int iRet = setsockopt(sclient,
				SOL_SOCKET,
				SO_RCVBUF,
				(const char *)&recvBufSize,
				sizeof(recvBufSize));
			while (leftBytes > 0) {
				int ret = recvfrom(sclient, recvRaw, 60000, 0, (sockaddr *)&(sin), &len);

				memcpy(recvData, recvRaw, ret);
				leftBytes -= ret;
				offset += (ret / 2);

				int numPoints = ret / (12);
				std::cout << "ret size is " << ret << std::endl;

				for (size_t i = 0; i < numPoints; i++) {
					cloudTemp->points[headPointer].x = recvData[i * 6 + 0] / 500.0;
					cloudTemp->points[headPointer].y = recvData[i * 6 + 1] / 500.0;
					cloudTemp->points[headPointer].z = recvData[i * 6 + 2] / 500.0;
					cloudTemp->points[headPointer].r = recvData[i * 6 + 3];
					cloudTemp->points[headPointer].g = recvData[i * 6 + 4];
					cloudTemp->points[headPointer].b = recvData[i * 6 + 5];
					headPointer++;
				}

			}

			pointCloudRecved->operator+=(*cloudTemp);
		}

		/* 调用main中的callback 来把获取到的点云和原本的点云融合 */
		if (cloudSignal->num_slots() > 0) {
			cloudSignal->operator()();
		}

	}
	closesocket(sclient);
	WSACleanup();

}

void Client::start() {
	boost::thread(&Client::connectToHost, this);
}

Client::~Client()
{
}
