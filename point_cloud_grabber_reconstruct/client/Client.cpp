#include "Client.h"



Client::Client()
{
	typedef boost::signals2::signal<void()> Signal;
	cloudSignal = new Signal();
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
	sclient = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	sin.sin_family = AF_INET;
	sin.sin_port = htons(8888);
	sin.sin_addr.S_un.S_addr = inet_addr("127.0.0.1");
	len = sizeof(sin);

	while (true) {
		
		if (cloudSignal->num_slots() > 0) {
			cloudSignal->operator()();
		}

		//const char * sendData = "���Կͻ��˵����ݰ�.\n";
		///*  ��һ������  ��ȡ�����ݰ�����  */
		//sendto(sclient, sendData, strlen(sendData), 0, (sockaddr *)&sin, len);

		//char bufferSizeRaw[255];
		//unsigned int bufferSize;
		//int ret = recvfrom(sclient, bufferSizeRaw, 255, 0, (sockaddr *)&sin, &len);
		//if (ret > 0) {
		//	bufferSize = bytesToInt(bufferSizeRaw);
		//	std::cout << "total size is " << bufferSize << std::endl;
		//}

		///* �ڶ�������  ��ȡ�������ݰ� */
		//sendto(sclient, sendData, strlen(sendData), 0, (sockaddr *)&sin, len);
		//
		//char recvRaw[60010];
		//short recvData[60020];
		//unsigned int leftBytes = bufferSize;
		//while (leftBytes > 0) {
		//	int ret = recvfrom(sclient, recvRaw, 60000, 0, (sockaddr *)&sin, &len);
		//	memcpy(recvData, recvRaw, 60000);
		//	std::cout << "ret size is " << ret << std::endl;

		//	leftBytes -= ret;
		//}
		//
		//std::cout << "one phase ended " << std::endl;

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
