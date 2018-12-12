#include "Client.h"



Client::Client()
{
	pointsRecved.resize(512*424);
}


Client::~Client()
{
	
}

std::vector<MyPoint> Client::getDataRecved()
{
	return pointsRecved;
}

void Client::Request(std::string ip)
{
	WORD socketVersion = MAKEWORD(2, 2);
	WSADATA wsaData;
	if (WSAStartup(socketVersion, &wsaData) != 0)
	{
		return;
	}
	sclient = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	sin.sin_family = AF_INET;
	sin.sin_port = htons(8888);
	sin.sin_addr.S_un.S_addr = inet_addr(ip.c_str());
	len = sizeof(sin);
	
	const char * sendData = "开始传送图片\n";
	sendto(sclient, sendData, strlen(sendData), 0, (sockaddr *)&(sin), len);


	int recvBufSize = 2500000;
	int iRet = setsockopt(sclient,
		SOL_SOCKET,
		SO_RCVBUF,
		(const char *)&recvBufSize,
		sizeof(recvBufSize));
	int leftBytes = 512 * 424 * 12;
	char recvRaw[PACK_SIZE];
	short recvData[PACK_SIZE / sizeof(short)];
	unsigned int pointer = 0;

	std::cout << "图片接收开始" << std::endl;

	while (leftBytes > 0) {
		std::cout << "剩余比特数为: " << leftBytes << std::endl;

		int ret = recvfrom(sclient, recvRaw, PACK_SIZE, 0, (sockaddr *)&(sin), &len);

		std::cout << "接收到数据包了，大小为： "  << ret << std::endl;

		memcpy(recvData, recvRaw, ret);
		leftBytes -= ret;

		int numPoints = ret / 12;

		for (size_t i = 0; i < numPoints; i++) {
			pointsRecved[pointer].x = recvData[i * 6 + 0] / 500.0;
			pointsRecved[pointer].y = recvData[i * 6 + 1] / 500.0;
			pointsRecved[pointer].z = recvData[i * 6 + 2] / 500.0;
			pointsRecved[pointer].r = recvData[i * 6 + 3];
			pointsRecved[pointer].g = recvData[i * 6 + 4] ;
			pointsRecved[pointer].b = recvData[i * 6 + 5] ;
			pointer++;
		}


	}
	
	closesocket(sclient);
	WSACleanup();
}


void Client::sendMatrix(float matrix[][3], float vector[3], std::string ip) {
	WORD socketVersion = MAKEWORD(2, 2);
	WSADATA wsaData;
	if (WSAStartup(socketVersion, &wsaData) != 0)
	{
		return;
	}

	sclient = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	sin.sin_family = AF_INET;
	sin.sin_port = htons(8888);
	sin.sin_addr.S_un.S_addr = inet_addr(ip.c_str());
	len = sizeof(sin);
	

	short matrixToVec[9];
	unsigned int index = 0;
	for (size_t i = 0; i < 3; i++) {
		for (size_t j = 0; j < 3; j++) {
			matrixToVec[index++] = matrix[i][j]*500;
		}
	}
	char sendMatrix[9*sizeof(short)];
	memset(sendMatrix, 0, 9 * sizeof(short));
	memcpy(sendMatrix, matrixToVec, 9 * sizeof(short));
	sendto(sclient, sendMatrix, 9 * sizeof(short), 0, (sockaddr *)&(sin), len);

	char recvMsg[255];
	int ret = recvfrom(sclient, recvMsg, 255, 0, (sockaddr *)&(sin), &len);
	

	short VecShort[3];
	for (size_t i = 0; i < 3; i++) {
		VecShort[i] = vector[i] * 500;
	}
	char sendVector[3 * sizeof(short)];
	memset(sendVector, 0, 3 * sizeof(short));
	memcpy(sendVector, VecShort, 3 * sizeof(short));
	sendto(sclient, sendVector, 3 * sizeof(short), 0, (sockaddr *)&(sin), len);


	ret = recvfrom(sclient, recvMsg, 255, 0, (sockaddr *)&(sin), &len);
}
