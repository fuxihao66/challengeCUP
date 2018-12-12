#include "Host.h"



Host::Host()
{
	nAddrLen = sizeof(remoteAddr);
}


Host::~Host()
{
}

void Host::waitForReqForImg(const std::vector<myPoint>& points)
{
	WSADATA wsaData;
	WORD sockVersion = MAKEWORD(2, 2);
	if (WSAStartup(sockVersion, &wsaData) != 0)
	{
		return;
	}

	serSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (serSocket == INVALID_SOCKET)
	{
		printf("socket error !");
		return;
	}
	sockaddr_in serAddr;
	serAddr.sin_family = AF_INET;
	serAddr.sin_port = htons(8888);
	serAddr.sin_addr.S_un.S_addr = INADDR_ANY;
	if (bind(serSocket, (sockaddr *)&serAddr, sizeof(serAddr)) == SOCKET_ERROR)
	{
		printf("bind error !");
		closesocket(serSocket);
		return;
	}

	std::cout << "���ڵȴ�����" << std::endl;
	char recvData[255];                    // �ӵ�����
	int ret = recvfrom(serSocket, recvData, 255, 0, (sockaddr*)&remoteAddr, &nAddrLen);
	std::cout << "�ѽ��յ�����" << std::endl;
	if (ret > 0)
	{
		recvData[ret] = 0x00;
		printf("���ܵ�һ�����ӣ�%s \r\n", inet_ntoa(remoteAddr.sin_addr));
		printf(recvData);
		printf("����׼���������ݳ���\n");
	}

	// ѹ��ͼƬ
	std::vector<short> pointsCompressed(points.size()*6);
	for (size_t i = 0; i < points.size(); i++) {
		pointsCompressed[i * 6 + 0] = points[i].x * 500;
		pointsCompressed[i * 6 + 1] = points[i].y * 500;
		pointsCompressed[i * 6 + 2] = points[i].z * 500;
		pointsCompressed[i * 6 + 3] = points[i].r;
		pointsCompressed[i * 6 + 4] = points[i].g;
		pointsCompressed[i * 6 + 5] = points[i].b;
	}
	int leftBytes = pointsCompressed.size() * sizeof(short);
	int sendTimes = ceil(((float)leftBytes)/PACK_SIZE);
	
	char * temp = new char[PACK_SIZE];
	// ����ͼƬ
	for (size_t i = 0; i < sendTimes; i++) {
		unsigned int packLen = PACK_SIZE < leftBytes ? PACK_SIZE : leftBytes;
		
		memset(temp, 0, packLen);
		memcpy(temp, pointsCompressed.data() + i * PACK_SIZE / sizeof(short), packLen);
		sendto(serSocket, temp, packLen, 0, (sockaddr *)&remoteAddr, nAddrLen);
		leftBytes -= PACK_SIZE;
		std::cout << "package " << i << " has been sent" << std::endl;
	}
	std::cout << "���ͳɹ�" << std::endl;


	closesocket(serSocket);
	WSACleanup();
}

void Host::waitForMatrix(float matrix[][3], float vector[3])
{
	WSADATA wsaData;
	WORD sockVersion = MAKEWORD(2, 2);
	if (WSAStartup(sockVersion, &wsaData) != 0)
	{
		return;
	}

	serSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (serSocket == INVALID_SOCKET)
	{
		printf("socket error !");
		return;
	}
	sockaddr_in serAddr;
	serAddr.sin_family = AF_INET;
	serAddr.sin_port = htons(8888);
	serAddr.sin_addr.S_un.S_addr = INADDR_ANY;
	if (bind(serSocket, (sockaddr *)&serAddr, sizeof(serAddr)) == SOCKET_ERROR)
	{
		printf("bind error !");
		closesocket(serSocket);
		return;
	}


	char recvData[255];                    // ���վ���
	int ret = recvfrom(serSocket, recvData, 255, 0, (sockaddr*)&remoteAddr, &nAddrLen);
	if (ret > 0) {
		short matVec[9];
		memcpy(matVec, recvData, ret);
		for (size_t i = 0; i < 3; i++) {
			for (size_t j = 0; j < 3; j++) {
				matrix[i][j] = (float)matVec[i*3+j] / 500;
			}
		}
	}

	char recvMsg[255] = "���վ���ɹ�";
	sendto(serSocket, recvMsg, sizeof(unsigned int), 0, (sockaddr *)&remoteAddr, nAddrLen);

	
	ret = recvfrom(serSocket, recvData, 255, 0, (sockaddr*)&remoteAddr, &nAddrLen);
	if (ret > 0) {
		short Vec[3];
		memcpy(Vec, recvData, ret);
		for (size_t i = 0; i < 3; i++) {
			vector[i] = (float)Vec[i] / 500;
		}
	}


	//  ���ճɹ�
	char recvVecMsg[255] = "���������ɹ�";
	sendto(serSocket, recvVecMsg, sizeof(unsigned int), 0, (sockaddr *)&remoteAddr, nAddrLen);
}
