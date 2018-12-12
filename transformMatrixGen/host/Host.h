#pragma once
//#include "kinect2_grabber.h"
#include <winsock2.h> 
#pragma comment(lib,"ws2_32.lib")  
#include <algorithm>
#include <stdio.h>
#include <iostream>
#include <vector>
#include "myMath.h"
#define PACK_SIZE 60000
using namespace myMath;
class Host
{
private:
	SOCKET serSocket;
	sockaddr_in remoteAddr;
	int nAddrLen;
public:
	Host();
	~Host();

	void waitForReqForImg(const std::vector<myPoint>& points);   // 等待请求并发送图片信息
	void waitForMatrix(float matrix[][3], float vector[3]);      // 等待矩阵的传输
};

