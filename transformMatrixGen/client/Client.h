#pragma once
#include "MyMath.h"
#include <vector>
#include <string>
#include <iostream>
#include <winsock2.h> 
#pragma comment(lib,"ws2_32.lib")  

#define PACK_SIZE 60000

class Client
{
private:
	std::vector<MyPoint> pointsRecved;

	SOCKET sclient;
	sockaddr_in sin;
	int len;
public:
	Client();
	~Client();

	std::vector<MyPoint> getDataRecved();
	void sendMatrix(float matrix[][3], float vector[3], std::string ip);
	void Request(std::string ip);
};

