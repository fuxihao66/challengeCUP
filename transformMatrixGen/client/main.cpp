
#include <iostream>
#include <opencv2/opencv.hpp>
#include <map>
#include "Client.h"
#include "tinyxml2.h"
#include "ReadIP.h"
namespace tx = tinyxml2;

#define IMG_HEIGHT 424
#define IMG_WIDTH 512
#define RED_START 170
#define RED_END 179
#define BLUE_START 100
#define BLUE_END 126
#define PINK_START 156
#define PINK_END 169
#define GREEN_START 35
#define GREEN_END 77
//#define YELLOW_START 26
//#define YELLOW_END 34

cv::Mat extractSpecificRange(int rangeStart, int rangeEnd, cv::Mat& originImage) {
	cv::Mat extracted;
	cv::inRange(originImage, cv::Scalar(rangeStart, 100, 100), cv::Scalar(rangeEnd, 255, 255), extracted);

	return extracted;
}



void generateMid(cv::Mat& image, cv::Point2i& mid) {
	int channels = image.channels();
	
	double x_sum = 0.0;
	double y_sum = 0.0;
	int count = 0;
	if (channels == 1) {
		for (int i = 0; i < image.rows; ++i) {
			for (int j = 0; j < image.cols; ++j){
				if (image.at<uchar>(i, j) > 200) {
					x_sum += j;
					y_sum += i;
					count++;
				}

			}
		}
	}

	mid.x = x_sum / count;
	mid.y = y_sum / count;
}



//

std::map<int, cv::Point3f> pointsToMap(std::vector<MyPoint>& points) {
	int numPoints = points.size();
	std::map<int, cv::Point3f> mapping;
	for (size_t i = 0; i < numPoints; i++) {
		mapping[i] = cv::Point3f(points[i].x, points[i].y, points[i].z);
	}

	return mapping;
}

void genMatirxAndVector(std::vector<MyPoint>& standard,
						std::vector<MyPoint>& nonstandard,
						float matrix[][3], float vector[3])
{
	std::map<int, cv::Point3f> map2DTo3D = pointsToMap(nonstandard);
	std::map<int, cv::Point3f> map2DTo3D_sta = pointsToMap(standard);
	cv::Mat rgb_image;
	cv::Mat rgb_image_sta;
	rgb_image.create(IMG_HEIGHT, IMG_WIDTH, CV_8UC3);
	rgb_image_sta.create(IMG_HEIGHT, IMG_WIDTH, CV_8UC3);
	for (size_t i = 0; i < IMG_HEIGHT; i++) {
		for (size_t j = 0; j < IMG_WIDTH; j++) {
			int index = i * IMG_WIDTH + j;
			rgb_image.at<cv::Vec3b>(i, j)[0] = nonstandard[index].b;
			rgb_image.at<cv::Vec3b>(i, j)[1] = nonstandard[index].g;
			rgb_image.at<cv::Vec3b>(i, j)[2] = nonstandard[index].r;

			rgb_image_sta.at<cv::Vec3b>(i, j)[0] = standard[index].b;
			rgb_image_sta.at<cv::Vec3b>(i, j)[1] = standard[index].g;
			rgb_image_sta.at<cv::Vec3b>(i, j)[2] = standard[index].r;
		}
	}
	cv::Mat hsv_image;
	cv::Mat hsv_image_sta;
	cv::cvtColor(rgb_image, hsv_image, cv::COLOR_BGR2HSV);
	cv::cvtColor(rgb_image_sta, hsv_image_sta, cv::COLOR_BGR2HSV);


	int colorRange[4][2] = {
		{RED_START, RED_END}, {GREEN_START, GREEN_END}, {PINK_START, PINK_END}, {BLUE_START, BLUE_END}
	};

	cv::Mat featurePointImage[4];
	cv::Mat featurePointStandard[4];

	for (size_t i = 0; i < 4; i++) {
		featurePointImage[i] = extractSpecificRange(colorRange[i][0], colorRange[i][1], hsv_image);
	}
	for (size_t i = 0; i < 4; i++) {
		featurePointStandard[i] = extractSpecificRange(colorRange[i][0], colorRange[i][1], hsv_image_sta);
	}

	cv::Point2i midPoints[4];
	cv::Point2i midPointsSta[4];
	for (size_t i = 0; i < 4; i++) {
		generateMid(featurePointImage[i], midPoints[i]);
	}
	for (size_t i = 0; i < 4; i++) {
		generateMid(featurePointStandard[i], midPointsSta[i]);
	}

	///*
	//solve linear equation group to get transforming matrix and vector
	//*/
	
	for (size_t i = 0; i < 3; i++) {
		cv::Mat A;
		A.create(4, 4, CV_32FC1);
		for (size_t j = 0; j < 4; j++) {
			int index = midPoints[j].x + midPoints[j].y*IMG_WIDTH;
			A.at<float>(j, 0) = map2DTo3D[index].x; // x
			A.at<float>(j, 1) = map2DTo3D[index].y; // y
			A.at<float>(j, 2) = map2DTo3D[index].z; // z
			A.at<float>(j, 3) = 1.0; // 1
		}

		std::cout << "matrix prepared well" << std::endl;

		cv::Mat b;
		b.create(4, 1, CV_32FC1);
		for (size_t j = 0; j < 4; j++) {
			int index = midPointsSta[j].x + midPointsSta[j].y*IMG_WIDTH;
			switch (i) {
			case 0:
				b.at<float>(j, 0) = map2DTo3D_sta[index].x;
				break;
			case 1:
				b.at<float>(j, 0) = map2DTo3D_sta[index].y;
				break;
			default:
				b.at<float>(j, 0) = map2DTo3D_sta[index].z;
			}
		}

		std::cout << "vector prepared well" << std::endl;

		cv::Mat result = A.inv()*b;   // 

		for (size_t j = 0; j < 3; j++) {
			matrix[i][j] = result.at<float>(j);
		}

		vector[i] = result.at<float>(3);
	}



}

//int main() {
//	cv::Mat bgr_img = cv::imread("data/red.jpg");
//	std::cout << bgr_img;
//	
//	return 0;
//
//}

//int main() {
//	std::cout << "程序开始" << std::endl;
//	auto ips = getIPS("ips.xml");
//	std::cout << ips[0] << std::endl;
//
//	assert(ips.size() == 1);
//
//
//
//	Client client;
//	std::vector<std::vector<MyPoint>> totalPoints(ips.size());     // points from all machines
//	// 发起请求  获取图片和映射
//	{
//		std::cout << "开始发起请求" << std::endl;
//		for (auto it = ips.begin(); it != ips.end(); it++) {
//			client.Request(*it);
//			totalPoints.push_back(client.getDataRecved());
//		}
//		std::cout << "request success" << std::endl;
//	}
//
//
//	// 处理图片 把矩阵发给服务端
//	{
//		for (size_t i = 1; i < ips.size(); i++) {
//			float matrix[3][3];
//			float vector[3];
//			genMatirxAndVector(totalPoints[0], totalPoints[i], matrix, vector);
//
//			client.sendMatrix(matrix, vector, ips[i]);
//		}
//
//		float identity[][3] = { {1.3, 1, 0}, {0, 1.0, 0}, {0, 0, 1.0} };
//		float nullVector[] = { 0.0, 0.0, 0.0 };
//		client.sendMatrix(identity, nullVector, ips[0]);                 // 以第0号为标准
//	}
//
//
//	std::cout << "发送成功，进程结束" << std::endl;
//
//	cv::waitKey(0);
//	return 0;
//}

int main(){

	std::cout << "程序开始" << std::endl;
	auto ips = getIPS("ips.xml");
	std::cout << ips[0] << std::endl;

	assert(ips.size() > 1);



	Client client;
	std::vector<std::vector<MyPoint>> totalPoints(ips.size());     // points from all machines
	// 发起请求  获取图片和映射
	{
		for (auto it = ips.begin(); it != ips.end(); it++) {
			client.Request(*it);
			totalPoints.push_back(client.getDataRecved());
		}
	}


	// 处理图片 把矩阵发给服务端
	{
		for (size_t i = 1; i < ips.size(); i++) {
			float matrix[3][3];
			float vector[3];
			genMatirxAndVector(totalPoints[0],totalPoints[i], matrix, vector);

			client.sendMatrix(matrix, vector, ips[i]);
		}

		float identity[][3] = { {1.0, 0, 0}, {0, 1.0, 0}, {0, 0, 1.0} };
		float nullVector[] = { 0.0, 0.0, 0.0 };
		client.sendMatrix(identity, nullVector, ips[0]);                 // 以第0号为标准
	}


	std::cout << "发送成功，进程结束" << std::endl;

	cv::waitKey(0);
	return 0;
}

//std::map<int, cv::Point3f> readConfig(std::string fileName) {
	//	std::map<int, cv::Point3f> mapping;
	//	tx::XMLDocument doc;
	//	doc.LoadFile(fileName.c_str());
	//
	//	tx::XMLElement * root = doc.FirstChildElement("root");
	//
	//	const char* w = root->FirstChildElement("width")->GetText();
	//	const char* h = root->FirstChildElement("height")->GetText();
	//	int width = atoi(w);
	//	int height = atoi(h);
	//
	//	tx::XMLElement* table = root->FirstChildElement("table");
	//
	//	auto rowNode = table->FirstChildElement("row");
	//	for (size_t i = 0; i < height; i++) {
	//		auto ele = rowNode->FirstChildElement("element");
	//		for (size_t j = 0; j < width; j++) {
	//			float x = atof(ele->FirstChildElement("x")->GetText());
	//			float y = atof(ele->FirstChildElement("y")->GetText());
	//			float z = atof(ele->FirstChildElement("z")->GetText());
	//			mapping[j+i*IMG_WIDTH] = cv::Point3f(x,y,z);
	//			ele = ele->NextSiblingElement();
	//		}
	//		rowNode = rowNode->NextSiblingElement();
	//	}
	//
	//
	//	return mapping;
	//}
void legacy(){
	// 处理图片 创建变换矩阵
	//cv::Mat bgr_img = cv::imread("cal.ppm");
	//cv::Mat bgr_img_sta = cv::imread("sta.ppm");
	//cv::Mat hsv_image;
	//cv::Mat hsv_image_sta;
	//cv::cvtColor(bgr_img, hsv_image, cv::COLOR_BGR2HSV);
	//cv::cvtColor(bgr_img_sta, hsv_image_sta, cv::COLOR_BGR2HSV);


	//int colorRange[4][2] = {
	//	{RED_START, RED_END}, {GREEN_START, GREEN_END}, {PINK_START, PINK_END}, {BLUE_START, BLUE_END}
	//};

	//cv::Mat featurePointImage[4];
	//cv::Mat featurePointStandard[4];

	//for (size_t i = 0; i < 4; i++) {
	//	featurePointImage[i] = extractSpecificRange(colorRange[i][0], colorRange[i][1], hsv_image);
	//}
	//for (size_t i = 0; i < 4; i++) {
	//	featurePointStandard[i] = extractSpecificRange(colorRange[i][0], colorRange[i][1], hsv_image_sta);
	//}

	//cv::Point2i midPoints[4];
	//cv::Point2i midPointsSta[4];
	//for (size_t i = 0; i < 4; i++) {
	//	generateMid(featurePointImage[i], midPoints[i]);
	//}
	//for (size_t i = 0; i < 4; i++) {
	//	generateMid(featurePointStandard[i], midPointsSta[i]);
	//}

	//
	//std::map<int, cv::Point3f> map2DTo3D = readConfig("local.xml");
	//std::map<int, cv::Point3f> map2DTo3D_sta = readConfig("standard.xml");

	//std::cout << "begin to generate transforming matrix" << std::endl;

	/////*
	////solve linear equation group to get transforming matrix and vector
	////*/
	//cv::Mat rotate; 
	//rotate.create(3, 3, CV_32FC1);
	//cv::Mat trans;
	//trans.create(3, 1, CV_32FC1);
	//for (size_t i = 0; i < 3; i++){
	//	cv::Mat A;
	//	A.create(4, 4, CV_32FC1);
	//	for (size_t j = 0; j < 4; j++) {
	//		int index = midPoints[j].x + midPoints[j].y*IMG_WIDTH;
	//		A.at<float>(j, 0) = map2DTo3D[index].x; // x
	//		A.at<float>(j, 1) = map2DTo3D[index].y; // y
	//		A.at<float>(j, 2) = map2DTo3D[index].z; // z
	//		A.at<float>(j, 3) = 1.0; // 1
	//	}
	//	
	//	std::cout << "matrix prepared well" << std::endl;

	//	cv::Mat b;
	//	b.create(4, 1, CV_32FC1);
	//	for (size_t j = 0; j < 4; j++) {
	//		int index = midPointsSta[j].x + midPointsSta[j].y*IMG_WIDTH;
	//		switch (i) {
	//		case 0:
	//			b.at<float>(j,0) = map2DTo3D_sta[index].x;
	//			break;
	//		case 1:
	//			b.at<float>(j,0) = map2DTo3D_sta[index].y;
	//			break;
	//		default:
	//			b.at<float>(j,0) = map2DTo3D_sta[index].z;
	//		}
	//	}

	//	std::cout << "vector prepared well" << std::endl;

	//	cv::Mat result = A.inv()*b;   // 

	//	for (size_t j = 0; j < 3; j++) {
	//		rotate.at<float>(i, j) = result.at<float>(j);
	//	}

	//	trans.at<float>(i, 0) = result.at<float>(3);
	//}



	///*
	//save matrix and vector to xml file
	//*/
	/*{
		tx::XMLDocument doc;
		tx::XMLElement* root = doc.NewElement("root");
		doc.InsertEndChild(root);

		tx::XMLElement* matrix = doc.NewElement("matrix");
		root->InsertEndChild(matrix);
		for (size_t i = 0; i < 3; i++) {
			for (size_t j = 0; j < 3; j++) {
				tx::XMLElement* element = doc.NewElement("element");
				element->SetText(rotate.at<float>(i,j));
				matrix->InsertEndChild(element);
			}
		}

		tx::XMLElement* vector = doc.NewElement("vector");
		root->InsertEndChild(vector);
		for (size_t i = 0; i < 3; i++) {
			tx::XMLElement* element = doc.NewElement("element");
			element->SetText(trans.at<float>(i));
			vector->InsertEndChild(element);
		}

		doc.SaveFile("mappingMatrix.xml");
	}*/
}