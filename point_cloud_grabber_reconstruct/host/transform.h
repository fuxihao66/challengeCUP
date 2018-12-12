#pragma once
#include "tinyxml2.h"
#include <string>
#include <iostream>
#include <vector>
namespace tx = tinyxml2;

void readConfig(std::string fileName, double matrix[][3], double vector[3]) {
	tx::XMLDocument doc;
	doc.LoadFile(fileName.c_str());


	tx::XMLElement * root = doc.FirstChildElement("root");

	tx::XMLElement * matrixNode = root->FirstChildElement("matrix");
	tx::XMLElement * mElement = matrixNode->FirstChildElement("element");


	for (size_t i = 0; i < 3; i++) {
		for (size_t j = 0; j < 3; j++) {
			matrix[i][j] = atof(mElement->GetText());
			printf("¾ØÕóÔªËØÊÇ£º%lf", matrix[i][j]);
			mElement = mElement->NextSiblingElement();
		}
	}


	tx::XMLElement* vectorNode = root->FirstChildElement("vector");
	tx::XMLElement * vElement = vectorNode->FirstChildElement("element");
	for (size_t i = 0; i < 3; i++) {
		vector[i] = atof(vElement->GetText());
		vElement = vElement->NextSiblingElement();
	}
}