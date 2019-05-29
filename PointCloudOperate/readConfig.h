#pragma once
#include "tinyxml2.h"
#include <string>
#include <iostream>
#include <vector>
namespace tx = tinyxml2;


inline void readConfig(std::string fileName, int& numClouds, float& minX, float&  maxX, float& minY,
					float& maxY, float& minZ, float& maxZ, float& statisticalRemovalStd,
					float& voxelFilterLeafSize, float& resampleRadius) {
	tx::XMLDocument doc;
	doc.LoadFile(fileName.c_str());

	tx::XMLElement * root = doc.FirstChildElement("root");

	const char* num = root->FirstChildElement("numClouds")->GetText();
	numClouds = atoi(num);

	tx::XMLElement* userNode = root->FirstChildElement("CuttingBox");

	minX = atof( userNode->FirstChildElement("minX")->GetText() );
	minX = atof(userNode->FirstChildElement("minX")->GetText());
	minX = atof(userNode->FirstChildElement("minX")->GetText());
	minX = atof(userNode->FirstChildElement("minX")->GetText());
	minX = atof(userNode->FirstChildElement("minX")->GetText());
	minX = atof(userNode->FirstChildElement("minX")->GetText());
	
	statisticalRemovalStd = atof(root->FirstChildElement("statisticalRemovalStd")->GetText());
	voxelFilterLeafSize = atof(root->FirstChildElement("voxelFilterLeafSize")->GetText());
	resampleRadius = atof(root->FirstChildElement("resampleRadius")->GetText());

}