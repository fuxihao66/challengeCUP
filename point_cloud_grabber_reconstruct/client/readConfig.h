#pragma once
#include "tinyxml2.h"
#include <string>
#include <iostream>
#include <vector>
namespace tx = tinyxml2;


inline std::vector<std::string> readConfig(std::string fileName) {
	tx::XMLDocument doc;
	doc.LoadFile(fileName.c_str());

	std::vector<std::string> ips;

	tx::XMLElement * root = doc.FirstChildElement("root");

	const char* num = root->FirstChildElement("num")->GetText();
	int numOfKinect = atoi(num);

	tx::XMLElement* userNode = root->FirstChildElement("ipconfig");



	auto ipNode = userNode->FirstChildElement("ip");
	for (size_t i = 0; i < numOfKinect; i++) {
		ips.push_back(ipNode->GetText());
		ipNode = ipNode->NextSiblingElement();
	}

	return ips;
}