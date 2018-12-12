#pragma once
#include <vector>
#include <string>
#include <iostream>
#include "tinyxml2.h"
namespace tx = tinyxml2;


std::vector<std::string> getIPS(std::string filepath) {
	std::vector<std::string> ipVector;
	tx::XMLDocument doc;
	doc.LoadFile(filepath.c_str());

	tx::XMLElement * root = doc.FirstChildElement("root");
	int num = atoi(root->FirstChildElement("num")->GetText());

	tx::XMLElement* ips = root->FirstChildElement("ips");
	tx::XMLElement* ip = ips->FirstChildElement("ip");

	for (size_t i = 0; i < num; i++) {

		ipVector.push_back(ip->GetText());
		ip = ip->NextSiblingElement();
	}

	

	return ipVector;
}