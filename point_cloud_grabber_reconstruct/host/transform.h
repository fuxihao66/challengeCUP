#pragma once
#include "tinyxml2.h"
#include <string>
#include <iostream>
#include <vector>
namespace tx = tinyxml2;

void readConfig(std::string fileName, float& x, float& z, float& theta) {
	tx::XMLDocument doc;
	doc.LoadFile(fileName.c_str());

	tx::XMLElement * root = doc.FirstChildElement("root");

	const char* th = root->FirstChildElement("Theta")->GetText();
	theta = atof(th);

	tx::XMLElement* vectorNode = root->FirstChildElement("OriginDistance");

	z = atof(vectorNode->FirstChildElement("z")->GetText());
	x = atof(vectorNode->FirstChildElement("x")->GetText());
}