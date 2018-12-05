// Disable Error C4996 that occur when using Boost.Signals2.
#ifdef _DEBUG
#define _SCL_SECURE_NO_WARNINGS
#endif
//

//#include "main.h"
#include "kinect2_grabber.h"
#include "tinyxml2.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <string>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <Kinect.h> 
using namespace std;


int main(void)
{
	boost::shared_ptr<myMath::myImage> image = nullptr;
	int h, w;
	boost::mutex mutex;


	boost::function<void(boost::shared_ptr<myMath::myImage>& imageData)> function = [&image, &mutex, &h, &w](boost::shared_ptr<myMath::myImage>& imageData) {
		boost::mutex::scoped_lock lock(mutex);
		
		image = imageData->makeShared();
		h = imageData->height;
		w = imageData->width;
		
	};

	

	// Kinect2Grabber
	boost::shared_ptr<pcl::Grabber> grabber = boost::make_shared<pcl::Kinect2Grabber>();

	// Register Callback Function
	boost::signals2::connection connection = grabber->registerCallback(function);

	// Start Grabber
	grabber->start();



	while (1) {
		std::cout << "initializing" << std::endl;
		if (image)
			break;
	}

	{
		boost::mutex::scoped_lock lock(mutex);
		FILE *f = fopen("image.ppm", "w");         // Write image to PPM file.
		fprintf(f, "P3\n%d %d\n%d\n", w, h, 255);
		for (int i = 0; i < image->points.size(); i++) {
			
			fprintf(f, "%d %d %d ", image->points[i].r, image->points[i].g, image->points[i].b);
		}
	}

	{
		boost::mutex::scoped_lock lock(mutex);
		// write mapping 
		tinyxml2::XMLDocument doc;
		tinyxml2::XMLElement* root = doc.NewElement("root");
		doc.InsertEndChild(root);

		tinyxml2::XMLElement* width = doc.NewElement("width");
		width->SetText(w);
		root->InsertEndChild(width);

		tinyxml2::XMLElement* height = doc.NewElement("height");
		height->SetText(h);
		root->InsertEndChild(height);

		tinyxml2::XMLElement* table = doc.NewElement("table");
		root->InsertEndChild(table);

		for (size_t i = 0; i < h; i++) {
			tinyxml2::XMLElement* row = doc.NewElement("row");
			table->InsertEndChild(row);
			for (size_t j = 0; j < w; j++){
				tinyxml2::XMLElement* element = doc.NewElement("element");
				row->InsertEndChild(element);
				tinyxml2::XMLElement* x_axis = doc.NewElement("x");
				tinyxml2::XMLElement* y_axis = doc.NewElement("y");
				tinyxml2::XMLElement* z_axis = doc.NewElement("z");
				x_axis->SetText(image->points[i*w + j].x);
				y_axis->SetText(image->points[i*w + j].y);
				z_axis->SetText(image->points[i*w + j].z);
				element->InsertEndChild(x_axis);
				element->InsertEndChild(y_axis);
				element->InsertEndChild(z_axis);
			}
		}

		doc.SaveFile("mapping.xml");
	}


	std::cout << "has been initiated" << std::endl;

	// Stop Grabber
	grabber->stop();

	// Disconnect Callback Function
	if (connection.connected()) {
		connection.disconnect();
	}

	return 0;

}
