#pragma once
#include <pcl/point_types.h>
#include <vector>
#include <pcl/io/boost.h>
namespace myMath {
	struct myPoint {
		float x;
		float y;
		float z;
		unsigned int r;
		unsigned int g;
		unsigned int b;
		myPoint() {
			x = 0.0;
			y = 0.0;
			z = 0.0;
			r = 0;
			g = 0;
			b = 0;
		}
		myPoint(const myPoint& p) {
			x = p.x;
			y = p.y;
			z = p.z;
			r = p.r;
			g = p.g;
			b = p.b;
		}
		void copyFromPCLPoint(pcl::PointXYZRGB point) {
			x = point.x;
			y = point.y;
			z = point.z;
			r = point.r;
			g = point.g;
			b = point.b;
		}

		void copyFromMyPoint(myPoint point) {
			x = point.x;
			y = point.y;
			z = point.z;
			r = point.r;
			g = point.g;
			b = point.b;
		}

	};

	struct myImage {
		std::vector<myPoint> points;
		int width;
		int height;

		myImage() {
			width = 0;
			height = 0;
		}

		myImage(const myImage& img) {
			this->resize(img.width, img.height);
			for (int i = 0; i < width*height; i++) {
				points[i].copyFromMyPoint(img.points[i]);
			}
		}

		void resize(int w, int h) {
			points.resize(w*h);
			width = w;
			height = h;
		}

		boost::shared_ptr<myImage> makeShared() {
			
			return boost::make_shared<myImage>(*this);
		}
	};
}