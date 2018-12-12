#pragma once

struct MyPoint {
	unsigned int r;
	unsigned int g;
	unsigned int b;
	float x;
	float y;
	float z;

	MyPoint() {
		x = 0.0;
		y = 0.0;
		z = 0.0;
		r = 0;
		g = 0;
		b = 0;
	}
	MyPoint(const MyPoint& p) {
		x = p.x;
		y = p.y;
		z = p.z;
		r = p.r;
		g = p.g;
		b = p.b;
	}
};