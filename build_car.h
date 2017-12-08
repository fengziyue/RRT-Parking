#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <complex.h>
#include <time.h>
#include<opencv2/opencv.hpp>

// Things that are used from the LEDA library
#include <LEDA/graph/graph.h>
#include <LEDA/core/list.h>
#include <LEDA/geo/point.h>
#include <LEDA/geo/polygon.h>
#include <LEDA/graphics/ps_file.h>
#include <LEDA/core/random_source.h>
#include <LEDA/graphics/window.h>

#define PI 3.1415926
#define carL 10
#define MAXfi PI/6

using namespace leda;

window W2(625, 625);
cv::Scalar col_white(255, 255, 255), col_gb(0, 127, 127), col_black(255, 0, 0), col_grey(127, 127, 127), col_red(0, 0, 255), col_green(0, 127, 0);

//------------------ car_move classes -----------------------
class car_u {
public:
	double v;
	double fi;
public:
	friend std::ostream& operator<<(std::ostream& os, car_u u);
	friend std::istream& operator>>(std::istream& is, car_u &u);
};

std::ostream& operator<<(std::ostream& os, car_u u)
{
	os << "Car Speed = " << u.v << ", Turnning Angle = " << u.fi;
	return os;
}

std::istream& operator>>(std::istream& is, car_u &u)
{
	is >> u.v >> u.fi;
	return is;
}

class car_q {
public:
	double x;
	double y;
	double cita;
public:
	friend std::ostream& operator<<(std::ostream& os, car_q q);
	friend std::istream& operator>>(std::istream& is, car_q &q);
};

std::ostream& operator<<(std::ostream& os, car_q q)
{
	os << "Car Position: x = " << q.x << ", y = " << q.y << ", cita = " << q.cita;
	return os;
}

std::istream& operator>>(std::istream& is, car_q &q)
{
	is >> q.x >> q.y >> q.cita;
	return is;
}

class car_move {
public:
	car_q q;
	car_u u;
public:
	friend std::ostream& operator<<(std::ostream& os, car_move m);
	friend std::istream& operator>>(std::istream& is, car_move &m);
};

std::ostream& operator<<(std::ostream& os, car_move m)
{
	os << "Car Position: x = " << (m.q).x << ", y = " << (m.q).y << ", cita = " << (m.q).cita << std::endl << "Car Speed = " << (m.u).v << ", Turnning Angle = " << (m.u).fi;
	return os;
}

std::istream& operator>>(std::istream& is, car_move &m)
{
	is >> (m.q).x >> (m.q).y >> (m.q).cita >> (m.u).v >> (m.u).fi;
	return is;
}
//=========================================================================

//---------------------------- draw Car ---------------------------------
polygon drawWheel(vector w, double a, vector T, double cita)
{
	list<point> car_wheel;
	polygon carWheel;

	car_wheel.clear();

	car_wheel.push(point(-1.5, -0.8));
	car_wheel.push(point(-1.5, 0.8));
	car_wheel.push(point(1.5, 0.8));
	car_wheel.push(point(1.5, -0.8));

	carWheel = polygon(car_wheel);
	carWheel = carWheel.rotate(a);
	carWheel = carWheel + w;
	carWheel = carWheel + T;
	carWheel = carWheel.rotate(point(T.xcoord(), T.ycoord()), cita);

	//W2.draw_filled_polygon(carWheel);

	return(carWheel);
}

polygon drawCarBody(vector T, double cita)
{
	list<point> car_body;
	polygon carBody;

	car_body.clear();

	car_body.push(point(-7, -3));
	car_body.push(point(-7, 3));
	car_body.push(point(7, 3));
	car_body.push(point(7, -3));

	carBody = polygon(car_body);
	carBody = carBody.rotate(cita);
	carBody = carBody + T;

	return(carBody);

}

cv::Mat drawCar(cv::Mat picture, car_move carM0, cv::Scalar col_x)
{
	/*
	int i;
	polygon carBody;
	
	array<segment> H(4), Hj(4);
	array<vector> w(4);
	double a[4] = { 0,0,0,0 };
	vector T, T0, T1;
	double cita, cita0, cita1;
	double fi0;

	T0 = vector((carM0.q).x, (carM0.q).y);
	cita0 = (carM0.q).cita;
	T1 = vector((carM1.q).x, (carM1.q).y);
	cita1 = (carM1.q).cita;

	fi0 = (carM1.u).fi;
	a[0] = fi0;
	a[1] = fi0;

	H[0] = segment(0, -2.5, 0, 2.5);
	H[1] = segment(10, -2.5, 10, 2.5);
	H[2] = segment(0, -0.8, 10, -0.4);
	H[3] = segment(0, 0.8, 10, 0.4);

	w[0] = vector(10, 2.5);
	w[1] = vector(10, -2.5);;
	w[2] = vector(0, -2.5);
	w[3] = vector(0, 2.5);

	double dcita;

	dcita = cita1 - cita0;
	if (dcita>PI)
		dcita -= 2 * PI;
	else if (dcita<-PI)
		dcita += 2 * PI;
	for (int j = 0; j<2; j++) {

		if (j >= 1) {
			W2.set_color(col_white);
			W2.draw_polygon(carBody);
			for (i = 0; i<4; i++)
			{
				W2.draw_segment(Hj[i]);
				drawWheel(w[i], a[i], T, cita);
			}

		}

		W2.set_color(col_x);
		T = T0 + j*(T1 - T0) / 1;
		cita = cita0 + j*(dcita) / 1;

		carBody = drawCarBody(T, cita);
		W2.draw_polygon(carBody);

		for (i = 0; i<4; i++)
		{
			Hj[i] = H[i] + T;
			Hj[i] = Hj[i].rotate(point(T.xcoord(), T.ycoord()), cita);
			W2.draw_segment(Hj[i]);
			W2.set_color(col_black);
			drawWheel(w[i], a[i], T, cita);
			W2.set_color(col_x);
		}

		for (i = 0; i<10000; i++)W2.set_color(col_black);
		if (T0 == T1&&cita0 == cita1)break;

	}
	*/
	vector T = vector((carM0.q).x, (carM0.q).y);
	polygon car_body = drawCarBody(T, (carM0.q).cita);
	list<point> car_body_points = car_body.vertices();
	cv::Point points[1][4];
	for (int i = 0; i < 4; i++)
	{
		point cur_point = car_body_points.front();
		car_body_points.pop();
		points[0][i] = cv::Point(cur_point.xcoord()*5, cur_point.ycoord()*5);
	}
	cv::Point* pt[1] = { points[0] };
	int npt[1] = { 4 };
	polylines(picture, pt, npt, 1, 1, col_x, 1);

	std::cout << std::endl;
	std::cout << "draw_car:" << carM0 << std::endl;
	std::cout << std::endl;

	return picture;
}

int outBox(polygon P)
{
	point v;

	forall_vertices(v, P) {
		if (v.xcoord()<0 || v.xcoord()>100 || v.ycoord()<0 || v.ycoord()>100) {
			return(1);
		}
	}

	return(0);
}

