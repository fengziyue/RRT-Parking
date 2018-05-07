#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <iostream>
//#include"build_car.h"
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

using namespace leda;

#define max_num_Vertices 100
#define max_num_Polygons 100
#define PI 3.1415926

//extern window W;
//extern window W2;
extern cv::Scalar col_white, col_gb, col_black, col_grey, col_red, col_green;

using namespace leda;

int cmp(point const& p1, point const&  p2)
{
	if (p1.xcoord()<p2.xcoord())
		return(-1);
	else if (p1.xcoord()>p2.xcoord())
		return(1);
	else if (p1.ycoord()<p2.ycoord())
		return(-1);
	else return(1);
}

list<polygon> u();
list<polygon> n();
list<polygon> c();

list<polygon> obst(void)
{
	/*
	int i, k, n = 100;

	int st = 0;

	array<point> temp(3);
	array<point> p(max_num_Vertices);
	list<point> L_upper, L_lower;

	//ps_file F(15.0,15.0,"hw1.ps");

	//*************** Obstacles(Polygons) **************************
	while (st == 0)
	{
		i = -1;
		//-------draw a convex-----------
		while (1)
		{
			i++;
			k = W2.get_mouse(p[i]);
			if (k == MOUSE_BUTTON(2)) { n = i; break; }
			else if (k == MOUSE_BUTTON(1))
			{
				W2.draw_point(p[i]);
			}
			else if (k == MOUSE_BUTTON(3)) { n = i; st = 1; break; }
			else i--;
		}


		p.sort(cmp, 0, n - 1);

		L_upper.clear();
		L_upper.clear();

		L_upper.push(p[0]);
		L_upper.push(p[1]);

		int a0 = 0;
		int a1 = 1;
		int a2 = 2;

		for (i = 2; i<n; i++) {
			L_upper.push(p[i]);

			while (L_upper.length()>2) {
				temp[0] = L_upper.contents(L_upper[a0]);
				temp[1] = L_upper.contents(L_upper[a1]);
				temp[2] = L_upper.contents(L_upper[a2]);

				if (!right_turn(temp[2], temp[1], temp[0])) {
					L_upper.erase(L_upper[1]);
					//      				m=L_upper.length(); 
					//      				cout << m;
				}
				else break;
			}
		}

		L_upper.reverse_items();

		L_lower.push(p[n - 1]);
		L_lower.push(p[n - 2]);

		for (i = n - 3; i >= 0; i--) {
			L_lower.push(p[i]);
			while (L_lower.length()>2) {
				temp[0] = L_lower.contents(L_lower[a0]);
				temp[1] = L_lower.contents(L_lower[a1]);
				temp[2] = L_lower.contents(L_lower[a2]);

				if (!right_turn(temp[2], temp[1], temp[0])) {
					L_lower.erase(L_lower[1]);
				}
				else break;
			}
		}

		L_lower.pop();
		L_lower.Pop();

		L_lower.reverse_items();
		L_upper.conc(L_lower);

		//	L_upper.print();

		plys.push(polygon(L_upper));
		W.draw_filled_polygon(polygon(L_upper), col_grey);
		W2.draw_filled_polygon(polygon(L_upper), col_grey);

		//F.draw_polygon(L_upper);
		//a convex done

	}//============while(st==0)===============
	*/
	char map;
	std::cout << "which map? c,u,n" << std::endl;
	std::cin >> map;
	list<polygon> plys;
	if (map == 'u')
		plys = u();
	else if (map == 'n')
		plys = n();
	else if (map == 'c')
		plys = c();

	std::cout << "obst " << std::endl;

	return plys;

}
cv::Mat draw_obsts(list<polygon> leda_obsts, cv::Mat picture)
{
	int num = leda_obsts.length();
	int npt[20];
	cv::Point points[20][30];

	polygon cur_poly;
	list<point> cur_poly_points;
	for (int i = 0; i < num; i++)
	{
		cur_poly = leda_obsts.front();
		leda_obsts.pop();
		cur_poly_points = cur_poly.vertices();
		int num_points = cur_poly_points.length();
		npt[i] = num_points;
		for (int j = 0; j < num_points; j++)
		{
			point cur_point = cur_poly_points.front();
			cur_poly_points.pop();
			points[i][j] = cv::Point(cur_point.xcoord()*5, cur_point.ycoord()*5);
		}
	}
	cv::Point* pt[20];
	for (int i = 0; i < num; i++)
	{
		pt[i] = points[i];
	}
	cv::polylines(picture, pt, npt, num, 1, cv::Scalar(255, 0, 0), 1);
	//cv::fillPoly(picture, &pt[0], &npt[0], num, cv::Scalar(200, 0, 0), 8);
	return picture;
}

list<polygon> u()
{
	list<polygon> plys;
	polygon ply;
	list<point> car_wheel;

	car_wheel.clear();

	car_wheel.push(point(40, 50));
	car_wheel.push(point(40, 55));
	car_wheel.push(point(60, 55));
	car_wheel.push(point(60, 50));

	ply = polygon(car_wheel);
	plys.push(ply);

	car_wheel.clear();

	car_wheel.push(point(34, 40));
	car_wheel.push(point(34, 56));
	car_wheel.push(point(40, 56));
	car_wheel.push(point(40, 40));

	ply = polygon(car_wheel);
	plys.push(ply);

	car_wheel.clear();

	car_wheel.push(point(60, 40));
	car_wheel.push(point(60, 56));
	car_wheel.push(point(66, 56));
	car_wheel.push(point(66, 40));

	ply = polygon(car_wheel);
	plys.push(ply);
	return plys;
}

list<polygon> n()
{
	list<polygon> plys;
	polygon ply;
	list<point> car_wheel;

	car_wheel.clear();

	car_wheel.push(point(40, 35));
	car_wheel.push(point(40, 38));
	car_wheel.push(point(60, 38));
	car_wheel.push(point(60, 35));

	ply = polygon(car_wheel);
	plys.push(ply);

	car_wheel.clear();

	car_wheel.push(point(34, 35));
	car_wheel.push(point(34, 45));
	car_wheel.push(point(38, 45));
	car_wheel.push(point(38, 35));

	ply = polygon(car_wheel);
	plys.push(ply);

	car_wheel.clear();

	car_wheel.push(point(62, 35));
	car_wheel.push(point(62, 45));
	car_wheel.push(point(66, 45));
	car_wheel.push(point(66, 35));

	ply = polygon(car_wheel);
	plys.push(ply);
	return plys;
}

list<polygon> c()
{
	list<polygon> plys;
	polygon ply;
	list<point> car_wheel;

	car_wheel.clear();

	car_wheel.push(point(40, 35));
	car_wheel.push(point(40, 38));
	car_wheel.push(point(60, 38));
	car_wheel.push(point(60, 35));

	ply = polygon(car_wheel);
	plys.push(ply);

	car_wheel.clear();

	car_wheel.push(point(40, 48));
	car_wheel.push(point(40, 55));
	car_wheel.push(point(60, 55));
	car_wheel.push(point(60, 48));

	ply = polygon(car_wheel);
	plys.push(ply);

	car_wheel.clear();

	car_wheel.push(point(35, 35));
	car_wheel.push(point(35, 50));
	car_wheel.push(point(40, 50));
	car_wheel.push(point(40, 35));

	ply = polygon(car_wheel);
	plys.push(ply);
	return plys;
}






