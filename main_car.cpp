#include "obst.h"
#include "RRT_car.h"
#include<opencv2/opencv.hpp>
#include<Windows.h>
#include<time.h>

using namespace leda;


int main()
{	
	system("color 9F");
	cv::Mat picture(500, 500, CV_8UC3, cv::Scalar(0,0,0));

	clock_t start, end;
	double time;

	GRAPH<car_q, car_u> RRT;
	car_q nearist_q;
	int k, k2;
	list<car_move> carPath;
	car_move carM0, carM1;
	car_q qInit, qGoal;
	car_u uInit;
	list<polygon> obsts;

	
	qInit.x = 50;
	qInit.y = 60;
	qInit.cita = 0;
	uInit.v = 0;
	uInit.fi = 0;
	carM0.q = qInit;
	carM0.u = uInit;
	carM1 = carM0;

	qGoal.x = 50;
	qGoal.y = 43;
	qGoal.cita = 0;

	picture=drawCar(picture,carM0,col_gb);
	carM0.q = qGoal;
	carM1 = carM0;
	picture=drawCar(picture,carM0,col_black);

	obsts = obst();

	picture = draw_obsts(obsts, picture);

	cv::imshow("obsts", picture);
	cv::waitKey(1);
	
	std::cout << "start RRT" << std::endl;

	list<car_q> search_list;
	GRAPH<car_q, car_u> goal_tree = Build_goal(qGoal, obsts, search_list, picture);
	std::cout << search_list.length() << std::endl;
	cv::imshow("obsts", picture);
	cv::waitKey(1);

	start = clock();

	RRT = Build_RRT(qInit, qGoal, obsts, search_list, nearist_q);

	carPath = Car_Path(qInit, qGoal, RRT, obsts, goal_tree, nearist_q);

	end = clock();
	time = (double)(end - start) / CLOCKS_PER_SEC;
	std::cout <<"time = "<< time << std::endl;

	carM0 = carPath.pop();

	char mode;
	std::cout << "please switch display mode , 'p' for Path , 'a' for Animation ." << std::endl;
	std::cin >> mode;

	int if_draw = 0;
	
	if (mode == 'p'|| mode == 'P')
	{
		while (!carPath.empty()) {
			carM1 = carPath.pop();

			if (if_draw % 2 == 0)

			{
				picture = drawCar(picture, carM1, col_gb);
				cv::imshow("obsts", picture);
				cv::waitKey(1);
			}

			if (carPath.length() == 0)
			{
				picture = drawCar(picture, carM1, col_red);
				cv::imshow("obsts", picture);
				cv::waitKey(1);
			}
			if_draw++;
		}
	}
	if (mode == 'a'|| mode == 'A')
	{
		while (!carPath.empty()) {
			carM1 = carPath.pop();

			if (if_draw % 2 == 0)

			{
				picture = cv::Mat(500, 500, CV_8UC3, cv::Scalar(0, 0, 0));
				picture = draw_obsts(obsts, picture);
				picture = drawCar(picture, carM1, col_gb);
				cv::imshow("obsts", picture);
				cv::waitKey(1);
			}

			if (carPath.length() == 0)
			{
				picture = drawCar(picture, carM1, col_red);
				cv::imshow("obsts", picture);
				cv::waitKey(1);
			}
			if_draw++;
			Sleep(40);
		}
	}
	std::cout << if_draw << std::endl;
	
	//cv::waitKey(0);
	system("pause");
	return 0;
}