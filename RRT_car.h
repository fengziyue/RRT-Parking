#include "build_car.h"
#include<Windows.h>
using namespace leda;

int A = carL*carL;
double dt = 1;
random_source R;
//window W;


//=================================================================

double dist(car_q p, car_q q)
{
	double d;
	double dcita;
	
	double R = 1.4*carL;
	
	point Q1 = point(p.x + cos(p.cita - 0.5*PI)*R, p.y + sin(p.cita - 0.5*PI)*R);
	point Q2 = point(p.x + cos(p.cita + 0.5*PI)*R, p.y + sin(p.cita + 0.5*PI)*R);

	bool if_near = sqrt((q.x - Q1.xcoord())*(q.x - Q1.xcoord()) + (q.y - Q1.ycoord())*(q.y - Q1.ycoord())) > 11 && sqrt((q.x - Q2.xcoord())*(q.x - Q2.xcoord()) + (q.y - Q2.ycoord())*(q.y - Q2.ycoord())) > 11;
	//near = 1;

	if (if_near)
	{
		dcita = p.cita - q.cita;
		if (dcita>PI)
			dcita -= 2 * PI;
		else if (dcita<-PI)
			dcita += 2 * PI;

		d = (p.x - q.x)*(p.x - q.x) + (p.y - q.y)*(p.y - q.y) + A*dcita*dcita;
		d = pow(d, 0.5);
	}
	else
	{
		d = PI*R;
	}
	//std::cout << near << std::endl;
	return(d);
}

node Nearest_Vertex(car_q q, GRAPH<car_q, car_u> &G)
{

	car_q   vr;
	node  vn_node, vr_node;
	double d = 10000, dr;

	forall_nodes(vr_node, G) {
		vr = G.inf(vr_node);
		dr = dist(vr, q);
		if (dr < d) {
			vn_node = vr_node;
			d = dr;
		}
	}
	return(vn_node);
}

car_q Rand_Conf()
{
	car_q q;
	q.x = -1;
	q.y = -1;
	q.cita = -5;

	while (q.x<0 || q.x>100) {
		R >> q.x;
		q.x *= 100;
	}

	while (q.y<0 || q.y>100) {
		R >> q.y;
		q.y *= 100;
	}

	while (q.cita<-PI || q.cita >= PI) {
		R >> q.cita;
		q.cita = q.cita * 2 * PI - PI;
	}
	return(q);

}

car_q Goal_Region_Biased_Conf(car_q goal, double PG, double d)
{
	double x;
	R >> x;
	car_q q;

	if (x< PG)
		return (goal);

	else if (x< 0.5) {
		q = Rand_Conf();
		while (dist(q, goal)>2 * d)
			q = Rand_Conf();
		return(q);
	}
	else return(Rand_Conf());

}

car_q Goal_Tree_Biased_Conf(list<car_q> &cur_list, list<car_q> &search_list, int &if_to_tree)
{
	double x;
	R >> x;
	car_q q;

	if (x < 0.5)
	{
		q = Rand_Conf();
		if_to_tree = 0;
	}
	else
	{
		if (cur_list.empty())
			cur_list = search_list;
		q = cur_list.pop();
		if_to_tree = 1;
	}
	return q;
}

car_q New_conf(car_q vn, car_u u)
{
	car_q new_q;
	
	new_q.x = vn.x + u.v*dt*cos(vn.cita)*cos(u.fi);
	new_q.y = vn.y + u.v*dt*sin(vn.cita)*cos(u.fi);
	new_q.cita = vn.cita + (u.v*dt / carL)*sin(u.fi);
	
	//Sleep(100);
	//std::cout << vn << "   " << u << std::endl;

	return(new_q);
}

car_u Optm_Input(car_q vn, car_q q, list<polygon> obsts)
{
	int d1, d2, dr, d = 10000;
	int k1 = 0, k2 = 0;
	car_q new_q, p1, p2, p;
	car_u optm_u, u1, u2, u;
	polygon carBody, obs;
	list<segment> obs_segs;
	segment obs_seg;
	list_item it;
	double fi[3];
	int v[2] = { 1, -1 };

	fi[0] = atan(carL*(vn.cita - q.cita) / ((vn.x - q.x)*cos(vn.cita) + (vn.y - q.y)*sin(vn.cita)));
	fi[1] = atan(carL*(vn.cita - q.cita + 2 * PI) / ((vn.x - q.x)*cos(vn.cita) + (vn.y - q.y)*sin(vn.cita)));
	fi[2] = atan(carL*(vn.cita - q.cita - 2 * PI) / ((vn.x - q.x)*cos(vn.cita) + (vn.y - q.y)*sin(vn.cita)));

	for (int i = 0; i<2; i++)
		for (int j = 0; j<3; j++) {
			u.v = v[i];
			u.fi = fi[j];
			if (u.fi<-MAXfi)
				u.fi = -MAXfi;
			else if (u.fi>MAXfi)
				u.fi = MAXfi;
			p = New_conf(vn, u);
			dr = dist(p, q);
			if (dr<d) {
				d = dr;
				new_q = p;
				optm_u = u;
			}
		}


	u1 = optm_u;
	u2 = optm_u;

	carBody = drawCarBody(vector(new_q.x, new_q.y), new_q.cita);


	if (outBox(carBody) == 1) {
		u1.fi = u1.fi + PI / 20;
		u2.fi = u2.fi - PI / 20;
		p1 = New_conf(vn, u1);
		p2 = New_conf(vn, u2);
		k1 = 1;
		k2 = 1;
	}

	else {
		it = obsts.first();
		while (it != nil) {
			obs = obsts.inf(it);
			//intersect = obs.intersection(carBody);
			int is_intersect = 0;
			obs_segs = obs.segments();
			while (!obs_segs.empty()) {
				obs_seg = obs_segs.front();
				obs_segs.pop();
				list<point> inter = carBody.intersection(obs_seg);
				int inter_empty = inter.empty();
				if (!inter_empty)
				{
					is_intersect = 1;
					break;
				}
			}
			if (is_intersect) {
				u1.fi = u1.fi + PI / 20;
				u2.fi = u2.fi - PI / 20;
				p1 = New_conf(vn, u1);
				p2 = New_conf(vn, u2);
				k1 = 1;
				k2 = 1;
				break;
			}
			it = obsts.succ(it);
		}
	}

	while (k1 == 1) {
		k1 = 0;
		d1 = dist(p1, q);
		carBody = drawCarBody(vector(p1.x, p1.y), p1.cita);

		if (outBox(carBody) == 1) {
			u1.fi = u1.fi + PI / 20;
			p1 = New_conf(vn, u1);
			k1 = 1;
		}
		else {
			it = obsts.first();
			while (it != nil) {
				obs = obsts.inf(it);
				//intersect = obs.intersection(carBody);
				int is_intersect = 0;
				forall_segments(obs_seg, obs) {
					list<point> inter = carBody.intersection(obs_seg);
					int inter_empty = inter.empty();
					if (!inter_empty)
					{
						is_intersect = 1;
						break;
					}
				}
				if (is_intersect) {
					u1.fi = u1.fi + PI / 20;
					p1 = New_conf(vn, u1);
					k1 = 1;
					break;
				}
				it = obsts.succ(it);
			}
		}
		if (abs(u1.fi)>MAXfi) {
			d1 = 10000;
			break;
		}
	}

	while (k2 == 1) {
		k2 = 0;
		d2 = dist(p2, q);
		carBody = drawCarBody(vector(p2.x, p2.y), p2.cita);

		if (outBox(carBody) == 1) {
			u2.fi = u2.fi - PI / 20;
			p2 = New_conf(vn, u2);
			k2 = 1;
		}
		else {
			it = obsts.first();
			while (it != nil) {
				obs = obsts.inf(it);
				//intersect = obs.intersection(carBody);
				int is_intersect = 0;
				forall_segments(obs_seg, obs) {
					list<point> inter = carBody.intersection(obs_seg);
					int inter_empty = inter.empty();
					if (!inter_empty)
					{
						is_intersect = 1;
						break;
					}
				}
				if (is_intersect) {
					u2.fi = u2.fi - PI / 20;
					p2 = New_conf(vn, u2);
					k2 = 1;
					break;
				}
				it = obsts.succ(it);
			}
		}

		if (abs(u2.fi)>MAXfi) {
			d2 = 10000;
			break;
		}
	}

	if (d1 == 10000 && d2 == 10000)
		optm_u.v = 0;
	else if (d1 <= d2)
		optm_u = u1;
	else optm_u = u2;

	return(optm_u);

}



GRAPH<car_q, car_u> Build_RRT(car_q q_init, car_q goal, list<polygon> obsts, list<car_q>search_list, car_q &nearist_q)
{
	GRAPH<car_q, car_u> G;
	car_q q, vn;
	node vn_node, qn_node;
	car_q new_q;
	car_u new_u;
	list<car_q> cur_list = search_list;
	int if_to_tree = 0;
	double d = 10000;
	double cur_d;
	char c;

	G.new_node(q_init);

	while (d>0.5) {
		for (int i = 0; i<1000; i++) {
			q = Goal_Tree_Biased_Conf(cur_list, search_list, if_to_tree);
			vn_node = Nearest_Vertex(q, G);
			vn = G.inf(vn_node);

			new_u = Optm_Input(vn, q, obsts);
			new_q = New_conf(vn, new_u);

			qn_node = G.new_node(new_q);
			G.new_edge(vn_node, qn_node, new_u);
			if (if_to_tree)
			{
				double cur_d = dist(q, new_q);
				if (cur_d < d)
				{
					nearist_q = new_q;
					d = cur_d;
				}
			}
		}
		std::cout << d << std::endl;
	}

	return(G);
}


list<car_move> Car_Path(car_q q_init, car_q goal, GRAPH<car_q, car_u> &G, list<polygon> obsts, GRAPH<car_q, car_u> &goal_tree, car_q nearist_q)
{
	std::cout << "car_path:" << std::endl;
	node vn_node, root_node;
	edge e;
	car_q vn;
	list<car_move> init_path, goal_path, path, init_path_inverse;
	car_move carM;
	polygon obs;

	vn_node = Nearest_Vertex(nearist_q, G);

	root_node = G.first_node();

	while (vn_node != root_node) {
		e = G.first_in_edge(vn_node);
		carM.q = G.inf(vn_node);
		carM.u = G.inf(e);
		init_path.push(carM);
		vn_node = G.source(e);
		vn = G.inf(vn_node);
		
		//std::cout << "car_path from " << vn.x << "," << vn.y << " to " << (carM.q).x << "," << (carM.q).y << std::endl;
		
	}
	

	carM.q = G.inf(vn_node);
	(carM.u).v = 0;
	(carM.u).fi = 0;
	init_path.push(carM);

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	vn_node = Nearest_Vertex(nearist_q, goal_tree);

	root_node = goal_tree.first_node();

	while (vn_node != root_node) {
		e = goal_tree.first_in_edge(vn_node);
		carM.q = goal_tree.inf(vn_node);
		carM.u = goal_tree.inf(e);
		goal_path.push(carM);
		vn_node = goal_tree.source(e);
		vn = goal_tree.inf(vn_node);

		//std::cout << "car_path from " << vn.x << "," << vn.y << " to " << (carM.q).x << "," << (carM.q).y << std::endl;

	}

	while (!init_path.empty())
	{
		init_path_inverse.push(init_path.pop());
	}
	while (!goal_path.empty())
	{
		path.push(goal_path.pop());
	}
	while (!init_path_inverse.empty())
	{
		path.push(init_path_inverse.pop());
	}
	return(path);
}


GRAPH<car_q,car_u> Build_goal(car_q goal, list<polygon> obsts, list<car_q> &search_list,cv::Mat &picture)
{
	GRAPH<car_q, car_u> G;
	node root_node = G.new_node(goal);
	double angle;
	double depth = 10;
	car_q q;
	car_u u;
	node node_a, node_b;
	segment lane = segment(goal.x + depth, goal.y + 10, goal.x + depth, goal.y - 10);
	segment up_edge = segment(goal.x + depth, goal.y + 5, goal.x + depth, goal.y + 10);
	segment down_edge = segment(goal.x + depth, goal.y - 5, goal.x + depth, goal.y - 10);
	int restric = 0;
	for (angle = PI / 6; angle >= (-PI / 6); angle -= (PI / 90))
	{
		q = goal;
		int stright_count = 0;
		int turn_count = 0;
		int leaf = 0;
		polygon car_body = drawCarBody(vector(q.x, q.y), q.cita);
		while (q.x <= (goal.x + depth) || !(car_body.intersection(lane)).empty())
		{
			if (!(car_body.intersection(up_edge)).empty() || !(car_body.intersection(down_edge)).empty())
			{
				stright_count++;
				turn_count = 0;
				q = goal;
				for (int i = 0; i < stright_count; i++)
				{
					u.fi = 0;
					u.v = 1;
					q = New_conf(q, u);
				}
				car_body = drawCarBody(vector(q.x, q.y), q.cita);
			}
			else
			{
				turn_count++;
				u.fi = angle;
				u.v = 1;
				q = New_conf(q, u);
				car_body = drawCarBody(vector(q.x, q.y), q.cita);
			}
		}
		car_q temp_q = goal;
		node_a = root_node;
		for (int i = 0; i < stright_count; i++)
		{
			u.fi = 0;
			u.v = 1;
			cv::Point pta = cv::Point(temp_q.x * 5, temp_q.y * 5);
			temp_q = New_conf(temp_q, u);
			cv::Point ptb = cv::Point(temp_q.x * 5, temp_q.y * 5);
			node_b = G.new_node(temp_q);
			G.new_edge(node_a, node_b, u);
			cv::line(picture, pta, ptb, col_white);
			node_a = node_b;
		}
		for (int i = 0; i < turn_count; i++)
		{
			u.fi = angle;
			u.v = 1;
			cv::Point pta = cv::Point(temp_q.x * 5, temp_q.y * 5);
			temp_q = New_conf(temp_q, u);
			cv::Point ptb = cv::Point(temp_q.x * 5, temp_q.y * 5);
			node_b = G.new_node(temp_q);
			G.new_edge(node_a, node_b, u);
			cv::line(picture, pta, ptb, col_white);
			node_a = node_b;
		}
		car_body = drawCarBody(vector(temp_q.x, temp_q.y), temp_q.cita);
		while (leaf < 20 && !outBox(car_body))
		{
			if (temp_q.cita > (PI / 2) || temp_q.cita < (-PI / 2))
				u.fi = 0;
			else
				u.fi = angle;
			u.v = 1;
			cv::Point pta = cv::Point(temp_q.x * 5, temp_q.y * 5);
			temp_q = New_conf(temp_q, u);
			cv::Point ptb = cv::Point(temp_q.x * 5, temp_q.y * 5);
			node_b = G.new_node(temp_q);
			G.new_edge(node_a, node_b, u);
			cv::line(picture, pta, ptb, col_white);
			node_a = node_b;
			if (leaf % 2 == 0)
			{
				search_list.push(temp_q);
			}
			leaf++;
			car_body = drawCarBody(vector(temp_q.x, temp_q.y), temp_q.cita);
			if (leaf == 20)
				restric++;
		}
	}
	std::cout << restric << "path are restricted by 20" << std::endl;
	return G;
}
