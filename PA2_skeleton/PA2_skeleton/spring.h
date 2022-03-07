
#pragma once
 
#include <iostream>
#include "Node.h"
using namespace std;

class mass_spring{
public:
	double		spring_coef;
	double		damping_coef;
	Node	*p1;
	Node	*p2;
	double		initial_length;
 
public:
 
	mass_spring(Node *p1, Node *p2)
	{
		damping_coef = 5.0;
		this->p1 = p1;
		this->p2 = p2;
		init();
	}

	void init()
	{
		vec3 S_length = (p2->position - p1->position);
		initial_length = S_length.length();
	}

	void internal_force(double dt)
	{
		//Basic Implements 2-1. Compute Spring Force
		/*add hook_force and damping force*/
		vec3 n1 = (p1->position - p2->position);
		n1.Normalize();
		vec3 n2 = (p2->position - p1->position);
		n2.Normalize();
																	//.length가 맞는지를 모르겠네;
		vec3 f1 = n1 * (spring_coef * (p1->position.dist(p2->position) - initial_length) + (damping_coef * (p1->velocity - p2->velocity)).dot(n1));
		vec3 f2 = n2 * (spring_coef * (p2->position.dist(p1->position) - initial_length) + (damping_coef * (p2->velocity - p1->velocity)).dot(n2));
		p1->add_force(f2);
		p2->add_force(f1);
	}
	void draw();

};