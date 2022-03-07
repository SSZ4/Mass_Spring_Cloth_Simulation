
#pragma once

#include "spring.h"
#include "Node.h"
#include <stdio.h>
#include <math.h>
#include <vector>
#include <algorithm>
#include <fstream>

class mass_cloth
{
public:

	std::vector<Node *> nodes;
	std::vector<mass_spring *> spring;
	std::vector<Node *> faces;

	int			size_x, size_y, size_z;
	double		dx, dy, dz;
	double		structural_coef;
	double		shear_coef;
	double		bending_coef;
	int			iteration_n;
	int			drawMode;
	

	mass_cloth()
	{ 	 
	}
	~mass_cloth()
	{ 
		for (int i = 0; i < nodes.size(); i++){ delete nodes[i]; }
		for (int i = 0; i < spring.size(); i++){ delete spring[i]; }
		nodes.clear();
		spring.clear();
		faces.clear();
	}
	enum DrawModeEnum{
		DRAW_MASS_NODES,
		DRAW_SPRINGS,
		DRAW_FACES
	};
 
public:
	void init()
	{
		//Basic Implements 1. Init Nodes and Shear and Structural Springs
		//Additional Implements 1. Init Bending Spring
		
		nodes.push_back(new Node(vec3(1110,1110,1110)));//0번 인덱스 채우는용
		//노드는 문제없다
		for (int y = size_y; y >= 1; y--) {//1~50짜리 50개 생성
			for (int x = size_x; x >= 1; x--) {
				Node* xp = new Node(vec3(x-20, 20, y));

				xp->inipos = vec3(size_x - (x + 1), 30, size_y - (y + 1)) / 49;// +vec3(1.0, 0.0, 1.0);
				nodes.push_back(xp);
			}
		}
		//for (int i = 1; i <= 50; i++) //맨 윗줄 고정.
			nodes[1]->isFixed = true;
			nodes[50]->isFixed = true;
		
		for (int y = 0; y <= size_y-1; y++) {
			for (int x = 1; x <= size_x; x++) {
				if (x != size_x) {
					mass_spring* sp = new mass_spring(nodes[x + size_x * y], nodes[x + size_x * y + 1]);
					sp->spring_coef = structural_coef;
					spring.push_back(sp);
				}
				if (y != size_y-1) {
					mass_spring* sp = new mass_spring(nodes[x + size_x * y], nodes[x + size_x * (y+1)]);
					sp->spring_coef = structural_coef;
					spring.push_back(sp);
				}
			}
		}
		
		for (int y = 0; y < size_y-1; y++) {
			for (int x = 1; x <= size_x-1; x++) {				
				mass_spring* sp = new mass_spring(nodes[x + size_x * y], nodes[x + 1 + size_x * (y + 1)]);
				sp->spring_coef = shear_coef;
				spring.push_back(sp);

				sp = new mass_spring(nodes[x + 1 + size_x * y], nodes[x + size_x * (y + 1)]);
				sp->spring_coef = shear_coef;
				spring.push_back(sp);
			}
		}

		for (int y = 0; y < size_y; y++) {
			for (int x = 1; x <= size_x; x++) {
				if (x <= size_x-2) {
					mass_spring* sp = new mass_spring(nodes[x + size_x * y], nodes[x + size_x * y + 2]);
					sp->spring_coef = bending_coef;
					spring.push_back(sp);
				}
				if (y <= size_y - 3) {
					mass_spring* sp = new mass_spring(nodes[x + size_x * y], nodes[x + size_x * (y + 2)]);
					sp->spring_coef = bending_coef;
					spring.push_back(sp);
				}
			}
		}

		
		//Basic Implements 3-1. Generate Faces
		for (int y = 0; y <= size_y-2; y++) {
			for (int x = 1; x <= size_x-1; x++) {
				faces.push_back(nodes[x + size_x * y]);
				faces.push_back(nodes[x + size_x * (y+1)]);
				faces.push_back(nodes[(x+1) + size_x * (y+1)]);

				faces.push_back(nodes[x + size_x * y]);
				faces.push_back(nodes[(x+1) + size_x * (y + 1)]);
				faces.push_back(nodes[(x + 1) + size_x * y]);
			}
		}
		
		
		
		GLuint Texture;
		Texture = LoadTexture("t1logo.bmp");
		//Additional Implements 4-2. Initialize Texture Coordinates	
		


	}
	
	void computeNormal()
	{	
		for (int i = 0; i < nodes.size(); i++) {
			nodes[i]->normal = vec3(0, 0, 0);
		}
		//Basic Implements 3-2. Compute Vertex Normal
		for (int i = 0; i < faces.size(); i += 3) {
			vec3 v1 = faces[i+1]->position - faces[i]->position;
			vec3 v2 = faces[i+2]->position - faces[i]->position;
			vec3 facenormal = v1.Cross(v2);
			facenormal.Normalize();

			faces[i]->normal += facenormal;
			faces[i+1]->normal += facenormal;
			faces[i+2]->normal += facenormal;			
		}

		for (int i = 0; i < nodes.size(); i++) {
			nodes[i]->normal.Normalize();
		}
	}
	
	void add_force(vec3 additional_force)
	{		 
		for (int i = 0; i < nodes.size(); i++)
		{
			nodes[i]->add_force(additional_force);
		}
	}

	void compute_force(double dt, vec3 gravity)
	{
		for (int i = 0; i < nodes.size(); i++)
		{
			nodes[i]->add_force(gravity * nodes[i]->mass);
		}
		/* Compute Force for all springs */
		for (int i = 0; i < spring.size(); i++)
		{
			spring[i]->internal_force(dt);
		}
	}

	void integrate(double dt)
	{
		/* integrate Nodes*/
		for (int i = 0; i < nodes.size(); i++)
		{
			nodes[i]->integrate(dt);
		}
	}
	
	void collision_response(vec3 ground)
	{
		//Basic Implements 4. Collision Check with ground
		vec3 gnormal = vec3(0.0f, 1.0f, 0.0f);
		gnormal.Normalize();

		for (int i = 1; i < nodes.size(); i++) {
			if ((nodes[i]->position - ground).dot(gnormal) < 1
				&&gnormal.dot(nodes[i]->velocity) < 0) {			
				
				vec3 vn = gnormal.dot(nodes[i]->velocity) * gnormal;
				vec3 vt = nodes[i]->velocity - vn;
				vec3 result = vt - vn;
				nodes[i]->velocity = result;
				
			}
		}		
	}

	void collision_sphere(vec3 sphere, float radius) {
		

		for (int i = 1; i < nodes.size(); i++) {
			vec3 gnormal = nodes[i]->position - sphere;
			gnormal.Normalize();

			if (nodes[i]->position.dist(sphere) < radius
				&& (gnormal.dot(nodes[i]->velocity) < 0)) {			

				vec3 vn = gnormal.dot(nodes[i]->velocity) * gnormal;
				vec3 vt = nodes[i]->velocity - vn;
				vec3 result = vt - vn;
				nodes[i]->velocity = result;
			}
		}
	}

	GLuint LoadTexture(const char* filename)
	{

		GLuint texture;
		int width, height;
		unsigned char* data;
		FILE* file;
		file = fopen(filename, "rb");

		if (file == NULL) return 0;
		width = 512;
		height = 512;
		data = (unsigned char*)malloc(width * height * 3);
		fread(data, 54, 1, file);		//int size = fseek(file,);

		fread(data, width * height * 3, 1, file);
		fclose(file);
		for (int i = 0; i < width * height; ++i)
		{
			int index = i * 3;
			unsigned char B, R;
			B = data[index];
			R = data[index + 2];

			data[index] = R;
			data[index + 2] = B;

		}
		glGenTextures(1, &texture);
		glBindTexture(GL_TEXTURE_2D, texture);
		glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);


		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		gluBuild2DMipmaps(GL_TEXTURE_2D, 3, width, height, GL_RGB, GL_UNSIGNED_BYTE, data);
		free(data);

		return texture;
	}


	void draw();
};


