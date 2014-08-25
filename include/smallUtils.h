#ifndef SMALL_UTILS_H
#define SMALL_UTILS_H

#include "nvVector.h"
#include <iostream>
#include <vector>
#include <string>
#include <cstdarg>

#define PI 3.14159265358979
#define SQR(x) ((x) * (x))

struct Edge;
typedef std::vector<vec3f> Path;
typedef std::vector<std::vector<Edge> > Graph;

const double pow2[3] = {1.0 , 1024.0 , 1024.0 * 1024.0};

static char logFileName[128];

bool restartLog(std::string fileName);
bool writeLog(const char* msg , ...);

double point2double(const vec3f& p);

template<typename T>
bool allocate3(T*** &d , const vec3i& size)
{
	if (d != NULL)
		throw("Not initialized pointer!");
	d = new T**[size.x];
	for (int i = 0; i < size.x; i++)
	{
		d[i] = new T*[size.y];
		for (int j = 0; j < size.y; j++)
		{
			d[i][j] = new T[size.z];
		}
	}
	return true;
}

template<typename T>
bool deallocate3(T*** &d , const vec3i& size)
{
	if (d == NULL)
		return true;
	for (int i = 0; i < size.x; i++)
	{
		for (int j = 0; j < size.y; j++)
		{
			delete[] d[i][j];
		}
		delete[] d[i];
	}
	delete[] d;
	d = NULL;
	return true;
}

void writeBMP(const char* filename , int w , int h ,
    unsigned char* rgb);

#endif