#ifndef SMALL_UTILS_H
#define SMALL_UTILS_H

#include "nvVector.h"
#include <iostream>
#include <vector>
#include <string>
#include <cstdarg>
#include <algorithm>
#include "engine.h"
#define PI 3.14159265358979
#define EPS 1e-8
#define SQR(x) ((x) * (x))

struct Edge;
typedef std::vector<vec3d> Path;
typedef std::vector<std::vector<Edge> > Graph;
typedef std::vector<unsigned> Cycle;
typedef std::vector<Cycle> CycleGroup;

const double pow2[3] = {1.0 , 100.0 , 100.0 * 100.0};

static char logFileName[128];

extern Engine *ep;

bool restartLog(std::string fileName);
bool writeLog(const char* msg , ...);

double point2double(const vec3d& p);

bool isEqual(const vec3d& lhs , const vec3d& rhs);
bool isValid(const vec3d& pos);
void setNull(vec3d& pos);
bool isSameCycle(std::vector<unsigned>& c1 , std::vector<unsigned>& c2);
bool isSameCycleGroup(CycleGroup& c1 , CycleGroup& c2);

vec3d pointScaling(const vec3d& pos , const vec3d& origin , int chosenAxis , double scale);

double weightBetweenSegs(const vec3d& x1 , const vec3d& y1 , 
	const vec3d& x2 , const vec3d& y2);

template<typename T>
T clampValue(const T& x , const T& minval , const T& maxval)
{
    return std::max(minval , std::min(maxval , x));
}

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