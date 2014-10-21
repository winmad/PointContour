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

struct BSpline
{
    std::vector<double> knots;
    std::vector<vec3d> ctrlNodes;
    /*
    BSpline& operator =(const BSpline& bsp)
    {
        knots.clear();
        ctrlNodes.clear();
        for (int i = 0; i < bsp.knots.size(); i++)
        {
            knots.push_back(bsp.knots[i]);
        }
        for (int i = 0; i < bsp.ctrlNodes.size(); i++)
        {
            ctrlNodes.push_back(bsp.ctrlNodes[i]);
        }
        return this;
    }
    */
    void clear()
    {
        knots.clear();
        ctrlNodes.clear();
    }
};

const double pow2[3] = {1.0 , 1024.0 , 1024.0 * 1024.0};

static char logFileName[128];

extern Engine *ep;
void convert2Spline(Path& path , BSpline& bsp);

bool restartLog(std::string fileName);
bool writeLog(const char* msg , ...);

double point2double(const vec3d& p);

bool isEqual(const vec3d& lhs , const vec3d& rhs);

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