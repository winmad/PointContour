#ifndef SPLINE_UTILS_H
#define SPLINE_UTILS_H

#include "smallUtils.h"

class BSpline
{
public:
    int K , N;
    std::vector<double> knots;
    std::vector<vec3d> ctrlNodes;
    std::vector<double> t; // range [0,1]
    std::vector<std::vector<double> > coefs;

    BSpline()
    {
    }

    BSpline(const BSpline& bsp)
    {
        /* printf("before copy construct\n"); */
        K = bsp.K;
        N = bsp.N;
        knots = bsp.knots;
        ctrlNodes = bsp.ctrlNodes;
        t = bsp.t;
        coefs = bsp.coefs;
        /* printf("after copy construct\n"); */
    }

    BSpline& operator=(const BSpline& bsp)
    {
        /* printf("before operator =\n"); */
        if (this != &bsp)
        {
            this->clear();
            K = bsp.K;
            N = bsp.N;
            knots = bsp.knots;
            ctrlNodes = bsp.ctrlNodes;
            t = bsp.t;
            coefs = bsp.coefs;
        }
        /* printf("after operator =\n"); */
        return *this;
    }

    void clear()
    {
        knots.clear();
        ctrlNodes.clear();
        t.clear();
		coefs.clear();
    }

    void newCoefs()
    {
        coefs.clear();
		coefs.resize(N);
        for (int i = 0; i < N; i++) coefs[i].resize(K);
    }

	void copyBSP(BSpline &bsp)
	{
		K = bsp.K;
		N = bsp.N;
		knots = bsp.knots;
		ctrlNodes = bsp.ctrlNodes;
        t = bsp.t;
		coefs = bsp.coefs;
	}

    void calcCoefs();
    int getCtrlNodeIndex(const vec3d& pos);
    void updateBSpline(int index , const vec3d& newPos);
};

void convert2Spline(Path& path , BSpline& bsp);
void resampleBsp(BSpline& bsp , Path& path);
void resampleBspUniform(BSpline& bsp , int numPoints , Path& path);
void resampleLine(const vec3d& st , const vec3d& ed , int numPoints , Path& path);
void convert2Line(Path& path , BSpline& bsp);

double pathLength(Path& path);

#endif
