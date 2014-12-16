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
    double **coefs;

    BSpline()
    {
        coefs = NULL;
    }

    ~BSpline()
    {
        clear();
    }

    BSpline(const BSpline& bsp)
    {
        /* printf("before copy construct\n"); */
        if (coefs != NULL)
        {
            coefs = NULL;
        }
        K = bsp.K;
        N = bsp.N;
        knots = bsp.knots;
        ctrlNodes = bsp.ctrlNodes;
        t = bsp.t;
        if (bsp.coefs != NULL)
        {
            newCoefs();
            for (int i = 0; i < N; i++)
            {
                for (int j = 0; j < K; j++) coefs[i][j] = bsp.coefs[i][j];
            }
        }
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
            if (bsp.coefs != NULL)
            {
                newCoefs();
                for (int i = 0; i < N; i++)
                {
                    for (int j = 0; j < K; j++) coefs[i][j] = bsp.coefs[i][j];
                }
            }
        }
        /* printf("after operator =\n"); */
        return *this;
    }

    void clear()
    {
        if (coefs != NULL)
        {
            for (int i = 0; i < N; i++) delete[] coefs[i];
            delete[] coefs;
            coefs = NULL;
        }
        knots.clear();
        ctrlNodes.clear();
        t.clear();
    }

    void newCoefs()
    {
        if (coefs != NULL)
        {
            for (int i = 0; i < N; i++) delete[] coefs[i];
            delete[] coefs;
            coefs = NULL;
        }
        coefs = new double*[N];
        for (int i = 0; i < N; i++) coefs[i] = new double[K];
    }

	void copyBSP(BSpline &bsp)
	{
		K = bsp.K;
		N = bsp.N;
		knots = bsp.knots;
		ctrlNodes = bsp.ctrlNodes;
        t = bsp.t;
		newCoefs();
		for (int i = 0; i < N; ++ i)
			for (int j = 0; j < K; ++ j)
				coefs[i][j] = bsp.coefs[i][j];
	}

    void calcCoefs();
    int getCtrlNodeIndex(const vec3d& pos);
    void updateBSpline(int index , const vec3d& newPos);
};

void convert2Spline(Path& path , BSpline& bsp);
void resampleBsp(BSpline& bsp , Path& path);
void convert2Line(Path& path , BSpline& bsp);

#endif
