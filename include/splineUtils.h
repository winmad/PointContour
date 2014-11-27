#ifndef SPLINE_UTILS_H
#define SPLINE_UTILS_H

#include "smallUtils.h"

class BSpline
{
public:
    int K , N;
    std::vector<double> knots;
    std::vector<vec3d> ctrlNodes;
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
		newCoefs();
		for (int i = 0; i < N; ++ i)
			for (int j = 0; j < K; ++ j)
				coefs[i][j] = bsp.coefs[i][j];
	}

    void calcCoefs();
};

void convert2Spline(Path& path , BSpline& bsp);
void resampleBsp(BSpline& bsp , Path& path);

#endif
