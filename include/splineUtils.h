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

    void calcCoefs();
};

void convert2Spline(Path& path , BSpline& bsp);
void resampleBsp(BSpline& bsp , Path& path);

#endif
