#ifndef SPLINE_UTILS_H
#define SPLINE_UTILS_H

#include "smallUtils.h"

struct BSpline
{
    std::vector<double> knots;
    std::vector<vec3d> ctrlNodes;

    void clear()
    {
        knots.clear();
        ctrlNodes.clear();
    }
};

void convert2Spline(Path& path , BSpline& bsp);
void resampleBsp(BSpline& bsp , Path& path);

#endif
