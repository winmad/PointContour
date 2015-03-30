#ifndef AXIS_WIDGET_H
#define AXIS_WIDGET_H

#include "nvVector.h"
#include "smallUtils.h"
#include "splineUtils.h"
#include "LocalFrame.h"

class AxisWidget
{
public:
    void init();
    void resamplePoints();

    vec3d origin;
    double length , radius;

    int tableSize;
    std::vector<double> cosTable;
    std::vector<double> sinTable;

    std::vector<vec3d> axes;
    std::vector<Path> axesPoints;
    std::vector<Path> globePoints;
};

#endif