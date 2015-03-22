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

    vec3d origin;
    double length;
    std::vector<vec3d> axes;
    std::vector<Path> axesPoints;
};

#endif