#ifndef CURVE_MAPPING_TOOL_H
#define CURVE_MAPPING_TOOL_H

#include "nvVector.h"
#include "splineUtils.h"

const double pi = 3.1415926;

class CurveMapping
{
public:
	void init();
	bool map(const BSpline &bsp1, const BSpline &bsp2, double threshold);
	void getRotateAxis(const vec3d &u1, const vec3d &u2, const vec3d &v1, const vec3d &v2);
	void conv2mat();
	vec3d rotateVec(const vec3d &u);

	double thrAngle;
	double thrCosin;
	double thrRotate;

	vec3d rotateAxis;
	double theta;
	vec3d shiftVec;
	double rotateMat[3][3];
}

#endif //CURVE_MAPPING_TOOL_H