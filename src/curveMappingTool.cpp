#include "curveMappingTool.h"
#include "plane.h"

void CurveMapping::init()
{
	thrAngle = 10;
	thrCosin = cos(10 / 180 * pi);
}

bool CurveMapping::map(const BSpline &bsp1, const BSpline &bsp2, double threshold)
{
	vec3d p0 = bsp1.ctrlNodes[0];
	vec3d q0 = bsp2.ctrlNodes[0];
	shiftVec = q0 - p0;

}

void CurveMapping::getRotateAxis(const vec3d &u1, const vec3d &u2, const vec3d &v1, const vec3d &v2)
{
	vec3d n1 = u1 - v1;
	vec3d n2 = u2 - v2;
	n1.normalize();
	n2.normalize();
	if (abs(n1.dot(n2)) < 1 - thrCosin) 
	{
		rotateAxis = n1.cross(n2);
	}
	else
	{
		vec3d n3 = u1.cross(u2);
		vec3d n4 = v1.cross(v2);
		n3.normalize();
		n4.normalize();
		if (abs(n3.dot(n4)) < 1 - thrCosin)
		{
			rotateAxis = n3.cross(n4);
		}
		else
		{
			rotateAxis = u1 + v1 / 2;
		}
	}
	vec3d bisect1 = u1 + v1;
	vec3d bisect2 = u2 + v2;
	if (bisect1.length() == 0) theta = pi;
	else
	{
		bisect1.normalize();
		bisect2.normalize();
		double sinphi1 = bisect1.cross(u1).length();
		double cosphi1 = bisect1.dot(u1);
		double sinalpha1 = bisect1.cross(rotateAxis).length();
		double r1 = sqrt(sinphi1*sinphi1 + cosphi1*cosphi1 * sinalpha1*sinalpha1);
		double theta1 = 2 * asin(sinphi1 / r1);

		double sinphi2 = bisect2.cross(u2).length();
		double cosphi2 = bisect2.dot(u2);
		double sinalpha2 = bisect2.cross(rotateAxis).length();
		double r2 = sqrt(sinphi2*sinphi2 + cosphi2*cosphi2 * sinalpha2*sinalpha2);
		double theta2 = 2 * asin(sinphi2 / r2);

		theta = theta1;
		printf("theta1 = %lf, theta2 = %lf\n", theta1, theta2);
	}
	
	conv2mat();
	if (rotateVec(u1).dot(v1) < rotateVec(v1).dot(u1)) theta = -theta;
}

void CurveMapping::conv2mat()
{
	rotateAxis.normalize();
	double eye[3][3] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
	double trans[3][3] = {0, -rotateAxis.z, rotateAxis.y,
						  rotateAxis.z, 0, -rotateAxis.x,
						  -rotateAxis.y, rotateAxis.x, 0};
	for (int i = 0; i < 3; ++ i)
	{
		for (int j = 0; j < 3; ++ j)
		{
			rotateMat[i][j] = eye[i][j] * cos(theta);
			rotateMat[i][j] += (1 - cos(theta)) * rotateAxis._array[i] * rotateAxis._array[j];
			rotateMat[i][j] += trans[i][j] * sin(theta);
		}
	}
}

vec3d CurveMapping::rotateVec(const vec3d &u)
{
	vec3d v;
	for (int i = 0; i < 3; ++ i)
	{
		v._array[i] = 0;
		for (int j = 0; j < 3; ++ j)
		{
			v._array[i] += rotateMat[i][j] * u._array[j];
		}
	}
}