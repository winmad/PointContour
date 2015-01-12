#ifndef PLANE_H
#define PLANE_H

#include "smallUtils.h"
#include <map>
#include <utility>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

class Plane
{
public:
	Plane(double weight = 1) {this->weight = weight;}
	Plane(const vec3d& p, const vec3d& n, double weight = 1);
	Plane(const double& x , const double& y , const double&z ,
		const double& d , double weight = 1);
	Plane(const double& cosTheta , const double& cosPhi , const double& d);

	void normalize();
	void init(const vec3d& p, const vec3d& n, double weight);
	void init(const double& x , const double& y , const double& z ,
		const double& d , double weight);

	double dist(vec3d& point);
	double dist(Plane& plane);

	void add(Plane &plane);
	void remove(Plane &plane);
	vec3d reflect(const vec3d &pos) const;
	double calcValue(const vec3d& pos) const;
	void calcCosAngle(double& cosTheta , double& cosPhi);
	void calcAngle(double& theta , double& phi);
	vec3d intersect(vec3d& start , vec3d& dir);
	void fitFromPoints(Path& path);
		double posDist = std::abs(d - plane.d);
			plane.n = -plane.n;
			plane.d = -plane.d;
		d = d * weight + plane.d * plane.weight;
			plane.n = -plane.n;
			plane.d = -plane.d;
		d = d * weight - plane.d * plane.weight;
	void calculateAngle()
	{
		if (d > 0)
		{
			n = -n;
			d = -d;
		}
		r = std::abs(d);
		phi = acos(n.z);
		theta = acos(n.y / sin(phi));
	}

    vec3d p;
	vec3d n;
	double d;
	double weight;
	double phi, theta, r;
};

#endif //PLANE_H