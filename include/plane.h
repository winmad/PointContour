#ifndef PLANE_H
#define PLANE_H

#include "smallUtils.h"
#include <map>
#include <utility>
#include <vector>

class Plane
{
public:
	Plane(double weight = 1) {this->weight = weight;}
	Plane(const vec3d& p, const vec3d& n, double weight = 1)
	{
		this->n = n;
		d = -p.dot(n);
		this->weight = weight;
		normalize();
	}
	void normalize()
	{
		d /= n.length();
		n.normalize();
	}
	double dist(Plane &p)
	{
		if (n.dot(p.n) < 0)
		{
			p.n = -p.n;
			p.d = -p.d;
		}
		return sqrt(SQR((n - p.n).length()) + SQR(d - p.d));
	}
	void add(Plane &p)
	{
		if (n.dot(p.n) < 0)
		{
			p.n = -p.n;
			p.d = -p.d;
		}
		n = n * weight + p.n * p.weight;
		d = d * weight + p.d * p.weight;
		weight += p.weight;
		normalize();
	}
	void remove(Plane &p)
	{
		if (n.dot(p.n) < 0)
		{
			p.n = -p.n;
			p.d = -p.d;
		}
		n = n * weight - p.n * p.weight;
		d = d * weight - p.d * p.weight;
		weight -= p.weight;
		normalize();
	}
	vec3d reflect(vec3d &p)
	{
		return p + 2 * (-d - p.dot(n)) * n;
	}

	vec3d n;
	double d;
	double weight;
};

#endif //PLANE_H