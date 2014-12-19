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
        this->p = p;
		this->n = n;
		this->n.normalize();
		d = -p.dot(n);
		this->weight = weight;
	}
	void normalize()
	{
		p /= weight;
		n /= weight;
		n.normalize();
		d = -p.dot(n);
	}
	double dist(Plane &plane)
	{
		vec3d norm = plane.n;
		if (n.dot(plane.n) < 0)
		{
			norm = -plane.n;
			//plane.n = -plane.n;
			//plane.d = -plane.d;
		}
		//return sqrt(SQR((n - p.n).length()) + SQR(d - p.d));
		if (1 - std::abs(n.dot(plane.n)) > 0.05) return 1e10;
		norm = (n + norm) * 0.5;
		return std::abs((p - plane.p).dot(norm));
	}
	void add(Plane &plane)
	{
		vec3d norm = plane.n;
		if (n.dot(plane.n) < 0)
		{
			norm = -plane.n;
		}
		p = p * weight + plane.p * plane.weight;
		n = n * weight + norm * plane.weight;
		weight += plane.weight;
		normalize();
	}
	void remove(Plane &plane)
	{
		vec3d norm = plane.n;
		if (n.dot(plane.n) < 0)
		{
			norm = -plane.n;
		}
		p = p * weight - plane.p * plane.weight;
		n = n * weight - norm * plane.weight;
		weight -= plane.weight;
		normalize();
	}
	vec3d reflect(vec3d &pos)
	{
		return pos + 2 * (-d - pos.dot(n)) * n;
	}
    vec3d intersect(vec3d& start , vec3d& dir)
    {
        double denom = n.dot(dir);
        double numer = -n.dot(start) - d;
        if (std::abs(denom) < 1e-6) printf("intersect denom = 0\n");
        return start + dir * (numer / denom);
    }

    vec3d p;
	vec3d n;
	double d;
	double weight;
};

#endif //PLANE_H