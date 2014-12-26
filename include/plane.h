#ifndef PLANE_H
#define PLANE_H

#include "smallUtils.h"
#include <map>
#include <utility>
#include <vector>
#include <cmath>

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
    Plane(const double& x , const double& y , const double&z ,
        const double& d , double weight = 1)
    {
        init(x , y , z , d , weight);
    }
    Plane(const double& cosTheta , const double& cosPhi , const double& d)
    {
        vec3d norm;
        norm.z = cosTheta;
        double sinTheta = std::sqrt(1.0 - cosTheta * cosTheta);
        norm.x = cosPhi * sinTheta;
        norm.y = std::sqrt(1.0 - cosPhi * cosPhi) * sinTheta;
        init(norm.x , norm.y , norm.z , d , 1);
    }
	void normalize()
	{
		p /= weight;
		n /= weight;
		n.normalize();
		d = -p.dot(n);
	}
    void init(const double& x , const double& y , const double&z ,
        const double& d , double weight)
    {
        this->n = vec3d(x , y , z);
        this->d = d;
        if (std::abs(x) > 1e-4)
        {
            this->p = vec3d(-d / x , 0 , 0);
        }
        else if (std::abs(y) > 1e-4)
        {
            this->p = vec3d(0 , -d / y , 0);
        }
        else if (std::abs(z) > 1e-4)
        {
            this->p = vec3d(0 , 0 , -d / z);
        }
        this->n.normalize();
        this->weight = weight;
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
		norm = (n + norm) * 0.5;
		double angleDist = 1 - std::abs(n.dot(plane.n));
		double posDist = std::abs((p - plane.p).dot(norm));

        printf("angleDist = %.6f , posDist = %.6f\n" ,
            angleDist , posDist);
        printf("p = (%.6f,%.6f,%.6f), n = (%.6f,%.6f,%.6f) , w = %.6f\n" ,
            p.x , p.y , p.z , n.x , n.y , n.z , weight);

		if (angleDist > 0.1) return 1e10;
		return posDist;
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
	vec3d reflect(const vec3d &pos) const
	{
		return pos + 2 * (-d - pos.dot(n)) * n;
	}
    double calcValue(const vec3d& pos) const
    {
        return pos.dot(n) + d;
    }
    void calcCosAngle(double& cosTheta , double& cosPhi)
    {
        cosTheta = n.z;
        vec3d v(n.x , n.y , 0);
        v.normalize();
        if (std::abs(1.0 - n.z) > 1e-4)
        {
            cosPhi = n.x;
        }
        else
        {
            cosPhi = 1.0;
        }
    }
    void calcAngle(double& theta , double& phi)
    {
        double cosTheta , cosPhi;
        calcCosAngle(cosTheta , cosPhi);
        theta = std::acos(cosTheta);
        phi = std::acos(cosPhi);
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