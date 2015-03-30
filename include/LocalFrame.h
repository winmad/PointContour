#pragma once
#include "nvVector.h"
#include <cmath>

struct LocalFrame
{
	vec3d s;
	vec3d t;
	vec3d n;

	void buildFromNormal(const vec3d& normal)
	{
		n = normal;
		vec3d tmpT;
		tmpT = (std::abs(n.z) > 0.99f) ? vec3d(1,0,0) : vec3d(0,0,1);
		s = n.cross(tmpT);
		s.normalize();
		t = s.cross(n);
		t.normalize();
	}

	vec3d toWorld(const vec3d& a) const
	{
		return s * a.x + n * a.y + t * a.z;
	}

	vec3d toLocal(const vec3d &a) const
	{
		return vec3d(a.dot(s) , a.dot(n) , a.dot(t));
	}
};



//	   (n)
//		A	
//		|
//	  /-|----/
//	 /	|---->(s)
//	/--/---/
//	  /	
//	 v
//	(t)	