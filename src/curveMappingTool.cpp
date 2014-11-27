#include "curveMappingTool.h"
#include "plane.h"

void CurveMapping::init()
{
	thrAngle = 10;
	thrCosin = cos(10 / 180 * pi);
	thrScale = 1e-2;
	curvatureK = 1;
	thrDiff = 10 / 180 * pi;
}

bool CurveMapping::map(const Path &line1, const Path &line2, double threshold)
{
	if ((line1[line1.size() - 1] - line1[0]).length() < 1e-6)
	{
		if ((line2[line2.size() - 1] - line2[0]).length() < 1e-6)
		{
			int idx = matchCycle(line1, line2);
			if (idx == -1) return false;
		}
		else
			return false;
	}
	else
	{
		if ((line2[line2.size() - 1] - line2[0]).length() < 1e-6) return false;
	}
	double length1 = 0;
	double length2 = 0;
	int segment = 10;
	int step = (line1.size() - 1) / segment;
	for (int i = 1; i < line1.size(); i += step)
	{
		length1 += (line1[i] - line1[i-1]).length();
		length2 += (line2[i] - line2[i-1]).length();
		vec3d t1 = line1[i] - line1[0];
		vec3d t2 = line2[i] - line2[0];
		if (abs(t1.length() / length1 - t2.length() / length2) > thrScale)
		{
			return false;
		}
	}
	vec3d p0 = line1[0];
	vec3d q0 = line2[0];
	shiftVec = q0 - p0;
	return false;
}

bool CurveMapping::matchCycle(const Path &c1, const Path &c2)
{
	std::vector<int> idx1, idx2;
	std::vector<double> curvature1, curvature2;
	for (int i = 0; i < c1.size(); ++ i)
	{
		curvature1.push_back(calc(c1, i, curvatureK));
		curvature2.push_back(calc(c2, i, curvatureK));
	}
	//getExtremeCurv(curvature1, idx1);
	//getExtremeCurv(curvature2, idx2);
	//if (abs((int)idx1.size() - (int)idx2.size()) > 0) return false;
	if (idx1.size() > 0)
	{
		for (int i = 0; i < idx1.size(); ++ i)
		{
			for (int j = 0; j < idx2.size(); ++ j)
			{
				for (int k = 0; k < idx1.size(); ++ k)
				{

				}
			}
		}
	}
}

void CurveMapping::getExtremeCurv(const std::vector<double> &c, std::vector<int> &idx)
{
	int maxIdx = 0, minIdx = 0, incIdx = -1, decIdx = -1;
	int l = c.size();
	double last = c[0];
	double maxc = last, minc = last;
	for (int i = 1; i < l * 2; ++ i)
	{
		int t = i % l;
		double tmp = c[t];
		if (tmp - last > thrDiff && decIdx > incIdx)
		{
			idx.push_back(minIdx);
			incIdx = i;
			minc = pi;
			maxc = tmp;
			maxIdx = t;
			if (i > l) break;
			continue;
		}
		if (last - tmp > thrDiff && incIdx > decIdx)
		{
			idx.push_back(maxIdx);
			decIdx = i;
			maxc = 0;
			minc = tmp;
			minIdx = t;
			if (i > l) break;
			continue;
		}
		if (tmp > maxc && incIdx > decIdx)
		{
			maxc = tmp;
			maxIdx = t;
			continue;
		}
		if (tmp < minc && decIdx > incIdx)
		{
			minc = tmp;
			minIdx = t;
			continue;
		}
	}
	if (idx.back() == idx.front()) idx.pop_back();
}

double CurveMapping::calc(const Path &c, int idx, int k)
{
	int l = c.size();
	double ans = 0;
	for (int i = 0; i < k; ++ i)
	{
		vec3d v1 = c[(idx - k - 1 + l) % l] - c[idx];
		vec3d v2 = c[idx] - c[(idx + k + 1) % l];
		v1.normalize();
		v2.normalize();
		ans += (v1).dot(v2);
	}
	ans /= k;
	return acos(ans);
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
	return v;
}