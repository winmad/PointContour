#include "constraints.h"
#include "curveNet.h"
#include <Eigen/Dense>

const double ConstraintDetector::collinearThr = 0.05;
const double ConstraintDetector::coplanarThr = 0.03;
const double ConstraintDetector::parallelThr = 0.05;
const double ConstraintDetector::orthoThr = 0.1;
const double ConstraintDetector::tangentThr = 0.05;
const double ConstraintDetector::symmetryThr = 0.1;
const double ConstraintDetector::ratioThr = 0.05;
const double ConstraintDetector::planeDiffThr = 0.1;

bool ConstraintDetector::collinearTest(Path& path , BSpline& bsp)
{
    if (path.size() <= 2)
    {
        bsp.clear();
        for (int i = 0; i < 2; i++)
        {
            bsp.ctrlNodes.push_back(path[i]);
			bsp.t.push_back((double)i);
        }
        return true;
    }
    vec3d x1 = path[0] , x2 = path[path.size() - 1];
    double totDist = 0.0;
    vec3d v = x2 - x1;
    double denom = std::abs(v.length());
    for (int i = 1; i < (int)path.size() - 1; i++)
    {
		vec3d u1 = path[i] - x1;
		vec3d u2 = path[i] - x2;
		double len1 = u1.length();
		double len2 = u2.length();
		u1 /= len1; u2 /= len2;
        v = u1.cross(u2) * len1 * len2;
        double numer = std::abs(v.length());
        double d = numer / denom;
        //if (d > denom * 0.05) return false;
        totDist += d;
    }

    // printf("--- collinear test: %.6f op %.6f ---\n" , totDist / (double)path.size() , denom * 0.03);
    if (totDist / (double)path.size() < denom * 0.03)
    {
        bsp.clear();
        bsp.ctrlNodes.push_back(x1);
        bsp.ctrlNodes.push_back(x2);
        v = x2 - x1;
        double totLen = 0;
        bsp.t.push_back(0);
        for (int i = 1; i < path.size(); i++)
        {
            double len = (path[i] - path[i - 1]).length();
            totLen += len;
            bsp.t.push_back(totLen);
        }
        for (int i = 0; i < path.size(); i++) bsp.t[i] /= totLen;

        for (int i = 1; i < (int)path.size() - 1; i++)
        {
            path[i] = x1 + v * bsp.t[i];
        }
        return true;
    }
    return false;
}

bool ConstraintDetector::coplanarTest(BSpline& bsp , Plane& plane)
{
	double res = 0.0 , totWeight = 0.0;
    vec3d p(0.0) , n(0.0);
    Path path;
    resampleBspUniform(bsp , 40 , path);
    double pathLen = pathLength(path);
    //plane.weight = 0;
    /*
    p = vec3d(0.0);
    for (int i = 0; i < path.size(); i++)
    {
        p += path[i];
    }
    p /= path.size();

    plane = Plane(p , n , 0);
	for (int i = 0; i < (int)bsp.ctrlNodes.size() - 1; i++)
	{
		for (int j = i + 1; j < (int)bsp.ctrlNodes.size() - 1; j++)
		{
			vec3d x1 = bsp.ctrlNodes[i] , y1 = bsp.ctrlNodes[i + 1];
			vec3d x2 = bsp.ctrlNodes[j] , y2 = bsp.ctrlNodes[j + 1];
			if (checkParallel(x1 , y1 , x2 , y2 , parallelThr))
			{
				continue;
			}
            vec3d u1 = y1 - x1 , u2 = y2 - x2;
            u1.normalize(); u2.normalize();
			vec3d norm = u1.cross(u2);
			norm.normalize();
			//vec3d p = (x1 + y1 + x2 + y2) * 0.25;

			double score = 0.0 , weight = 1.0;
			// weight = (x1 - y1).length() * (x2 - y2).length();
            // * weightBetweenSegs(x1 , y1 , x2 , y2);
            Plane newPlane(p , norm , weight);
            // plane.add(newPlane);
            
			vec3d d = (x2 + y2) * 0.5 - p;
			score += std::abs(d.dot(norm));
			d = (x1 + y1) * 0.5 - p;
			score += std::abs(d.dot(norm));
			res += score * 0.5 * weight;
			totWeight += weight;
		}
	}
	res /= totWeight;
    printf("coplanar test score = %.8f\n" , res);
    */
    
    plane.fitFromPoints(path);
    
    for (int i = 0; i < path.size(); i++)
    {
        res += std::abs(plane.dist(path[i]));
    }
    printf("coplane norm = (%.6f,%.6f,%.6f)\n" , plane.n.x , plane.n.y , plane.n.z);
    printf("coplane score = %.6f\n" , res / path.size());
	if (res / path.size() < pathLen * 0.03) return true;
    plane.weight = 0; plane.p = vec3d(0.0); plane.n = vec3d(0.0);
	return false;
}

bool ConstraintDetector::checkParallel(const vec3d& x1 , const vec3d& y1 ,
                             const vec3d& x2 , const vec3d& y2 , const double& threshold)
{
    vec3d v1 , v2;
    v1 = y1 - x1;
    v1.normalize();
    v2 = y2 - x2;
    v2.normalize();
    if (1.0 - std::abs(v1.dot(v2)) > threshold) return false;
    return true;
}

bool ConstraintDetector::checkCollinear(const vec3d& x1 , const vec3d& y1 ,
                              const vec3d& x2 , const vec3d& y2 , const double& threshold)
{
    if (!checkParallel(x1 , y1 , x2 , y2 , threshold)) return false;
    
    vec3d v1 , v2;
    v1 = y1 - x1; v2 = x2 - x1;
    v1.normalize(); v2.normalize();
    double cosine = v1.dot(v2);
    if (1.0 - std::abs(cosine) > threshold) return false;
    
    //v1 = y1 - x1;
    v2 = y2 - x1;
    v2.normalize();
    cosine = v1.dot(v2);
    if (1.0 - std::abs(cosine) > threshold) return false;
    
    v1 = x2 - y1; v2 = y2 - y1;
    v1.normalize(); v2.normalize();
    cosine = v1.dot(v2);
    if (1.0 - std::abs(cosine) > threshold) return false;
    
    v1 = x2 - x1; v2 = y2 - x1;
    v1.normalize(); v2.normalize();
    cosine = v1.dot(v2);
    if (1.0 - std::abs(cosine) > threshold) return false;
    
	v1 = y1 - x1; v2 = y2 - x2;
	double len1 = v1.length() , len2 = v2.length();
	v1 /= len1; v2 /= len2;

    vec3d denom = v1.cross(v2) * len1 * len2;
    vec3d numer = (x2 - x1).dot(denom);
    double d = numer.length() / denom.length();
    // printf("dist = %.6f\n" , d);
    if (d > 0.01) return false;
    
    return true;
}

bool ConstraintDetector::checkCoplanar(BSpline& bsp1 , Plane& plane1 ,
    BSpline& bsp2 , Plane& plane2 , const double& threshold)
{
    int numPoints = 5;
    double res = 0.0;
    if (bsp1.ctrlNodes.size() > 2 && bsp2.ctrlNodes.size() > 2)
    {
        // printf("type 3 & type 3\n");
        if (plane1.dist(plane2) > threshold) return false;
    }
    else if (bsp1.ctrlNodes.size() > 2 && bsp2.ctrlNodes.size() == 2)
    {
        // printf("type 3 & type 1\n");
        Path path;
        resampleBspUniform(bsp2 , numPoints , path);
        for (int i = 0; i < path.size(); i++)
        {
            res += std::abs(plane1.dist(path[i]));
        }
        res /= (double)path.size();
        if (res > threshold) return false;
    }
    else if (bsp1.ctrlNodes.size() == 2 && bsp2.ctrlNodes.size() > 2)
    {
        // printf("type 1 & type 3\n");
        Path path;
        resampleBspUniform(bsp1 , numPoints , path);
        for (int i = 0; i < path.size(); i++)
        {
            res += std::abs(plane2.dist(path[i]));
        }
        res /= (double)path.size();
        if (res > threshold) return false;
    }
    else if (bsp1.ctrlNodes.size() == 2 && bsp2.ctrlNodes.size() == 2)
    {
        // printf("type 1 & type 1\n");
        vec3d x1 = bsp1.ctrlNodes[0] , y1 = bsp1.ctrlNodes[1];
        vec3d x2 = bsp2.ctrlNodes[0] , y2 = bsp2.ctrlNodes[1];
        if (checkParallel(x1 , y1 , x2 , y2 , parallelThr)) return true;
        vec3d v1 = y1 - x1 , v2 = y2 - x2;
        v1.normalize(); v2.normalize();
        vec3d n = v1.cross(v2);
        n.normalize();
        vec3d d = (x2 + y2) * 0.5 - (x1 + y1) * 0.5;
        res = std::abs(d.dot(n));
        if (res > threshold) return false;
    }
    return true;
}

/*
bool ConstraintDetector::checkCoplanar(const BSpline& bsp1 , const BSpline& bsp2 ,
                             const double& threshold)
{
    double sum = 0.0;
    double denom = 0.0;
	bool isParallel = true;
    for (int i = 0; i < (int)bsp1.ctrlNodes.size() - 1; i++)
    {
        for (int j = 0; j < (int)bsp2.ctrlNodes.size() - 1; j++)
        {
            vec3d x1 = bsp1.ctrlNodes[i] , y1 = bsp1.ctrlNodes[i + 1];
            vec3d x2 = bsp2.ctrlNodes[j] , y2 = bsp2.ctrlNodes[j + 1];
            if (checkParallel(x1 , y1 , x2 , y2 , parallelThr))
            {
                continue;
            }
			if (isParallel) isParallel = false;
			double weight = (y1 - x1).length() * (y2 - x2).length() *
				weightBetweenSegs(x1 , y1 , x2 , y2);
            vec3d v1 = y1 - x1 , v2 = y2 - x2;
            v1.normalize(); v2.normalize();
            vec3d n = v1.cross(v2);
            n.normalize();
            vec3d d = (x2 + y2) * 0.5 - (x1 + y1) * 0.5;
			double tp = std::abs(d.dot(n));
            sum += tp * weight;
			denom += weight;
        }
    }
	if (isParallel) return true;
    //printf("between coplanar = %.8f\n" , sum / denom);
    if (sum / denom < threshold) return true;
    return false;
}
*/
/*
bool ConstraintDetector::checkCoplanar(const vec3d& x1 , const vec3d& y1 ,
                             const vec3d& x2 , const vec3d& y2 , const double& threshold)
{
    if (checkParallel(x1 , y1 , x2 , y2 , parallelThr)) return true;
    vec3d n = (y1 - x1).cross(y2 - x2);
    n.normalize();
    vec3d d = (x2 + y2) * 0.5 - (x1 + y1) * 0.5;
    d.normalize();
    //printf("%.8f\n" , std::abs(d.dot(n)));
    if (std::abs(d.dot(n)) < threshold) return true;
    return false;
}
*/
bool ConstraintDetector::checkOrtho(const vec3d& x0 , const vec3d& x1 ,
                          const vec3d& x2 , const double& threshold)
{
    vec3d v1 = x1 - x0;
    v1.normalize();
    vec3d v2 = x2 - x0;
    v2.normalize();
    // printf("v1 = (%.6f,%.6f,%.6f) , v2 = (%.6f,%.6f,%.6f)\n" , v1.x , v1.y , v1.z ,
    // v2.x , v2.y , v2.z);
    if (std::abs(v1.dot(v2)) < threshold) return true;
    return false;
}

bool ConstraintDetector::checkTangent(const vec3d& x0 , const vec3d& x1 ,
                            const vec3d& x2 , const double& threshold)
{
    vec3d v1 = x1 - x0;
    v1.normalize();
    vec3d v2 = x2 - x0;
    v2.normalize();
    if (1.0 - std::abs(v1.dot(v2)) < threshold) return true;
    return false;
}

bool ConstraintDetector::checkSymmetry(const vec3d& x , const vec3d& nx ,
                             const vec3d& y , const vec3d& ny , const double& threshold)
{
    vec3d v0 = x - y;
    v0.normalize();
    vec3d v1 = nx + ny;
    if (abs(nx.length() - ny.length()) > ny.length() * ratioThr) return false;
    v1.normalize();
    
    vec3d v2 = nx - ny;
    v2.normalize();
    if (abs(v0.dot(v1)) < threshold) return true;
    //if (abs(v0.dot(v2)) < threshold) return true;
    return false;
}


ConstraintSet::ConstraintSet(CurveNet *curveNet)
{
    net = curveNet;
    clear();
}

void ConstraintSet::clear()
{
    collinearSet.clear();
    parallelSet.clear();
    coplanarSet.clear();
    orthoSet.clear();
	symmetricPlanes.clear();
	symmLines.clear();
}

void ConstraintSet::copyFrom(ConstraintSet* conSet)
{
    collinearSet = conSet->collinearSet;
    parallelSet = conSet->parallelSet;
    coplanarSet = conSet->coplanarSet;
    orthoSet = conSet->orthoSet;
    symmetricPlanes = conSet->symmetricPlanes;
    symmLines = conSet->symmLines;
    symmPoints = conSet->symmPoints;
}

bool ConstraintSet::checkCycleSpline(int i)
{
    return (net->polyLines[i][0] - net->polyLines[i][net->polyLines[i].size()-1]).length() < 1e-6;
}

void ConstraintSet::addCollinearConstraint(int bspIndex)
{
    BSpline& bsp = net->bsplines[bspIndex];
    if (net->curveType[bspIndex] == 1)
    {
        collinearSet.makeSet(bspIndex , 0);
        for (int i = 0; i < net->numPolyLines; i++)
        {
            if (i == bspIndex) continue;
            if (net->curveType[i] != 1) continue;
            if (net->bsplines[i].ctrlNodes.size() == 0) continue;
            if (collinearSet.sameRoot(bspIndex , 0 , i , 0)) continue;
            if (ConstraintDetector::checkCollinear(bsp.ctrlNodes.front() ,
                    bsp.ctrlNodes.back() , net->bsplines[i].ctrlNodes.front() ,
                    net->bsplines[i].ctrlNodes.back() , ConstraintDetector::collinearThr))
            {
                printf("collinear: (%d) , (%d)\n" , bspIndex , i);
                collinearSet.merge(bspIndex , 0 , i , 0);
            }
        }
    }
}

void ConstraintSet::addParallelConstraint(int bspIndex)
{
    BSpline& bsp = net->bsplines[bspIndex];
    if (net->curveType[bspIndex] == 1)
    {
        parallelSet.makeSet(bspIndex , 0);
        for (int i = 0; i < net->numPolyLines; i++)
        {
            if (i == bspIndex) continue;
            if (net->curveType[i] != 1) continue;
            if (net->bsplines[i].ctrlNodes.size() == 0) continue;
            if (parallelSet.sameRoot(bspIndex , 0 , i , 0)) continue;
            if (ConstraintDetector::checkParallel(bsp.ctrlNodes.front() ,
                    bsp.ctrlNodes.back() , net->bsplines[i].ctrlNodes.front() ,
                    net->bsplines[i].ctrlNodes.back() , ConstraintDetector::parallelThr))
            {
                printf("parallel: (%d) , (%d)\n" , bspIndex , i);
                parallelSet.merge(bspIndex , 0 , i , 0);
            }
        }
    }
}

void ConstraintSet::addCoplanarConstraint(int bspIndex)
{
    BSpline& bsp = net->bsplines[bspIndex];
    Plane& plane = net->planes[bspIndex];
    if (net->curveType[bspIndex] != 2)
    {
        coplanarSet.newCurve(bspIndex , 0);
        coplanarSet.connect(bspIndex , 0 , bspIndex , 0 , 1);
        for (int i = 0; i < net->numPolyLines; i++)
        {
            if (i == bspIndex) continue;
            if (net->curveType[i] != 1 && net->curveType[i] != 3) continue;
            if (net->bsplines[i].ctrlNodes.size() == 0) continue;
            if (ConstraintDetector::checkCoplanar(bsp , plane , net->bsplines[i] ,
                    net->planes[i] , ConstraintDetector::coplanarThr))
            {
                printf("coplanar: (%d) , (%d)\n" , bspIndex , i);
                coplanarSet.connect(bspIndex , 0 , i , 0 , 1);
            }
        }
    }
}

void ConstraintSet::addJunctionConstraint(int bspIndex)
{
    BSpline& bsp = net->bsplines[bspIndex];
    int numCtrlCurves = (int)bsp.ctrlNodes.size() - 1;
    for (int i = 0; i < numCtrlCurves; i++)
    {
        orthoSet.newCurve(bspIndex , i);
    }
	int candidates[2] = {0 , numCtrlCurves - 1};
    for (int candi = 0; candi < 2; candi++)
    {
		int i = candidates[candi];
        if (candi == 0)
        {
            int ni = net->getNodeIndex(bsp.ctrlNodes[0]);
			//printf("ni = %d, bsp.ctrlNodes[0] = (%.6f,%.6f,%.6f)\n" , ni , bsp.ctrlNodes[0].x ,
			//	bsp.ctrlNodes[0].y , bsp.ctrlNodes[0].z);
            for (int j = 0; j < net->edges[ni].size(); j++)
            {
                int ei = net->edges[ni][j].pli;
                if (ei == -1 || ei >= bspIndex) continue;
                // printf("(%d , %d)'s next: node = %d, line = %d\n" , bspIndex ,
                      // i , net->edges[ni][j].link , net->edges[ni][j].pli);
                
                int nodeIndex = (int)net->bsplines[ei].ctrlNodes.size() - 2;
                if (isEqual(bsp.ctrlNodes[0] , net->bsplines[ei].ctrlNodes[0])) nodeIndex = 1;
                
                int curveIndex = (nodeIndex == 1 ? 0 : nodeIndex);
                if (ConstraintDetector::checkOrtho(bsp.ctrlNodes[0] , bsp.ctrlNodes[1] ,
                        net->bsplines[ei].ctrlNodes[nodeIndex] , ConstraintDetector::orthoThr))
                {
                    printf("orthogonal: (%d , %d) , (%d , %d)\n" , bspIndex , i ,
                           ei , curveIndex);
                    orthoSet.connect(bspIndex , i , ei , curveIndex , 1);
                }
                else if (ConstraintDetector::checkTangent(bsp.ctrlNodes[0] , bsp.ctrlNodes[1] ,
                        net->bsplines[ei].ctrlNodes[nodeIndex] , ConstraintDetector::tangentThr))
                {
                    printf("tangent: (%d , %d) , (%d , %d)\n" , bspIndex , i ,
                           ei , curveIndex);
                    orthoSet.connect(bspIndex , i , ei , curveIndex , 2);
                }
            }
        }
        else
        {
            int ni = net->getNodeIndex(bsp.ctrlNodes[i + 1]);
            for (int j = 0; j < net->edges[ni].size(); j++)
            {
                int ei = net->edges[ni][j].pli;
                if (ei == -1 || ei >= bspIndex) continue;
                // printf("(%d , %d)'s next: node = %d, line = %d\n" , bspIndex ,
                      // i , net->edges[ni][j].link , net->edges[ni][j].pli);
                
                int nodeIndex = (int)net->bsplines[ei].ctrlNodes.size() - 2;
                if (isEqual(bsp.ctrlNodes[i + 1] , net->bsplines[ei].ctrlNodes[0])) nodeIndex = 1;
                
                int curveIndex = (nodeIndex == 1 ? 0 : nodeIndex);
                if (ConstraintDetector::checkOrtho(bsp.ctrlNodes[i + 1] , bsp.ctrlNodes[i] ,
                        net->bsplines[ei].ctrlNodes[nodeIndex] , ConstraintDetector::orthoThr))
                {
                    printf("orthogonal: (%d , %d) , (%d , %d)\n" , bspIndex , i ,
                           ei , curveIndex);
                    orthoSet.connect(bspIndex , i , ei , curveIndex , 1);
                }
                else if (ConstraintDetector::checkTangent(bsp.ctrlNodes[i + 1] , bsp.ctrlNodes[i] ,
                        net->bsplines[ei].ctrlNodes[nodeIndex] , ConstraintDetector::tangentThr))
                {
                    printf("tangent: (%d , %d) , (%d , %d)\n" , bspIndex , i ,
                           ei , curveIndex);
                    orthoSet.connect(bspIndex , i , ei , curveIndex , 2);
                }
            }
        }
    }
}
void ConstraintSet::addSymmetryConstraint(int bspIndex, bool add)
{
    if (net->curveType[bspIndex] == 1)
    {
        vec3d &x1 = net->bsplines[bspIndex].ctrlNodes[0];
        vec3d &x2 = net->bsplines[bspIndex].ctrlNodes[1];
        vec3d n1 = x1 - x2;
        n1.normalize();
        for (int i = 0; i < net->numPolyLines; ++ i)
        {
            if (i == bspIndex) continue;
            if (net->curveType[i] == 1)
            {
                vec3d &x3 = net->bsplines[i].ctrlNodes[0];
                vec3d &x4 = net->bsplines[i].ctrlNodes[1];
                vec3d n2 = x3 - x4;
                n2.normalize();
                if (abs((x1 - x2).length() - (x3 - x4).length()) > 1e-6) continue;
                if (ConstraintDetector::checkSymmetry(x1, n1, x3, n2,
                        ConstraintDetector::symmetryThr))
                {
                    Plane p1((x1 + x3) / 2, x1 - x3);
                    Plane p2((x2 + x4) / 2, x2 - x4);
                    if (p1.dist(p2) < ConstraintDetector::symmetryThr)
                    {
                        p1.add(p2);
                        p1.weight = (x1 - x2).length();
                        printf("add symmetry : (%d, %d)\n" , bspIndex, i);
                        addSymmetryPlane(p1, add, bspIndex, i);
                    }
                }
                if (ConstraintDetector::checkSymmetry(x1, n1, x4, -n2,
                        ConstraintDetector::symmetryThr))
                {
                    Plane p1((x1 + x4) / 2, x1 - x4);
                    Plane p2((x2 + x3) / 2, x2 - x3);
                    if (p1.dist(p2) < ConstraintDetector::symmetryThr)
                    {
                        p1.add(p2);
                        p1.weight = (x1 - x2).length();
                        printf("add symmetry : (%d, %d)\n" , bspIndex, i);
                        addSymmetryPlane(p1, add, bspIndex, i);
                    }
                }
            }
        }
    }
    else
    {
        int sampleNum = 10;
        int pointNum = net->polyLines[bspIndex].size();
        if (checkCycleSpline(bspIndex)) --pointNum;
        int step = pointNum / sampleNum;
        double length = 0;
        Path &c1 = net->polyLines[bspIndex];
        
        for (int i = 1; i < pointNum; ++ i) length += (c1[i] - c1[i-1]).length();
        
        for (int i = 0; i < net->numPolyLines; ++ i)
        {
            if (i == bspIndex) continue;
            if (net->curveType[i] == net->curveType[bspIndex] &&
                (!checkCycleSpline(bspIndex)^checkCycleSpline(i)))
            {
                bool flg = false;
                Path &c2 = net->polyLines[i];
                for (int j = 0; j < pointNum; ++ j)
                {
                    Plane p(0);
                    for (int k = 0; k < pointNum; k += step)
                    {
                        Plane nextPlane((c1[k]+c2[(j+k)%pointNum])/2, c1[k]-c2[(j+k)%pointNum], (c1[k]-c2[(j+k)%pointNum]).length());
                        p.add(nextPlane);
                    }
                    double dist = 0;
                    for (int k = 0; k < pointNum; k += step)
                    {
                        dist += (p.reflect(c1[k]) - c2[(j+k)%pointNum]).length();
                    }
                    if (dist < ConstraintDetector::symmetryThr)
                    {
                        p.weight = length;
                        addSymmetryPlane(p, add, bspIndex, i);
                        net->bsplines[bspIndex].copyBSP(net->bsplines[i]);
                        for (int t = 0; t < net->bsplines[i].ctrlNodes.size(); ++ t)
                        {
                            net->bsplines[bspIndex].ctrlNodes[t] = p.reflect(net->bsplines[i].ctrlNodes[t]);
                        }
                        flg = true;
                        break;
                    }
                    if (!checkCycleSpline(bspIndex))
                    {
                        if (flg)
                        {
                            net->bsplines[bspIndex].ctrlNodes[0] = c1[0];
                            net->bsplines[bspIndex].ctrlNodes[net->bsplines[bspIndex].N-1] = c1[pointNum-1];
                        }
                        break;
                    }
                }
                if (flg) break;
                for (int j = 0; j < pointNum; ++ j)
                {
                    Plane p(0);
                    for (int k = 0; k < pointNum; k += step)
                    {
                        Plane nextPlane((c1[pointNum-1-k]+c2[(j+k)%pointNum])/2, c1[pointNum-1-k]-c2[(j+k)%pointNum], (c1[pointNum-1-k]-c2[(j+k)%pointNum]).length());
                        p.add(nextPlane);
                    }
                    double dist = 0;
                    for (int k = 0; k < pointNum; k += step)
                    {
                        dist += (p.reflect(c1[pointNum-1-k]) - c2[(j+k)%pointNum]).length();
                    }
                    if (dist < ConstraintDetector::symmetryThr)
                    {
                        p.weight = length;
                        addSymmetryPlane(p, add, bspIndex, i);
                        net->bsplines[bspIndex].copyBSP(net->bsplines[i]);
                        for (int t = 0; t < net->bsplines[i].ctrlNodes.size(); ++ t)
                        {
                            net->bsplines[bspIndex].ctrlNodes[t] = p.reflect(net->bsplines[i].ctrlNodes[t]);
                        }
                        flg = true;
                        break;
                    }
                    if (!checkCycleSpline(bspIndex))
                    {
                        if (flg)
                        {
                            net->bsplines[bspIndex].ctrlNodes[0] = c1[pointNum-1];
                            net->bsplines[bspIndex].ctrlNodes[net->bsplines[bspIndex].N-1] = c1[0];
                        }
                        break;
                    }
                }
                if (flg) break;
            }
        }
    }
}

int ConstraintSet::addSymmetryPlane(Plane &p, bool add, int a, int b)
{
    for (int i = 0; i < symmetricPlanes.size(); ++ i)
    {
        if (symmetricPlanes[i].dist(p) < ConstraintDetector::planeDiffThr)
        {
            if (add)
            {
                symmetricPlanes[i].add(p);
                if (a > -1 && b > -1)
                {
                    symmLines[i].push_back(std::make_pair(a, b));
                }
            }
            else
            {
                symmetricPlanes[i].remove(p);
            }
            return i;
        }
    }
    if (add)
    {
        symmetricPlanes.push_back(p);
        std::vector<std::pair<int, int> > tmp;
        if (a > -1 && b > -1)
            tmp.push_back(std::make_pair(a, b));
        symmLines.push_back(tmp);
        symmPoints.push_back(std::vector<SelfSymmIdx>());
    }
    return (int)symmetricPlanes.size() - 1;
}

void ConstraintSet::addSelfSymmetryConstraint(int bspIndex)
{
    int sampleNum = 10;
    int pointNum = net->polyLines[bspIndex].size();
    if (checkCycleSpline(bspIndex)) --pointNum;
    int step = pointNum / sampleNum / 2;
    double length = 0;
    Path &c = net->polyLines[bspIndex];
    for (int i = 0; i < pointNum; ++ i)
    {
        Plane p(0);
        for (int j = 0; j < sampleNum; ++ j)
        {
            int l = (i + j * step) % pointNum;
            int r = (i - 1 - j * step + pointNum) % pointNum;
            Plane nextp((c[l]+c[r]) / 2, c[l] - c[r], (c[1] - c[r]).length());
            p.add(nextp);
        }
        double dist = 0;
        for (int j = 0; j < sampleNum; ++ j)
        {
            int l = (i + j * step) % pointNum;
            int r = (i - 1 - j * step + pointNum) % pointNum;
            dist += (p.reflect(c[l]) - c[r]).length();
        }
        if (dist < ConstraintDetector::symmetryThr)
        {
            for (int j = 0; j < sampleNum; ++ j)
            {
                int l = (i + j * step) % pointNum;
                int r = (i - 1 - j * step + pointNum) % pointNum;
                addSelfSymmPlane(p, true, bspIndex, l, r);
            }
        }
        if (!checkCycleSpline(bspIndex))
        {
            break;
        }
    }
}

int ConstraintSet::addSelfSymmPlane(Plane &p, bool add, int l, int a, int b)
{
    for (int i = 0; i < symmetricPlanes.size(); ++ i)
    {
        if (symmetricPlanes[i].dist(p) < ConstraintDetector::planeDiffThr)
        {
            if (add)
            {
                symmetricPlanes[i].add(p);
                symmPoints[i].push_back(SelfSymmIdx(l, a, b));
            }
            else
            {
                symmetricPlanes[i].remove(p);
            }
            return i;
        }
    }
    if (add)
    {
        symmetricPlanes.push_back(p);
        std::vector<SelfSymmIdx> tmp;
        tmp.push_back(SelfSymmIdx(l, a, b));
        symmPoints.push_back(tmp);
        std::vector<std::pair<int, int> > tmp2;
        symmLines.push_back(tmp2);
    }
    return (int)symmetricPlanes.size() - 1;
}

void ConstraintSet::addTransformConstraint(int bspIndex)
{
}
