#define _WCHAR_H_CPLUSPLUS_98_CONFORMANCE_
#include "partialSymmetry.h"
#include "pointCloudUtils.h"
#include "RandGenerator.h"
#include <string>
#include <nlopt.h>


PartialSymmetry::PartialSymmetry()
{
    pcUtils = NULL;
    signSpaceTree = NULL;
}

void PartialSymmetry::init(PointCloudUtils *_pcUtils)
{
    pcUtils = _pcUtils;
    isSampled.resize(pcUtils->pcData.size());
}

void PartialSymmetry::samplePointsUniform(int numSamples)
{
    for (int i = 0; i < isSampled.size(); i++) isSampled[i] = false;
    numSamples = std::min(numSamples , (int)pcUtils->pcData.size());
    for (int i = 0; i < numSamples; i++)
    {
        for (;;)
        {
            int j = (int)(RandGenerator::genFloat() * isSampled.size());
            if (!isSampled[j])
            {
                isSampled[j] = true;
                break;
            }
        }
    }

    signData.clear();
    for (int i = 0; i < pcUtils->pcData.size(); i++)
    {
        if (!isSampled[i]) continue;
        Tensor ts;
        ts.hessian = pcUtils->lerpHessian(pcUtils->pcData[i].pos);
        pcUtils->calcTensorDecomposition(ts);
        Data sign;
        for (int j = 0; j < 3; j++)
            sign.pos[j] = ts.eigenVal[j];
        sign.n = pcUtils->pcData[i].pos;
        sign.index = i;
        signData.push_back(sign);
    }
}

void PartialSymmetry::calcVotes()
{
    signSpaceTree = new PointKDTree<Data>(signData);

    int totVotes = 500000;
    // int knn = totVotes / signData.size();
    int knn = 15;
    votes.clear();

    // for (int i = 0; i < isSampled.size(); i++) isSampled[i] = false;
    for (int i = 0; i < signData.size(); i++) for (int j = i + 1; j < signData.size(); j++)
    {
        if ((signData[i].pos - signData[j].pos).length() > 1e-3) continue;
        /*
        if (i == 0)
        {
            isSampled[signData[i].index] = true;
            isSampled[signData[j].index] = true;
            printf("i: (%.6f,%.6f,%.6f), (%.6f,%.6f,%.6f)\n" ,
                signData[i].pos.x , signData[i].pos.y , signData[i].pos.z ,
                signData[i].n.x , signData[i].n.y , signData[i].n.z);
            printf("j: (%.6f,%.6f,%.6f), (%.6f,%.6f,%.6f)\n\n" ,
                signData[j].pos.x , signData[j].pos.y , signData[j].pos.z ,
                signData[j].n.x , signData[j].n.y , signData[j].n.z);
        }
        */
        vec3d normal = signData[i].n - signData[j].n;
        normal.normalize();
        if (normal.y < 0) normal = -normal;
        Plane plane((signData[i].n + signData[j].n) * 0.5 , normal);
        votes.push_back(plane);
        /*
        //int i = (int)(RandGenerator::genFloat() * signData.size());
        KnnQuery query(knn);
        signSpaceTree->searchKnn(0 , signData[i].pos , query);
        // writeLog("===== pos: (%.6f,%.6f,%.6f), sign: (%.6f,%.6f,%.6f) =====\n" ,
            // signData[i].n.x , signData[i].n.y , signData[i].n.z ,
            // signData[i].pos.x , signData[i].pos.y , signData[i].pos.z);
        for (int j = 0; j < query.knnPoints.size(); j++)
        {
            const Data* nb = query.knnPoints[j].point;
            // writeLog("pos: (%.6f,%.6f,%.6f), sign: (%.6f,%.6f,%.6f)\n" ,
                // nb->n.x , nb->n.y , nb->n.z , nb->pos.x , nb->pos.y , nb->pos.z);
            if (isEqual(signData[i].n , nb->n)) continue;
            vec3d normal = signData[i].n - nb->n;
            normal.normalize();
            if (normal.y < 0) normal = -normal;
            Plane plane((signData[i].n + nb->n) * 0.5 , normal);
            votes.push_back(plane);
        }
        */
    }
    delete signSpaceTree;
    
    std::string fileName = pcUtils->dataVotesPath + pcUtils->name + ".votes";
    FILE *fp = fopen(fileName.c_str() , "w");
    fprintf(fp , "%lu\n" , votes.size());
    for (int i = 0; i < votes.size(); i++)
    {
        double theta , phi;
        votes[i].calcAngle(theta , phi);
        fprintf(fp , "%.6f %.6f %.6f\n" , theta , phi , votes[i].d);
    }
    fclose(fp);

    /*
    fp = fopen("distances.txt" , "w");
    fprintf(fp , "%lu\n" , votes.size() * (votes.size() - 1));
    for (int i = 0; i < votes.size(); i++)
    {
        vec3d u;
        votes[i].calcAngle(u.x , u.y);
        u.z = votes[i].d;
        for (int j = i + 1; j < votes.size(); j++)
        {
            vec3d v;
            votes[j].calcAngle(v.x , v.y);
            v.z = votes[j].d;
            fprintf(fp , "%d %d %.6f\n" , i , j , (u - v).length());
        }
    }
    fclose(fp);
    */
}

void PartialSymmetry::findSymmPlanes()
{
	dcostheta = 2.0 / binNumTheta;
	dphi = pi / binNumPhi;
	maxR = 0;

	for (int i = 0; i < signData.size(); ++ i)
	{
		for (int j = i + 1; j < signData.size(); ++ j)
		{
			//Data x1 = pcUtils->pcData[i];
			//Data x2 = pcUtils->pcData[j];
			vec3d x1 = signData[i].n;
			vec3d x2 = signData[j].n;
			double d = (x1 - x2).length();
			if (d < 1e-6) continue;
			vec3d n = x1 - x2;
			vec3d x = (x1 + x2) / 2;
			n.normalize();
			if (n.x < 0) n = - n;
			double r = abs(n.dot(x));
			if (r > maxR) maxR = r;
		}
	}

	dr = maxR * 2 / binNumR;

	for (int i = 0; i < signData.size(); ++ i)
	{
		for (int j = i + 1; j < signData.size(); ++ j)
		{
			//Data x1 = pcUtils->pcData[i];
			//Data x2 = pcUtils->pcData[j];
			vec3d x1 = signData[i].n;
			vec3d x2 = signData[j].n;
			double d = (x1 - x2).length();
			if (d < 1e-6) continue;
			double weight = 1;
			vec3d n = x1 - x2;
			vec3d x = (x1 + x2) / 2;
			n.normalize();
			if (n.x < 0) n = - n;
			double theta = acos(n.z);
			double phi, cosphi;
			if (abs(sin(theta)) < 1e-6) cosphi = 0;
			else cosphi = n.y / sin(theta);
			if (cosphi < -1) cosphi = -1;
			if (cosphi > 1) cosphi = 1;
			phi = acos(cosphi);
			double r = - n.dot(x);

            int binTheta = (int)((n.z + 1.0) / dcostheta);
			int binPhi = (int)((phi - 0) / dphi);
			int binR = (int)((r + maxR) / dr);
			if (binTheta == binNumTheta) -- binTheta;
            //binTheta = clampValue(binTheta , 0 , binNumTheta - 1);
			if (binPhi == binNumPhi) -- binPhi;
            //binPhi = clampValue(binPhi , 0 , binNumPhi - 1);
			if (binR == binNumR) -- binR;
            //binR = clampValue(binR , 0 , binNumR - 1);
			//writeLog("position: %f, %f, %f, %f, %f, %f\n", n.x, n.y, n.z, theta, n.y / sin(theta), phi);
			//writeLog("bin:  %d, %d, %d\n", binTheta, binPhi, binR);
			weights[binTheta][binPhi][binR] += weight;
			symmPoints[binTheta][binPhi][binR].push_back(std::pair<int, int>(i, j));
		}
	}

	double maxWeight = 0;
	for (int i = 0; i < binNumTheta; ++ i)
	{
		for (int j = 0; j < binNumPhi; ++ j)
		{
			for (int k = 0; k < binNumR; ++ k)
			{
				if (weights[i][j][k] > maxWeight)
				{
					maxWeight = weights[i][j][k];
				}
			}
		}
	}
	double thresholdWeight = maxWeight * 0.5;


	for (int i = 0; i < binNumTheta; ++ i)
	{
		for (int j = 0; j < binNumPhi; ++ j)
		{
			for (int k = 0; k < binNumR; ++ k)
			{
				if (weights[i][j][k] > thresholdWeight)
				{
					/*double costheta = i * dcostheta - 1 + dcostheta / 2;
					double theta = acos(costheta);
					double phi = j * dphi + dphi / 2;
					double r = k * dr - maxR + dr / 2;
					vec3d n(sin(theta)*sin(phi), sin(theta)*cos(phi), cos(theta));
					Plane p(n.x, n.y, n.z, r);*/

					Plane p = adjustSymmPlane(i, j, k);
					if (p.weight > 0)
						candidatePlanes.push_back(p);
				}
			}
		}
	}
	
	for (int i = 1; i < candidatePlanes.size(); ++ i)
	{
		for (int j = 0; j < i; ++ j)
		{
			Plane &p1 = candidatePlanes[j];
			Plane &p2 = candidatePlanes[i];
			if (p1.weight == 0) continue;
			if (p1.dist(p2) < 1e-2)
			{
				p1.add(p2);
				p2.weight = 0;
				break;
			}
		}
	}
	maxWeight = 0;
	for (int i = 0; i < candidatePlanes.size(); ++ i)
	{
		if (candidatePlanes[i].weight > maxWeight)
			maxWeight = candidatePlanes[i].weight;
	}
	thresholdWeight = 0.5 * maxWeight;
	for (int i = 0; i < candidatePlanes.size(); ++ i)
	{
		if (candidatePlanes[i].weight < thresholdWeight)
		{
			int last = candidatePlanes.size() - 1;
			swap(candidatePlanes[i], candidatePlanes[last]);
			candidatePlanes.pop_back();
		}
	}
}

Plane PartialSymmetry::adjustSymmPlane(int i, int j, int k)
{

	double costheta = i * dcostheta - 1 + dcostheta / 2;
	double theta = acos(costheta);
	double phi = j * dphi + dphi / 2;
	double r = k * dr - maxR + dr / 2;
	vec3d n(sin(theta)*sin(phi), sin(theta)*cos(phi), cos(theta));
	Plane p(n.x, n.y, n.z, r);

	double lb[3] = {costheta - dcostheta, phi - dphi, r - dr}; /* lower bounds */
	double ub[3] = {costheta + dcostheta, phi + dphi, r + dr}; /* lower bounds */
	nlopt_opt opt;
	opt = nlopt_create(NLOPT_LN_COBYLA, 3); /* algorithm and dimensionality */
	nlopt_set_lower_bounds(opt, lb);
	nlopt_set_upper_bounds(opt, ub);
	std::vector<std::pair<vec3d, vec3d> > points;
	for (int t = 0; t < symmPoints[i][j][k].size(); ++ t)
	{
		points.push_back(std::pair<vec3d, vec3d>(signData[symmPoints[i][j][k][t].first].n, signData[symmPoints[i][j][k][t].second].n));
	}
	nlopt_set_min_objective(opt, &PartialSymmetry::myfunc, &points);
	
	nlopt_set_xtol_rel(opt, 1e-4);

	double x[3] = {costheta, phi, r};  /* some initial guess */
	double minf; /* the minimum objective value, upon return */

	Plane adjust_p;
	if (nlopt_optimize(opt, x, &minf) < 0) {
		//writeLog("nlopt failed!\n");
		adjust_p.init(0, 0, 0, 0, 0);
	}
	else {
		//writeLog("found minimum at f(%g,%g) = %0.10g\n", x[0], x[1], minf);
		costheta = x[0];
		theta = acos(costheta);
		phi = x[1];
		r = x[2];
		vec3d adjust_n(sin(theta)*sin(phi), sin(theta)*cos(phi), cos(theta));
		adjust_p.init(adjust_n.x, adjust_n.y, adjust_n.z, r, symmPoints[i][j][k].size());
	}
	nlopt_destroy(opt);
	return adjust_p;
}

double PartialSymmetry::myconstraint(unsigned n, const double *x, double *grad, void *data)
{
    my_constraint_data *d = (my_constraint_data *) data;
    double a = d->a, b = d->b;
    if (grad) {
        grad[0] = 3 * a * (a*x[0] + b) * (a*x[0] + b);
        grad[1] = -1.0;
    }
    return ((a*x[0] + b) * (a*x[0] + b) * (a*x[0] + b) - x[1]);
 }

double PartialSymmetry::myfunc(unsigned n, const double *x, double *grad, void *my_func_data)
{
	double costheta = x[0];
	double theta = acos(costheta);
	double phi = x[1];
	double r = x[2];
	vec3d _n(sin(theta)*sin(phi), sin(theta)*cos(phi), cos(theta));
	Plane p(_n.x, _n.y, _n.z, r);

	std::vector<std::pair<vec3d, vec3d> > &pointpair = *(std::vector<std::pair<vec3d, vec3d> >*)my_func_data;
	double objective = 0;
	for (int i = 0; i < pointpair.size(); ++ i)
	{
		vec3d &p1 = pointpair[i].first;
		vec3d &p2 = pointpair[i].second;
		vec3d rp = p.reflect(p1);
		objective += (rp - p2).length();
	}
    return objective;
}