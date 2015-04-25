#define _WCHAR_H_CPLUSPLUS_98_CONFORMANCE_
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <algorithm>
#include <cassert>
#include "optimization.h"
#include "splineUtils.h"
#include "pointCloudUtils.h"

using namespace std;

Optimization::Optimization()
{
    net = NULL;

    /*
    largeBound = 0.003;
    smallBound = 0.001;
    */
}

Optimization::~Optimization()
{
    delete net;
}

void Optimization::init(CurveNet *_net , PointCloudUtils *_pcUtils)
{
    net = _net;
    pcUtils = _pcUtils;

	// for debug
	largeBound = 2;
    smallBound = 0;
	numStartPoints = 120;
	maxRealTime = 0.2;

    numVars = numConsts = numVaris = 0;
    vars.clear();
    double2vi.clear();

	straightlines.clear();
	bsplines.clear();
	straightlinesIdx.clear();
	bsplinesIdx.clear();
	lastDraw.clear();

    numCons = 0;
    cons.clear();

	coplanes.clear();
	coplanarPoints.clear();

    printf("adding variables...\n");
    for (int i = 0; i < net->numNodes; i++)
    {
        if (!net->nodesStat[i]) continue;
        double weight;
        if (net->nodesEditType[i] == 0)
        {
            weight = 5;
        }
        else if (net->nodesEditType[i] == 1)
        {
            weight = 1;
        }
        else if (net->nodesEditType[i] == 2)
        {
            weight = 15;
        }
        addOptVariable(OptVariable(0 , i , weight));
    }
    for (int i = 0; i < net->numPolyLines; i++)
    {
        if (net->curveType[i] == -1) continue;
        if (net->bsplines[i].ctrlNodes.size() <= 2) continue;
        double weight;
        if (net->bspEditType[i] == 0)
        {
            weight = 5;
        }
        else if (net->bspEditType[i] == 1)
        {
            weight = 1;
        }
        else if (net->bspEditType[i] == 2)
        {
            weight = 15;
        }
        for (int j = 1; j < (int)net->bsplines[i].ctrlNodes.size() - 1; j++)
        {
            addOptVariable(OptVariable(1 , i , j , weight));
        }
    }

    for (int i = 0; i < numVars; i++)
    {
        if (vars[i].type == 0)
        {
            if (net->isNodesFixed[vars[i].ni])
            {
                vars[i].isVar = false;
                vars[i].index = numConsts++;
            }
            else
            {
                bool flag = true;
                for (int j = 0; j < net->edges[vars[i].ni].size(); j++)
                {
                    int pli = net->edges[vars[i].ni][j].pli;
                    if (net->isCurvesFixed[pli])
                    {
                        vars[i].isVar = false;
                        vars[i].index = numConsts++;
                        flag = false;
                        break;
                    }
                }
                if (flag)
                {
                    vars[i].isVar = true;
                    vars[i].index = numVaris++;
                }
            }
        }
        else if (vars[i].type == 1)
        {
            if (!net->isCurvesFixed[vars[i].ni])
            {
                vars[i].isVar = true;
                vars[i].index = numVaris++;
            }
            else
            {
                vars[i].isVar = false;
                vars[i].index = numConsts++;
            }
        }
    }

    /*
    // print vars
    writeLog("===== vars =====\n");
    for (int i = 0; i < numVars; i++)
    {
        writeLog("var %d: (%d , %d , %d)\n" , i , vars[i].type , vars[i].ni , vars[i].ci);
    }
    */
    
	//find var index for lines
    printf("finding var index for lines...\n");
	for (int i = 0; i < numVars; ++ i) lastDraw.push_back(false);
	for (int i = 0; i < net->numPolyLines; ++ i)
	{
		vector<int> tmpvec;
		if (net->curveType[i] == -1) continue;
		if (net->curveType[i] == 1)
		{
            std::pair<OptVariable , OptVariable> curveVars = bsp2var(i , 0 , 1);
			int tmpidx = getOptVarIndex(curveVars.first);
			tmpvec.push_back(tmpidx);
			tmpidx = getOptVarIndex(curveVars.second);
			tmpvec.push_back(tmpidx);
            bool flag_all_const = true;
            for (int j = 0; j < tmpvec.size(); j++)
            {
                if (vars[tmpvec[j]].isVar) flag_all_const = true;
            }
            if (flag_all_const) continue;
			straightlines.push_back(tmpvec);
			featurelines.push_back(tmpvec);
			curveIdx.push_back(straightlinesIdx.size());
			straightlinesIdx.push_back(i);
		}
		else
		{
            int numCtrlCurves = (int)net->bsplines[i].ctrlNodes.size() - 1;
            std::pair<OptVariable , OptVariable> curveVars = bsp2var(i , 0 , numCtrlCurves);
			int tmpidx = getOptVarIndex(curveVars.first);
			tmpvec.push_back(tmpidx);
			for (int j = 1; j < (int)net->bsplines[i].ctrlNodes.size() - 1; ++ j)
			{
				tmpidx = getOptVarIndex(OptVariable(1, i, j));
				tmpvec.push_back(tmpidx);
			}
            curveVars = bsp2var(i , numCtrlCurves - 1 , numCtrlCurves);
			tmpidx = getOptVarIndex(curveVars.second);
			tmpvec.push_back(tmpidx);
            bool flag_all_const = true;
            for (int j = 0; j < tmpvec.size(); j++)
            {
                if (vars[tmpvec[j]].isVar) flag_all_const = true;
            }
            if (flag_all_const) continue;
			bsplines.push_back(tmpvec);
			featurelines.push_back(tmpvec);
			curveIdx.push_back(bsplinesIdx.size());
			bsplinesIdx.push_back(i);
		}
	}

	//constraint
    printf("collinear and parallel constraints...\n");
    for (int i = 0; i < net->numPolyLines; i++)
    {
        if (net->curveType[i] != 1) continue;

        PolyLineIndex u , v;
        int u1 , u2 , v1 , v2;

        // collinear
        std::pair<int , int> root = net->conSet->collinearSet.find(i , 0);
        if (net->curveType[root.first] == -1)
        {
            std::pair<int , int> tmp;
            int j;
            for (j = 0; j < net->numPolyLines; j++)
            {
                if (net->curveType[j] != 1 || j == i) continue;
                tmp = net->conSet->collinearSet.find(j , 0);
                if (tmp.first == root.first) break;
            }
            root.first = j;
        }
        if (root.first != i)
        {
            u = net->polyLinesIndex[i];
            v = net->polyLinesIndex[root.first];
            u1 = getOptVarIndex(OptVariable(0 , u.ni[0]));
            u2 = getOptVarIndex(OptVariable(0 , u.ni[1]));
            v1 = getOptVarIndex(OptVariable(0 , v.ni[0]));
            v2 = getOptVarIndex(OptVariable(0 , v.ni[1]));
            if (isAllConst(u1 , u2 , v1 , v2)) continue;
            cons.push_back(OptConstraints(u1 , u2 , v1 , v2 , st_collinear));
        }
        // parallel
        root = net->conSet->parallelSet.find(i , 0);
        if (net->curveType[root.first] == -1)
        {
            std::pair<int , int> tmp;
            int j;
            for (j = 0; j < net->numPolyLines; j++)
            {
                if (net->curveType[j] != 1 || j == i) continue;
                tmp = net->conSet->parallelSet.find(j , 0);
                if (tmp.first == root.first) break;
            }
            root.first = j;
        }
        if (root.first != i)
        {
            u = net->polyLinesIndex[i];
            v = net->polyLinesIndex[root.first];
            u1 = getOptVarIndex(OptVariable(0 , u.ni[0]));
            u2 = getOptVarIndex(OptVariable(0 , u.ni[1]));
            v1 = getOptVarIndex(OptVariable(0 , v.ni[0]));
            v2 = getOptVarIndex(OptVariable(0 , v.ni[1]));
            if (isAllConst(u1 , u2 , v1 , v2)) continue;
            cons.push_back(OptConstraints(u1 , u2 , v1 , v2 , st_parallel));
        }
    }

    // coplanar
    printf("coplanar constraints...\n");
    for (int i = 0; i < net->numPolyLines; i++)
    {
        if (net->curveType[i] == 3)
        {
            //int numCtrlCurves = (int)net->bsplines[i].ctrlNodes.size() - 1;
            for (int j = 0; j < net->bsplines[i].ctrlNodes.size(); j++)
            {
                int vi;
                if (j == 0 || j == (int)net->bsplines[i].ctrlNodes.size() - 1)
                {
                    OptVariable var(0 , net->getNodeIndex(net->bsplines[i].ctrlNodes[j]));
                    vi = getOptVarIndex(var);
                }
                else
                {
                    OptVariable var(1 , i , j);
                    vi = getOptVarIndex(var);
                }
                Plane plane = net->planes[i];
                double len = pathLength(net->polyLines[i]);
                plane.weight = len * len;
                addCoplanar(vi , plane);
            }
        }
    }

    printf("coplanar between curves...\n");
    for (int i = 0; i < net->numPolyLines; i++)
    {
        if (net->curveType[i] == 2 || net->curveType[i] == -1) continue;
        for (int j = i + 1; j < net->numPolyLines; j++)
        {
            if (net->curveType[j] == 2 || net->curveType[j] == -1) continue;
            if (net->curveType[i] == 1 && net->curveType[j] == 1 &&
                net->conSet->parallelSet.find(i , 0) == net->conSet->parallelSet.find(j , 0))
                continue;

            if (net->conSet->coplanarSet.getMark(i , 0 , j , 0) == 1)
            {
                Path path , path2;
                resampleBspUniform(net->bsplines[i] , 30 , path);
                resampleBspUniform(net->bsplines[j] , 30 , path2);
                double len1 = pathLength(path);
                double len2 = pathLength(path2);
                for (int k = 0; k < path2.size(); k++) path.push_back(path2[k]);
                Plane plane;
                plane.fitFromPoints(path);
                plane.weight = len1 * len2;
                for (int x = 0; x < net->bsplines[i].ctrlNodes.size(); x++)
                {
                    int vi;
                    if (x == 0 || x == (int)net->bsplines[i].ctrlNodes.size() - 1)
                    {
                        OptVariable var(0 , net->getNodeIndex(net->bsplines[i].ctrlNodes[x]));
                        vi = getOptVarIndex(var);
                    }
                    else
                    {
                        OptVariable var(1 , i , x);
                        vi = getOptVarIndex(var);
                    }
                    addCoplanar(vi , plane);
                }
                for (int y = 0; y < net->bsplines[j].ctrlNodes.size(); y++)
                {
                    int vi;
                    if (y == 0 || y == (int)net->bsplines[j].ctrlNodes.size() - 1)
                    {
                        OptVariable var(0 , net->getNodeIndex(net->bsplines[j].ctrlNodes[y]));
                        vi = getOptVarIndex(var);
                    }
                    else
                    {
                        OptVariable var(1 , j , y);
                        vi = getOptVarIndex(var);
                    }
                    addCoplanar(vi , plane);
                }
            }
        }
    }

    printf("adding coplanes...\n");
	for (int i = 0; i < coplanes.size(); ++ i)
	{
		if (coplanarPoints[i].size() <= 3)
		{
			swap(coplanarPoints[i], coplanarPoints[coplanes.size() - 1]);
			swap(coplanes[i], coplanes[coplanes.size() - 1]);
			coplanarPoints.pop_back();
			coplanes.pop_back();
		}
	}
    /*
    net->coplanes.clear();
    for (int i = 0; i < net->planes.size(); i++)
    {
        if (net->curveType[i] == 3) net->coplanes.push_back(net->planes[i]);
    }
    */
    net->coplanes = coplanes;
    
    if (false)
    {
    // coplanar
    printf("coplanar constraints...\n");
    for (int i = 0; i < net->numPolyLines; i++)
    {
        if (net->curveType[i] == 3)
        {
            int numCtrlCurves = (int)net->bsplines[i].ctrlNodes.size() - 1;
            for (int j = 0; j < numCtrlCurves; j++)
            {
                for (int k = j + 1; k < numCtrlCurves; k++)
                {
                    std::pair<OptVariable , OptVariable> u = bsp2var(i , j , numCtrlCurves);
                    std::pair<OptVariable , OptVariable> v = bsp2var(i , k , numCtrlCurves);
                    int u1 = getOptVarIndex(u.first);
                    int u2 = getOptVarIndex(u.second);
                    int v1 = getOptVarIndex(v.first);
                    int v2 = getOptVarIndex(v.second);
                    //if (u1 == v1 || u1 == v2 || u2 == v1 || u2 == v2) continue;
                    addCoplanar(u1, u2, v1, v2);
                }
            }
        }
    }

    printf("coplanar between curves...\n");
    for (int i = 0; i < net->numPolyLines; i++)
    {
        if (net->curveType[i] == 2 || net->curveType[i] == -1) continue;
        for (int j = i + 1; j < net->numPolyLines; j++)
        {
            if (net->curveType[j] == 2 || net->curveType[j] == -1) continue;
            if (net->conSet->coplanarSet.getMark(i , 0 , j , 0) == 1)
            {
                for (int x = 0; x < (int)net->bsplines[i].ctrlNodes.size() - 1; x++)
                {
                    for (int y = 0; y < (int)net->bsplines[j].ctrlNodes.size() - 1; y++)
                    {
                        std::pair<OptVariable , OptVariable> u = bsp2var(i , x ,
                            (int)net->bsplines[i].ctrlNodes.size() - 1);
                        std::pair<OptVariable , OptVariable> v = bsp2var(j , y ,
                            (int)net->bsplines[j].ctrlNodes.size() - 1);
                        int u1 = getOptVarIndex(u.first);
                        int u2 = getOptVarIndex(u.second);
                        int v1 = getOptVarIndex(v.first);
                        int v2 = getOptVarIndex(v.second);
						if (u1 == v1 || u1 == v2 || u2 == v1 || u2 == v2) continue;
						addCoplanar(u1, u2, v1, v2);
                        //cons.push_back(OptConstraints(u1 , u2 , v1 , v2 , st_coplanar));
                    }
                }
            }
        }
    }

    printf("adding coplanes...\n");
	for (int i = 0; i < coplanes.size(); ++ i)
	{
		if (coplanarPoints[i].size() <= 3)
		{
			swap(coplanarPoints[i], coplanarPoints[coplanes.size() - 1]);
			swap(coplanes[i], coplanes[coplanes.size() - 1]);
			coplanarPoints.pop_back();
			coplanes.pop_back();
		}
	}

    net->coplanes.clear();
    for (int i = 0; i < net->planes.size(); i++)
    {
        if (net->curveType[i] == 3) net->coplanes.push_back(net->planes[i]);
    }
	// net->coplanes = coplanes;

    }
    
    // ortho & tangent
    printf("orthogonal and tangent constraints...\n");
    for (int i = 0; i < net->numPolyLines; i++)
    {
        if (net->curveType[i] == -1) continue;
        for (int j = i + 1; j < net->numPolyLines; j++)
        {
            if (net->curveType[j] == -1) continue;
            for (int x = 0; x < (int)net->bsplines[i].ctrlNodes.size() - 1; x++)
            {
                for (int y = (i != j ? 0 : x + 1); y < (int)net->bsplines[j].ctrlNodes.size() - 1; y++)
                {
                    int mark = net->conSet->orthoSet.getMark(i , x , j , y);
                    if (mark == 1 || mark == 2)
                    {
                        std::pair<OptVariable , OptVariable> u = bsp2var(i , x ,
                            (int)net->bsplines[i].ctrlNodes.size() - 1);
                        std::pair<OptVariable , OptVariable> v = bsp2var(j , y ,
                            (int)net->bsplines[j].ctrlNodes.size() - 1);
                        int u1 = getOptVarIndex(u.first);
                        int u2 = getOptVarIndex(u.second);
                        int v1 = getOptVarIndex(v.first);
                        int v2 = getOptVarIndex(v.second);

                        if (isAllConst(u1 , u2 , v1 , v2)) continue;
                        bool flag = (u1 == v1) || (u1 == v2) || (u2 == v1) || (u2 == v2);
                        assert(flag);
                        int x0 , x1 , x2;
                        if (u1 == v1)
                        {
                            x0 = u1; x1 = u2; x2 = v2;
                        }
                        else if (u1 == v2)
                        {
                            x0 = u1; x1 = u2; x2 = v1;
                        }
                        else if (u2 == v1)
                        {
                            x0 = u2; x1 = u1; x2 = v2;
                        }
                        else if (u2 == v2)
                        {
                            x0 = u2; x1 = u1; x2 = v1;
                        }
                        else
                        {
                            assert(false);
                        }
                        if (mark == 1)
                        {
                            cons.push_back(OptConstraints(x0 , x1 , x0 , x2 , st_ortho));
                        }
                        else if (mark == 2)
                        {
                            cons.push_back(OptConstraints(x0 , x1 , x0 , x2 , st_tangent));
                        }
                    }
                }
            }
        }
    }

	// symmetric
	for (int i = 0; i < net->conSet->symmLines.size(); ++ i)
	{
	}
    numCons = cons.size();

    /*
    // print constraints
    writeLog("===== cons =====\n");
    for (int i = 0; i < numCons; i++)
    {
        writeLog("cons %d: type = %d, (%d , %d) <-> (%d , %d)\n" , i , (int)cons[i].type ,
            cons[i].u1 , cons[i].u2 , cons[i].v1 , cons[i].v2);
    }
    */
}

void Optimization::generateDAT(string file)
{
	ofstream fout(file.data());
    // fout << "param N := " << numVars - 1 << ";\n";
    fout << "param N := " << numVaris - 1 << ";\n";
    fout << "param N_const := " << numConsts - 1 << ";\n";
    fout << "param largeBound := " << largeBound << ";\n";
    fout << "param smallBound := " << smallBound << ";\n";

    fout << "param init_p :=\n";
	for (int i = 0; i < numVars; ++ i)
	{
        if (!vars[i].isVar) continue;
		fout << "\t[" << vars[i].index << ", *]";
        // fout << "\t[" << i << ", *]";
		if (vars[i].type == 0)
		{
			fout << " 1 " << net->nodes[vars[i].ni].x
				 << " 2 " << net->nodes[vars[i].ni].y
				 << " 3 " << net->nodes[vars[i].ni].z
				 << "\n";
		}
		else
		{
			fout << " 1 " << net->bsplines[vars[i].ni].ctrlNodes[vars[i].ci].x
				 << " 2 " << net->bsplines[vars[i].ni].ctrlNodes[vars[i].ci].y
				 << " 3 " << net->bsplines[vars[i].ni].ctrlNodes[vars[i].ci].z
				 << "\n";
		}
	}
	fout << ";\n";

    fout << "param p_const :=\n";
    for (int i = 0; i < numVars; ++ i)
	{
        if (vars[i].isVar) continue;
		fout << "\t[" << vars[i].index << ", *]";
        // fout << "\t[" << i << ", *]";
		if (vars[i].type == 0)
		{
			fout << " 1 " << net->nodes[vars[i].ni].x
				 << " 2 " << net->nodes[vars[i].ni].y
				 << " 3 " << net->nodes[vars[i].ni].z
				 << "\n";
		}
		else
		{
			fout << " 1 " << net->bsplines[vars[i].ni].ctrlNodes[vars[i].ci].x
				 << " 2 " << net->bsplines[vars[i].ni].ctrlNodes[vars[i].ci].y
				 << " 3 " << net->bsplines[vars[i].ni].ctrlNodes[vars[i].ci].z
				 << "\n";
		}
	}
    fout << ";\n";

    fout << "param p_bound :=\n";
	for (int i = 0; i < numVars; ++ i)
	{
        if (!vars[i].isVar) continue;
		fout << " " << vars[i].index << " ";
        fout << largeBound << "\n";
        // if (vars[i].isVar) fout << largeBound << "\n";
        // else fout << smallBound << "\n";
	}
	fout << ";\n";

    fout << "param p_weight :=\n";
    for (int i = 0; i < numVars; ++ i)
    {
        if (!vars[i].isVar) continue;
        fout << " " << vars[i].index << " ";
        fout << vars[i].weight << "\n";
    }
    fout << ";\n";

	fout << "param PN := " << (int)coplanes.size() - 1 << ";\n";
	
	fout << "param init_plane :=\n";
	for (int i = 0; i < coplanes.size(); ++ i)
	{
		fout << "\t[" << i << ", *]";
		fout << " 1 " << coplanes[i].n.x
			 << " 2 " << coplanes[i].n.y
			 << " 3 " << coplanes[i].n.z
			 << " 4 " << coplanes[i].d
			 << "\n";
	}
	fout << ";\n";

    fout << "param RPN := " << (int)net->conSet->symmetricPlanes.size() - 1 << ";\n";

	fout << "param init_symmetric_plane :=\n";
	for (int i = 0; i < net->conSet->symmetricPlanes.size(); ++ i)
	{
		fout << "\t[" << i << ", *]";
		fout << " 1 " << net->conSet->symmetricPlanes[i].n.x
			 << " 2 " << net->conSet->symmetricPlanes[i].n.y
			 << " 3 " << net->conSet->symmetricPlanes[i].n.z
			 << " 4 " << net->conSet->symmetricPlanes[i].d
			 << "\n";
	}
	fout << ";\n";

    fout << "param SN := " << (int)straightlines.size() - 1 << ";\n";
	fout << "param sidx :=\n";
	for (int i = 0; i < straightlines.size(); ++ i)
	{
		fout << "\t[" << i << ", *]";
		fout << " 1 " << straightlines[i][0]
			 << " 2 " << straightlines[i][1]
			 << "\n";
	}
	fout << ";\n";

	fout << "param BN := " << (int)bsplines.size() - 1 << ";\n";
	fout << "param CN :=\n";
	for (int i = 0; i < bsplines.size(); ++ i)
	{
		fout << "\t" << i << " " << (int)bsplines[i].size() - 1 << "\n";
	}
	fout << ";\n";
	fout << "param bidx :=\n";
	for (int i = 0; i < bsplines.size(); ++ i)
	{
		fout << "\t[" << i << ", *]";
		for (int j = 0; j < bsplines[i].size(); ++ j)
		{
			fout << " " << j << " " << bsplines[i][j];
		}
		fout << "\n";
	}
	fout << ";\n";

	fout << "param SPN :=\n";
	for (int i = 0; i < straightlines.size(); ++ i)
	{
		fout << "\t" << i << " " 
			 << (int)net->originPolyLines[straightlinesIdx[i]].size() - 1 << "\n";
	}
	fout << ";\n";
	fout << "param sp :=\n";
	for (int i = 0; i < straightlines.size(); ++ i)
	{
		Path &tmpline = net->originPolyLines[straightlinesIdx[i]];
		for (int j = 0; j < tmpline.size(); ++ j)
		{
			fout << "\t[" << i << ", " << j << ", *]";
			fout << " 1 " << tmpline[j].x
				 << " 2 " << tmpline[j].y
				 << " 3 " << tmpline[j].z
				 << "\n";
		}
	}
	fout << ";\n";
	fout << "param BPN :=\n";
	for (int i = 0; i < bsplines.size(); ++ i)
	{
		fout << "\t" << i << " " 
			 << (int)net->originPolyLines[bsplinesIdx[i]].size() - 1 << "\n";
	}
	fout << ";\n";
	fout << "param bp :=\n";
	for (int i = 0; i < bsplines.size(); ++ i)
	{
		Path &tmpline = net->originPolyLines[bsplinesIdx[i]];
		for (int j = 0; j < tmpline.size(); ++ j)
		{
			fout << "\t[" << i << ", " << j << ", *]";
			fout << " 1 " << tmpline[j].x
				 << " 2 " << tmpline[j].y
				 << " 3 " << tmpline[j].z
				 << "\n";
		}
	}
	fout << ";\n";

	fout << "param coef :=\n";
	for (int i = 0; i < bsplines.size(); ++ i)
	{
		Path &tmpline = net->originPolyLines[bsplinesIdx[i]];
		for (int j = 0; j < tmpline.size(); ++ j)
		{
			fout << "\t[" << i << ", " << j << ", *]";
			for (int k = 0; k < bsplines[i].size(); ++ k)
			{
                fout << " " << k << " " << net->bsplines[bsplinesIdx[i]].coefs[j][k];
                // fout << " " << k << " " << net->bsplines[bsplinesIdx[i]].coefs[net->mapOrigin[bsplinesIdx[i]][j]][k];
			}
			fout << "\n";
		}
	}
	fout << ";\n";
	fout.close();
}

void Optimization::generateMOD(string file)
{
	ofstream fout(file.data());
	fout << "# parameters\n";
	fout << "set Dim2 = 1..2;\n";
	fout << "set Dim3 = 1..3;\n";
	fout << "set Dim4 = 1..4;\n";

    fout << "param largeBound;\n";
    fout << "param smallBound;\n";

	fout << "param N;\n";
    fout << "param N_const;\n";
	fout << "param init_p {0..N, Dim3};\n";
    fout << "param p_const {0..N_const, Dim3};\n";
	fout << "param p_bound {0..N};\n";
	fout << "param PN;\n";
	fout << "param init_plane {0..PN, Dim4};\n";
	fout << "param RPN;\n";
	fout << "param init_symmetric_plane {0..RPN, Dim4};\n";
	fout << "param SN;\n";
	fout << "param sidx {0..SN, Dim2};\n";
	fout << "param BN;\n";
	fout << "param CN {0..BN};\n";
	fout << "param bidx {i in 0..BN, 0..CN[i]};\n";
	fout << "param SPN {0..SN};\n";
	fout << "param sp {i in 0..SN, 0..SPN[i], Dim3};\n";
	fout << "param BPN {0..BN};\n";
	fout << "param bp {i in 0..BN, 0..BPN[i], Dim3};\n";
	fout << "param coef {i in 0..BN, 0..BPN[i], 0..CN[i]};\n";
	fout << "param p_weight {0..N};\n";

    fout << "\n# variables\n";
	fout << "var p {i in 0..N, t in Dim3} >= init_p[i, t] - p_bound[i], <= init_p[i, t] + p_bound[i], := init_p[i, t];\n";
	fout << "var plane {i in 0..PN, t in Dim4} >= init_plane[i, t] - largeBound, <= init_plane[i, t] + largeBound, := init_plane[i, t];\n";
    //fout << "var plane {i in 0..PN, t in Dim4} >= init_plane[i, t] - 0.05, <= init_plane[i, t] + 0.05, := init_plane[i, t];\n";
    fout << "var symmetric_plane {i in 0..RPN, t in Dim4} >= init_symmetric_plane[i, t] - largeBound, <= init_symmetric_plane[i, t] + largeBound, := init_symmetric_plane[i, t];\n";

	fout << "\n# intermediate variables\n";
	// fout << "var dir {i in 0..SN, t in Dim3} = (p[sidx[i, 2], t] - p[sidx[i, 1], t])"
		 // << " / sqrt(sum{j in Dim3} (p[sidx[i, 1], j] - p[sidx[i, 2], j]) ^ 2);\n";
    for (int i = 0; i < straightlines.size(); i++)
    {
        fout << "var dir" << i << " {t in Dim3} = ("
             << var2str(straightlines[i][1] , -1) << " - "
             << var2str(straightlines[i][0] , -1) << ")"
             << "/ sqrt(sum{j in Dim3} (" << var2str(straightlines[i][0] , -2) << " - "
             << var2str(straightlines[i][1] , -2) << ") ^ 2);\n";
    }

	fout << "\n# objective\n";
	fout << "minimize total_cost:\n";
	fout << "5 * (sum {i in 0..N, t in Dim3} "
		 << "p_weight[i] * (p[i, t] - init_p[i, t]) ^ 2)\n";
    /*
	fout << "+ 1 * (sum{i in 0..SN} p_weight[sidx[i,2]] * (sum{j in 0..SPN[i]}"
		<< "sum{t in Dim3}("
		<< "(sum{k in Dim3}(sp[i,j,k]-p[sidx[i,2],k])*dir[i,k])*dir[i,t]"
		<< "-(sp[i,j,t]-p[sidx[i,2],t])"
		<< ")^2"
		<< ")/SPN[i])\n";
    */
    fout << generateStraightLineDist();
    /*
	fout << "+ 1 * (sum{i in 0..BN} p_weight[bidx[i,0]] * (sum{j in 0..BPN[i]}"
		 << "sum{t in Dim3}("
		 << "bp[i,j,t]-sum{k in 0..CN[i]}coef[i,j,k]*p[bidx[i,k],t]"
		 << ")^2"
		 << ")/BPN[i])\n";
    */
    fout << generateBsplineDist();
	for (int i = 0; i < numCons; ++ i)
	{
		fout << "+ 10 * ";
		switch(cons[i].type)
		{
            case st_collinear:
                fout << generateLineCollinear(cons[i].u1, cons[i].u2, cons[i].v1, cons[i].v2);
                break;
            // case st_coplanar:
                // fout << generateLineCoplanar(cons[i].u1, cons[i].u2, cons[i].v1, cons[i].v2);
                // break;
            case st_ortho:
                fout << generateLineOrtho(cons[i].u1, cons[i].u2, cons[i].v1, cons[i].v2);
                break;
                //case ConstraintsType::st_parallel:
            case st_parallel:
                fout << generateLineParallel(cons[i].u1, cons[i].u2, cons[i].v1, cons[i].v2);
                break;
                //case ConstraintsType::st_tangent:
            case st_tangent:
                fout << generateLineTangent(cons[i].u1, cons[i].u2, cons[i].v1, cons[i].v2);
                break;
            default:
                break;
		}
		fout << "\n";
	}
	for (int i = 0; i < coplanes.size(); ++ i)
	{
		for (int j = 0; j < coplanarPoints[i].size(); ++ j)
		{
			fout << "+ 10 * "
				 << generateCoplanar(i, coplanarPoints[i][j])
				 << "\n";
		}
	}
	/*
	for (int i = 0; i < net->symmLines.size(); ++ i)
	{
		for (int j = 0; j < net->symmLines[i].size(); ++ j)
		{
			fout << "+"
				 << generateSymmetryLine(i, net->symmLines[i][j])
				 << "\n";
		}
	}
	for (int i = 0; i < net->symmPoints.size(); ++ i)
	{
		if (net->symmPoints[i].size() == 0) continue;
		for (int j = 0; j < net->symmPoints[i].size(); ++ j)
		{
			fout << "+" << generateSelfSymmPoint(i, j) << "\n";
		}
	}
	*/
	fout << ";\n\n";


	//constraints
	for (int i = 0; i < numCons; ++ i)
	{
		fout << "subject to constraint" << i << ": ";
		switch(cons[i].type)
		{
            //case ConstraintsType::st_collinear:
            case st_collinear:
                fout << generateLineCollinear(cons[i].u1, cons[i].u2, cons[i].v1, cons[i].v2);
                break;
                //case ConstraintsType::st_coplanar:
            case st_coplanar:
                fout << generateLineCoplanar(cons[i].u1, cons[i].u2, cons[i].v1, cons[i].v2);
                break;
                //case ConstraintsType::st_ortho:
            case st_ortho:
                fout << generateLineOrtho(cons[i].u1, cons[i].u2, cons[i].v1, cons[i].v2);
                break;
                //case ConstraintsType::st_parallel:
            case st_parallel:
                fout << generateLineParallel(cons[i].u1, cons[i].u2, cons[i].v1, cons[i].v2);
                break;
                //case ConstraintsType::st_tangent:
            case st_tangent:
                fout << generateLineTangent(cons[i].u1, cons[i].u2, cons[i].v1, cons[i].v2);
                break;
            default:
                break;
		}
		fout << " <= 0.01;\n";
	}
	for (int i = 0; i < coplanes.size(); ++ i)
	{
		for (int j = 0; j < coplanarPoints[i].size(); ++ j)
		{
			fout << "subject to coplanar" << i << "_" << j << ": "
				 << generateCoplanar(i, coplanarPoints[i][j])
				 << " <= 0.01;\n";
		}
	}
    /*
	for (int i = 0; i < net->symmLines.size(); ++ i)
	{
		for (int j = 0; j < net->symmLines[i].size(); ++ j)
		{
			fout << "subject to symmetry" << i << "_" << j << ": "
				 << generateSymmetryLine(i, net->symmLines[i][j])
				 << " <= smallBound;\n";
		}
	}
	for (int i = 0; i < net->symmPoints.size(); ++ i)
	{
		if (net->symmPoints[i].size() == 0) continue;
		fout << "subject to symmetryPoint" << i << ": ";
		for (int j = 0; j < net->symmPoints[i].size(); ++ j)
		{
			fout << "\n" << generateSelfSymmPoint(i, j) << "+";
		}
		fout << "0 <= smallBound;\n";
	}
	*/
    fout.close();
}

void Optimization::generateRUN(string file)
{
	ofstream fout(file.data());
    /*
#if defined(_WIN32)
	fout << "reset;\n"
		 << "option ampl_include 'D:\\fz\\point_cloud\\PointContour\\Release';\n"
		 << "option solver knitroampl;\n"
		 << "option knitro_options \"alg=1 bar_feasible=1 honorbnds=1 ms_enable=0 par_numthreads=4\";\n\n"
		 << "model test.mod;\n"
		 << "data test.dat;\n"
		 << "solve;\n"
		 << "printf {i in 0..N} \"%f %f %f\\n\", "
		 << "p[i, 1], p[i, 2], p[i, 3] "
		 << "> D:\\fz\\point_cloud\\PointContour\\Release\\result.out;\n";
#elif defined(__APPLE__)
    */
    fout << "reset;\n"
		 << "option ampl_include '"
         << amplIncludePath
         << "';\n"
		 << "option solver knitroampl;\n"
		 << "option knitro_options \"alg=0 bar_feasible=1 honorbnds=1 ms_enable=1 ms_maxsolves="
         << numStartPoints
         << " par_numthreads=8 ma_maxtime_real="
         << maxRealTime
         << "\";\n\n"
		 << "model test.mod;\n"
		 << "data test.dat;\n"
		 << "solve;\n"
		 << "printf \"%f\\n\", total_cost "
		 << "> "
		 << amplResultPath
		 << "result.out;\n"
		 << "printf {i in 0..N} \"%f %f %f\\n\", "
		 << "p[i, 1], p[i, 2], p[i, 3] "
		 << "> "
         << amplResultPath
         << "result.out;\n";
    //#endif
	fout.close();
}

void Optimization::generateBAT(string file)
{
	ofstream fout(file.data());
#if defined(_WIN32)
    std::string diskStr = amplExePath.substr(0 , 2) + "\n";
	fout << diskStr
		 << "cd "
         << amplExePath
         << "\n"
		 << "ampl.exe "
         << amplIncludePath
         << "test.run\n";
		 //<< "pause\n";
#elif defined(__APPLE__)
    fout << "cd "
         << amplExePath
         << "\n"
		 << "wine ampl.exe "
         << amplIncludePath
         << "test.run\n";
#endif
	fout.close();
}

void Optimization::run(CurveNet *net)
{
#if defined(_WIN32)
    string fileroot = amplIncludePath;
    generateDAT(fileroot + "test.dat");
	generateMOD(fileroot + "test.mod");
	generateRUN(fileroot + "test.run");
	generateBAT(fileroot + "test.bat");
	string cmd = fileroot + "test.bat";
	system(cmd.c_str());
#elif defined(__APPLE__)
    // timer.PushCurrentTime();
    string fileroot = amplIncludePath;
    generateDAT(fileroot + "test.dat");
	generateMOD(fileroot + "test.mod");
	generateRUN(fileroot + "test.run");
	generateBAT(fileroot + "test.sh");
    // timer.PopAndDisplayTime("\n\n***** file i/o time = %.6fs *****\n\n");
    string cmd = "chmod u+x " + fileroot + "test.sh";
    system(cmd.c_str());
    cmd = fileroot + "test.sh";
	system(cmd.c_str());
#endif
	ifstream fin(fileroot + "result.out");
    // timer.PushCurrentTime();
	double totError;
	fin >> totError;
    bool iterFlag;
	if (totError > 0.1 * vars.size()) 
	{
		// fin.close();
		wxString str("bad optimization result!\n");
        pcUtils->statusBar->SetStatusText(str);
        iterFlag = false;
		// return;
	}
    else
    {
        wxString str("good optimization result!\n");
        pcUtils->statusBar->SetStatusText(str);
        iterFlag = true;
    }
    std::vector<vec3d> varbuff;
    for (int i = 0; i < vars.size(); i++)
    {
        vec3d pos;
        fin >> pos.x >> pos.y >> pos.z;
        varbuff.push_back(pos);
    }

    printf("change curve ends in bsp...\n");
    for (int i = 0; i < net->numPolyLines; i++)
    {
        if (net->curveType[i] == -1) continue;
        int ci[2] = {0 , (int)net->bsplines[i].ctrlNodes.size() - 1};
        for (int j = 0; j < 2; j++)
        {
            int ni = net->getNodeIndex(net->bsplines[i].ctrlNodes[ci[j]]);
            int vi = getOptVarIndex(OptVariable(0 , ni));
            if (vars[vi].isVar)
            {
                net->bsplines[i].ctrlNodes[ci[j]] = varbuff[vars[vi].index];
            }
        }
    }

    printf("change nodes...\n");
    for (int i = 0; i < vars.size(); ++ i)
	{
        if (!vars[i].isVar) continue;
		if (vars[i].type == 0)
		{
			net->nodes[vars[i].ni] = varbuff[vars[i].index];
		}
		else
		{
			net->bsplines[vars[i].ni].ctrlNodes[vars[i].ci] = varbuff[vars[i].index];
		}
	}

    printf("resample polyLines...\n");
    for (int i = 0; i < net->numPolyLines; i++)
    {
        if (net->curveType[i] == -1) continue;
        /*
        if (net->bsplines[i].ctrlNodes.size() > 2 && net->bsplines[i].knots.size() == 0)
        {
            printf("!!!!! (%lu , %lu) !!!!!\n" , net->bsplines[i].ctrlNodes.size() ,
            net->bsplines[i].knots.size());
        }
        printf("curve id = %d, type = %d\n" , i , net->curveType[i]);
        for (int j = 0; j < net->bsplines[i].t.size(); j++) printf("%.6f " , net->bsplines[i].t[j]);
        printf("\n");
        for (int j = 0; j < net->bsplines[i].knots.size(); j++) printf("%.6f " , net->bsplines[i].knots[j]);
        printf("\n");
        */
        resampleBsp(net->bsplines[i] , net->polyLines[i]);
    }
	fin.close();

    // post process curve net
    net->evalCurveNetQuality();
    std::fill(net->nodesEditType.begin() , net->nodesEditType.end() , 0);
    std::fill(net->bspEditType.begin() , net->bspEditType.end() , 0);
    if (iterFlag)
    {
        /*
        for (int i = 0; i < net->numPolyLines; i++)
        {
            if (net->curveType[i] == -1) continue;
            for (int j = 0; j < net->originPolyLines.size(); j++)
            {
                net->originPolyLines[i][j] = net->polyLines[i][j];
            }
        }
        for (int i = 0; i < net->numPolyLines; i++)
        {
            if (net->curveType[i] != 3) continue;
            ConstraintDetector::coplanarTest(net->bsplines[i] , net->planes[i]);
        }
        */
    }
    // timer.PopAndDisplayTime("\n\n***** post processing time = %.6f *****\n\n");
}

string Optimization::var2str(int varIndex , int k)
{
    stringstream ss;
    if (vars[varIndex].isVar) ss << "p[" << vars[varIndex].index;
    else ss << "p_const[" << vars[varIndex].index;

    if (k == -1) ss << ",t]";
    else if (k == -2) ss << ",j]";
    else if (k == -3) ss << ",k]";
    else ss << "," << k << "]";

    return ss.str();
}

string Optimization::generateStraightLineDist()
{
    if (straightlines.size() == 0) return string("");
    stringstream ss;
    ss << "+ 1 * (";
    for (int i = 0; i < straightlines.size(); i++)
    {
        if (i > 0) ss << "+ ";
        ss << "p_weight[" << straightlines[i][1] << "] * (sum{j in 0..SPN[" << i << "]}"
           << "sum{t in Dim3}("
           << "(sum{k in Dim3}(sp[" << i << ",j,k]-" << var2str(straightlines[i][1] , -3)
           << ")*dir" << i << "[k])*dir" << i << "[t]"
           << "-(sp[" << i << ",j,t]-" << var2str(straightlines[i][1] , -1) << ")"
           << ")^2"
           << ")/SPN[" << i << "]";
        if (i < straightlines.size() - 1) ss << "\n";
    }
    ss << ")\n";
    return ss.str();
}

string Optimization::generateBsplineDist()
{
    if (bsplines.size() == 0) return string("");
    stringstream ss;
    ss << "+ 1 * (";
    for (int i = 0; i < bsplines.size(); i++)
    {
        if (i > 0) ss << "+ ";
        ss << "p_weight[" << bsplines[i][0] << "] * (sum{j in 0..BPN[" << i << "]}"
           << "sum{t in Dim3}("
           << "bp[" << i << ",j,t]-(";
        for (int k = 0; k < bsplines[i].size(); k++)
        {
            if (k > 0) ss << "+";
            ss << "coef[" << i << ",j," << k << "]*" << var2str(bsplines[i][k] , -1);
        }
        ss << ")"
           << ")^2"
           << ")/BPN[" << i << "]";
        if (i < bsplines.size() - 1) ss << "\n";
    }
    ss << ")\n";
    return ss.str();
}

string Optimization::generateLineOrtho(int u1, int u2, int v1, int v2)
{
	stringstream ss;
	ss << "abs(sum {t in Dim3}"
       << "((" << var2str(u1 , -1) << " - " << var2str(u2 , -1) << ")"
	   << " * "
       << "(" << var2str(v1 , -1) << " - " << var2str(v2 , -1) << ")))"
	   << " / "
	   << "sqrt(sum {t in Dim3}"
       << "(" << var2str(u1 , -1) << " - " << var2str(u2 , -1) << ") ^ 2)"
	   << " / "
	   << "sqrt(sum {t in Dim3}"
       << "(" << var2str(v1 , -1) << " - " << var2str(v2 , -1) << ") ^ 2)";
	return ss.str();
}

string Optimization::generateLineParallel(int u1, int u2, int v1, int v2)
{
	stringstream ss;
	ss << "abs(abs(sum {t in Dim3}"
       << "((" << var2str(u1 , -1) << " - " << var2str(u2 , -1) << ")"
	   << " * "
       << "(" << var2str(v1 , -1) << " - " << var2str(v2 , -1) << ")))"
	   << " / "
	   << "sqrt(sum {t in Dim3}"
       << "(" << var2str(u1 , -1) << " - " << var2str(u2 , -1) << ") ^ 2)"
	   << " / "
	   << "sqrt(sum {t in Dim3}"
       << "(" << var2str(v1 , -1) << " - " << var2str(v2 , -1) << ") ^ 2)"
	   << " - 1)";
	return ss.str();
}

string Optimization::generateLineCollinear(int u1, int u2, int v1, int v2)
{
	stringstream ss;
	ss << generateLineParallel(u1, u2, u1, v1);
	return ss.str();
}

string Optimization::generateLineCoplanar(int u1, int u2, int v1, int v2)
{
	stringstream ss;
	ss << "abs("
	   << "(" << "(p[" << u1 << ",2]-p[" << u2 << ",2])"
	   << "*"
	   << "(p[" << v1 << ",3]-p[" << v2 << ",3])"
	   << "-"
	   << "(p[" << u1 << ",3]-p[" << u2 << ",3])"
	   << "*"
	   << "(p[" << v1 << ",2]-p[" << v2 << ",2])" << ")"
	   << "*"
	   << "(p[" << u1 << ",1]-p[" << v1 << ",1])"
	   << "+"
	   << "(" << "(p[" << u1 << ",3]-p[" << u2 << ",3])"
	   << "*"
	   << "(p[" << v1 << ",1]-p[" << v2 << ",1])"
	   << "-"
	   << "(p[" << u1 << ",1]-p[" << u2 << ",1])"
	   << "*"
	   << "(p[" << v1 << ",3]-p[" << v2 << ",3])" << ")"
	   << "*"
	   << "(p[" << u1 << ",2]-p[" << v1 << ",2])"
	   << "+"
	   << "(" << "(p[" << u1 << ",1]-p[" << u2 << ",1])"
	   << "*"
	   << "(p[" << v1 << ",2]-p[" << v2 << ",2])"
	   << "-"
	   << "(p[" << u1 << ",2]-p[" << u2 << ",2])"
	   << "*"
	   << "(p[" << v1 << ",1]-p[" << v2 << ",1])" << ")"
	   << "*"
	   << "(p[" << u1 << ",3]-p[" << v1 << ",3])"
	   << ")"
	   << "/"
	   << "sqrt(sum{t in Dim3}"
	   << "(p[" << u1 << ",t]-p[" << v1 << ",t])^2)"
	   << "/"
	   << "sqrt("
	   << "(" << "(p[" << u1 << ",2]-p[" << u2 << ",2])"
	   << "*"
	   << "(p[" << v1 << ",3]-p[" << v2 << ",3])"
	   << "-"
	   << "(p[" << u1 << ",3]-p[" << u2 << ",3])"
	   << "*"
	   << "(p[" << v1 << ",2]-p[" << v2 << ",2])" << ")" << "^2"
	   << "+"
	   << "(" << "(p[" << u1 << ",3]-p[" << u2 << ",3])"
	   << "*"
	   << "(p[" << v1 << ",1]-p[" << v2 << ",1])"
	   << "-"
	   << "(p[" << u1 << ",1]-p[" << u2 << ",1])"
	   << "*"
	   << "(p[" << v1 << ",3]-p[" << v2 << ",3])" << ")" << "^2"
	   << "+"
	   << "(" << "(p[" << u1 << ",1]-p[" << u2 << ",1])"
	   << "*"
	   << "(p[" << v1 << ",2]-p[" << v2 << ",2])"
	   << "-"
	   << "(p[" << u1 << ",2]-p[" << u2 << ",2])"
	   << "*"
	   << "(p[" << v1 << ",1]-p[" << v2 << ",1])" << ")" << "^2"
	   << ")";

	return ss.str();
}

string Optimization::generateLineTangent(int u1, int u2, int v1, int v2)
{
	stringstream ss;
	ss << generateLineParallel(u1, u2, v1, v2)
	   << " + "
	   << generateLineParallel(u1, u2, u2, v2);
	return ss.str();
}

string Optimization::generateCoplanar(int plane, int point)
{
	stringstream ss;
	ss << "abs((sum{j in Dim3} (plane[" << plane << ", j] *"
       << var2str(point , -2) << "))"
       << " + plane[" << plane << ", 4])";
    //<< " + plane[" << plane << ", 4]) / "
    //<< "sqrt(sum{i in Dim3} (plane[" << plane << ", i] ^ 2))";
	return ss.str();
}

string Optimization::generateSymmetryLine(int plane, std::pair<int, int> linepair)
{
	stringstream ss;
	if (net->curveType[linepair.first] == 1)
	{
		int x1 = featurelines[linepair.first][0];
		int x2 = featurelines[linepair.first][1];
		int x3 = featurelines[linepair.second][0];
		int x4 = featurelines[linepair.second][1];
		vec3d v1 = net->nodes[vars[x1].ni];
		vec3d v2 = net->nodes[vars[x2].ni];
		vec3d v3 = net->nodes[vars[x3].ni];
		vec3d v4 = net->nodes[vars[x4].ni];
		Plane p = net->conSet->symmetricPlanes[plane];
		Plane tmp_p((v1 + v3) / 2, v1 - v3);
		if (p.dist(tmp_p) > ConstraintDetector::symmetryThr)
		{
			swap(v3, v4);
			swap(x3, x4);
		}
		ss << generateSymmetryPoint(plane, x1, x3)
		   << "+"
		   << generateSymmetryPoint(plane, x2, x4);
	}
	else
	{
		for (int i = 0; i < net->bsplines[linepair.first].ctrlNodes.size(); ++ i)
		{
			int x = featurelines[linepair.first][i];
			int y = featurelines[linepair.second][i];
			ss << generateSymmetryPoint(plane, x, y) << "+";
		}
		ss << "0";
	}
	return ss.str();
}

string Optimization::generateSymmetryPoint(int plane, int u, int v)
{
	stringstream ss;
	ss << "(sum{i in Dim3} (p[" << u << ", i] - "
	   << "(p[" << v << ", i] - 2*symmetric_plane[" << plane << ",4]*symmetric_plane[" << plane << ",i] - "
	   << "2*(sum{j in Dim3}symmetric_plane[" << plane << ",j]*p[" << v << ",j])*symmetric_plane[" << plane << ",i])"
	   << ") ^ 2)";
	return ss.str();
}

string Optimization::generateSelfSymmPoint(int plane, int n)
{
	int c = net->conSet->symmPoints[plane][n].n;
	int l = net->conSet->symmPoints[plane][n].n1;
	int r = net->conSet->symmPoints[plane][n].n2;
	c = curveIdx[c];
	stringstream ss;
	ss << "(sum{i in Dim3} ((sum{k in 0..CN[" << c << "]}coef[" << c << "," << l << ",k]*p[bidx[" << c << ",k],i]) - "
	   << "((sum{k in 0..CN[" << c << "]}coef[" << c << "," << r << ",k]*p[bidx[" << c << ",k],i])"
	   << "- 2*symmetric_plane[" << plane << ",4]*symmetric_plane[" << plane << ",i] - "
	   << "2*(sum{j in Dim3}symmetric_plane[" << plane << ",j]*"
	   << "(sum{k in 0..CN[" << c << "]}coef[" << c << "," << r << ",k]*p[bidx[" << c << ",k],i])"
	   << ")*symmetric_plane[" << plane << ",i])"
	   << ") ^ 2)";
	return ss.str();
}

void Optimization::addCoplanar(int p, int q, int r, int s)
{
	vec3d pos[4];
	int varNum[4] = {p, q, r, s};
	if (vars[p].type == 0) pos[0] = net->nodes[vars[p].ni];
	else pos[0] = net->bsplines[vars[p].ni].ctrlNodes[vars[p].ci];
	if (vars[q].type == 0) pos[1] = net->nodes[vars[q].ni];
	else pos[1] = net->bsplines[vars[q].ni].ctrlNodes[vars[q].ci];
	if (vars[r].type == 0) pos[2] = net->nodes[vars[r].ni];
	else pos[2] = net->bsplines[vars[r].ni].ctrlNodes[vars[r].ci];
	if (vars[s].type == 0) pos[3] = net->nodes[vars[s].ni];
	else pos[3] = net->bsplines[vars[s].ni].ctrlNodes[vars[s].ci];
	vec3d x = (pos[0] - pos[1]) , y = (pos[2] - pos[3]);
	x.normalize(); y.normalize();
    vec3d n = x.cross(y);
    n.normalize();
	// parallel
	if (ConstraintDetector::checkParallel(pos[0] , pos[1] , pos[2] , pos[3] ,
		ConstraintDetector::parallelThr))
		return;

    double weight = (pos[0]-pos[1]).length()*(pos[2]-pos[3]).length();
        //*weightBetweenSegs(pos[0] , pos[1] , pos[2] , pos[3]);
	Plane plane((pos[0]+pos[1]+pos[2]+pos[3])*0.25, n, weight);
    /*
    printf("------------------\n");
	printf("plane: p = (%.6f,%.6f,%.6f), n = (%.6f,%.6f,%.6f) , w = %.6f\n" ,
		plane.p.x , plane.p.y , plane.p.z ,
		plane.n.x , plane.n.y , plane.n.z , weight);
    */
	//net->coplanes.push_back(plane);
	bool exist = false;
	double planeDistThr = 0.05;
	for (int i = 0; i < coplanes.size(); ++ i)
	{
        if (coplanes[i].dist(plane) < planeDistThr)
		{
			exist = true;
			coplanes[i].add(plane);
			for (int k = 0; k < 4; ++ k)
			{
				bool existP = false;
				for (int j = 0; j < coplanarPoints[i].size(); ++ j)
				{
					if (coplanarPoints[i][j] == varNum[k])
					{
						existP = true;
						break;
					}
				}
				if (!existP) coplanarPoints[i].push_back(varNum[k]);
			}
			break;
		}
	}
	if (!exist)
	{
		coplanes.push_back(plane);
		vector<int> tmpPoints;
		for (int i = 0; i < 4; ++ i)
		{
			bool existP = false;
			for (int j = 0; j < tmpPoints.size(); ++ j)
			{
				if (tmpPoints[j] == varNum[i])
				{
					existP = true;
					break;
				}
			}
			if (!existP) tmpPoints.push_back(varNum[i]);
		}
		coplanarPoints.push_back(tmpPoints);
	}
}

void Optimization::addCoplanar(int varIndex , Plane& plane)
{
    bool exist = false;
    double planeDistThr = 0.05;
	for (int i = 0; i < coplanes.size(); i++)
	{
        if (coplanes[i].dist(plane) < planeDistThr)
		{
			exist = true;
			coplanes[i].add(plane);
			bool existP = false;
			for (int j = 0; j < coplanarPoints[i].size(); ++ j)
			{
				if (coplanarPoints[i][j] == varIndex)
                {
                    existP = true;
                    break;
                }
            }
            if (!existP) coplanarPoints[i].push_back(varIndex);
			break;
		}
	}
	if (!exist)
	{
		coplanes.push_back(plane);
		vector<int> tmpPoints;
		tmpPoints.push_back(varIndex);
		coplanarPoints.push_back(tmpPoints);
	}
}

void Optimization::addOptVariable(OptVariable optVar)
{
    double hashVal = var2double(optVar);
    if (double2vi.find(hashVal) != double2vi.end()) return;
    double2vi[hashVal] = numVars;
    vars.push_back(optVar);
    numVars++;
}

int Optimization::getOptVarIndex(const OptVariable& optVar)
{
    double hashVal = var2double(optVar);
    if (double2vi.find(hashVal) == double2vi.end())
    {
        return -1;
    }
    else
    {
        return double2vi[hashVal];
    }
}

double Optimization::var2double(const OptVariable& v)
{
    return v.type * pow2[2] + v.ni * pow2[1] + v.ci;
}

std::pair<OptVariable , OptVariable> Optimization::bsp2var(int bspIndex , int curveIndex , int numCtrlCurves)
{
    OptVariable u1 , u2;
    if (net->curveType[bspIndex] == -1)
    {
        printf("bsp2var error, bspIndex = %d\n" , bspIndex);
    }
    if (curveIndex == 0 && curveIndex == numCtrlCurves - 1)
    {
        int ni = net->getNodeIndex(net->bsplines[bspIndex].ctrlNodes[curveIndex]);
        u1 = OptVariable(0 , ni);
        ni = net->getNodeIndex(net->bsplines[bspIndex].ctrlNodes[curveIndex + 1]);
        u2 = OptVariable(0 , ni);
    }
    else if (curveIndex == 0)
    {
        int ni = net->getNodeIndex(net->bsplines[bspIndex].ctrlNodes[curveIndex]);
        u1 = OptVariable(0 , ni);
        u2 = OptVariable(1 , bspIndex , curveIndex + 1);
    }
    else if (curveIndex == numCtrlCurves - 1)
    {
        u1 = OptVariable(1 , bspIndex , curveIndex);
        int ni = net->getNodeIndex(net->bsplines[bspIndex].ctrlNodes[curveIndex + 1]);
        u2 = OptVariable(0 , ni);
    }
    else
    {
        u1 = OptVariable(1 , bspIndex , curveIndex);
        u2 = OptVariable(1 , bspIndex , curveIndex + 1);
    }
    return std::make_pair(u1 , u2);
}

bool Optimization::isAllConst(int u1 , int u2 , int v1 , int v2)
{
    return (!vars[u1].isVar) && (!vars[u2].isVar) && (!vars[v1].isVar) && (!vars[v2].isVar);
}