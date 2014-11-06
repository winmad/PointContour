#include <fstream>
#include <sstream>
#include <cstdlib>
#include "optimization.h"

using namespace std;

Optimization::Optimization()
{
    net = NULL;
}

void Optimization::init(CurveNet *_net)
{
    net = _net;

    numVars = 0;
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

    for (int i = 0; i < net->numNodes; i++)
    {
        if (!net->nodesStat[i]) continue;
        addOptVariable(OptVariable(0 , i));
    }
    for (int i = 0; i < net->numPolyLines; i++)
    {
        if (net->bsplines[i].ctrlNodes.size() <= 2) continue;
        for (int j = 1; j < (int)net->bsplines[i].ctrlNodes.size() - 1; j++)
        {
            addOptVariable(OptVariable(1 , i , j));
        }
    }

	//find var index for lines
	for (int i = 0; i < numVars; ++ i) lastDraw.push_back(false);
	for (int i = 0; i < net->numPolyLines; ++ i)
	{
		vector<int> tmpvec;
		if (net->curveType[i] == -1) continue;
		if (net->curveType[i] == 1)
		{
            std::pair<OptVariable , OptVariable> vars = bsp2var(i , 0 , 1);
			int tmpidx = getOptVarIndex(vars.first);
			tmpvec.push_back(tmpidx);
			lastDraw[tmpidx] = true;
			tmpidx = getOptVarIndex(vars.second);
			tmpvec.push_back(tmpidx);
			lastDraw[tmpidx] = true;
			straightlines.push_back(tmpvec);
			straightlinesIdx.push_back(i);
		}
		else
		{
            int numCtrlCurves = (int)net->bsplines[i].ctrlNodes.size() - 1;
            std::pair<OptVariable , OptVariable> vars = bsp2var(i , 0 , numCtrlCurves);
			int tmpidx = getOptVarIndex(vars.first);
			tmpvec.push_back(tmpidx);
			lastDraw[tmpidx] = true;
			for (int j = 1; j < (int)net->bsplines[i].ctrlNodes.size() - 1; ++ j)
			{
				tmpidx = getOptVarIndex(OptVariable(1, i, j));
				tmpvec.push_back(tmpidx);
				lastDraw[tmpidx] = true;
			}
            vars = bsp2var(i , numCtrlCurves - 1 , numCtrlCurves);
			tmpidx = getOptVarIndex(vars.second);
			tmpvec.push_back(tmpidx);
			lastDraw[tmpidx] = true;
			bsplines.push_back(tmpvec);
			bsplinesIdx.push_back(i);
		}
	}

	//constraint
    for (int i = 0; i < net->numPolyLines; i++)
    {
        if (net->curveType[i] != 1) continue;

        PolyLineIndex u , v;
        int u1 , u2 , v1 , v2;

        // collinear
        std::pair<int , int> root = net->collinearSet.find(i , 0);
        if (root.first != i)
        {
            u = net->polyLinesIndex[i];
            v = net->polyLinesIndex[root.first];
            u1 = getOptVarIndex(OptVariable(0 , u.ni[0]));
            u2 = getOptVarIndex(OptVariable(0 , u.ni[1]));
            v1 = getOptVarIndex(OptVariable(0 , v.ni[0]));
            v2 = getOptVarIndex(OptVariable(0 , v.ni[1]));
            cons.push_back(OptConstraints(u1 , u2 , v1 , v2 , st_collinear));
        }
        // parallel
        root = net->parallelSet.find(i , 0);
        if (root.first != i)
        {
            u = net->polyLinesIndex[i];
            v = net->polyLinesIndex[root.first];
            u1 = getOptVarIndex(OptVariable(0 , u.ni[0]));
            u2 = getOptVarIndex(OptVariable(0 , u.ni[1]));
            v1 = getOptVarIndex(OptVariable(0 , v.ni[0]));
            v2 = getOptVarIndex(OptVariable(0 , v.ni[1]));
            cons.push_back(OptConstraints(u1 , u2 , v1 , v2 , st_parallel));
        }
    }

    // coplanar
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
                    if (u1 == v1 || u1 == v2 || u2 == v1 || u2 == v2) continue;
                    addCoplanar(u1, u2, v1, v2);
                }
            }
        }
    }
    
    for (int i = 0; i < net->numPolyLines; i++)
    {
        if (net->curveType[i] == 2 || net->curveType[i] == -1) continue;
        for (int j = i + 1; j < net->numPolyLines; j++)
        {
            if (net->curveType[j] == 2 || net->curveType[j] == -1) continue;
            if (net->coplanarSet.getMark(i , 0 , j , 0) == 1)
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

    // ortho & tangent
    for (int i = 0; i < net->numPolyLines; i++)
    {
        if (net->curveType[i] == -1) continue;
        for (int j = i; j < net->numPolyLines; j++)
        {
            if (net->curveType[j] == -1) continue;
            for (int x = 0; x < (int)net->bsplines[i].ctrlNodes.size() - 1; x++)
            {
                for (int y = (i != j ? 0 : x + 1); y < (int)net->bsplines[j].ctrlNodes.size() - 1; y++)
                {
                    int mark = net->orthoSet.getMark(i , x , j , y);
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
                        if (mark == 1)
                        {
                            cons.push_back(OptConstraints(u1 , u2 , v1 , v2 , st_ortho));
                        }
                        else if (mark == 2)
                        {
                            cons.push_back(OptConstraints(u1 , u2 , v1 , v2 , st_tangent));
                        }
                    }
                }
            }
        }
    }
    numCons = cons.size();

    /*
    // print vars
    writeLog("===== vars =====\n");
    for (int i = 0; i < numVars; i++)
    {
        writeLog("var %d: (%d , %d , %d)\n" , i , vars[i].type , vars[i].ni , vars[i].ci);
    }
    // print constraints
    writeLog("===== cons =====\n");
    for (int i = 0; i < numCons; i++)
    {
        writeLog("cons %d: type = %d, (%d , %d) <-> (%d , %d)\n" , i , (int)cons[i].type ,
            cons[i].u1 , cons[i].u2 , cons[i].v1 , cons[i].v2);
    }
    */
}

bool samepoint(vec3d &a, vec3d &b)
{
	return abs(a.x - b.x) < EPS && abs(a.y - b.y) < EPS && abs(a.z - b.z) < EPS;
}

bool Optimization::isLinked(int i, int j, int p, int q)
{
	if (samepoint(net->bsplines[i].ctrlNodes[j], net->bsplines[p].ctrlNodes[q]))
		return true;
	if (samepoint(net->bsplines[i].ctrlNodes[j], net->bsplines[p].ctrlNodes[q+1]))
		return true;
	if (samepoint(net->bsplines[i].ctrlNodes[j+1], net->bsplines[p].ctrlNodes[q]))
		return true;
	if (samepoint(net->bsplines[i].ctrlNodes[j+1], net->bsplines[p].ctrlNodes[q+1]))
		return true;
	return false;
}

void Optimization::generateDAT(string file)
{
	ofstream fout(file.data());
	fout << "param N := " << numVars - 1 << ";\n";
	
	fout << "param init_p :=\n";
	for (int i = 0; i < numVars; ++ i)
	{
		fout << "\t[" << i << ", *]";
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
		fout << " " << i << " ";
		if (lastDraw[i]) fout << 0.05 << "\n";
		else fout << 0.01 << "\n";
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
	fout << "param N;\n";
	fout << "param init_p {0..N, Dim3};\n";
	fout << "param p_bound {0..N};\n";
	fout << "param PN;\n";
	fout << "param init_plane {0..PN, Dim4};\n";
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
	
	fout << "\n# variables\n";
	fout << "var p {i in 0..N, t in Dim3} >= init_p[i, t] - 0.05, <= init_p[i, t] + 0.05, := init_p[i, t];\n";
	fout << "var plane {i in 0..PN, t in Dim4} >= init_plane[i, t] - 0.05, <= init_plane[i, t] + 0.05, := init_plane[i, t];\n";

	fout << "\n# intermediate variables\n";
	fout << "var dir {i in 0..SN, t in Dim3} = (p[sidx[i, 1], t] - p[sidx[i, 2], t])"
		 << " / sqrt(sum{j in Dim3} (p[sidx[i, 1], j] - p[sidx[i, 2], j]) ^ 2);\n";

	fout << "\n# objective\n";
	fout << "minimize total_cost:\n";
	fout << "100 * (sum {i in 0..N, t in Dim3} "
		 << "(p[i, t] - init_p[i, t]) ^ 2)\n";
	fout << "+100 * (sum{i in 0..SN}(sum{j in 0..SPN[i]}"
		 << "sum{t in Dim3}("
		 << "(sum{k in Dim3}(sp[i,j,k]-p[sidx[i,2],k])*dir[i,k])*dir[i,t]"
		 << "-(sp[i,j,t]-p[sidx[i,2],t])"
		 << ")^2"
		 << ")/SPN[i])\n";
	fout << "+100 * (sum{i in 0..BN}(sum{j in 0..BPN[i]}"
		 << "sum{t in Dim3}("
		 << "bp[i,j,t]-sum{k in 0..CN[i]}coef[i,j,k]*p[bidx[i,k],t]"
		 << ")^2"
		 << ")/BPN[i])\n";

	for (int i = 0; i < numCons; ++ i)
	{
		fout << "+";
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
		fout << "\n";
	}
	for (int i = 0; i < coplanes.size(); ++ i)
	{
		for (int j = 0; j < coplanarPoints[i].size(); ++ j)
		{
			fout << "+"
				 << generateCoplanar(i, coplanarPoints[i][j])
				 << "\n";
		}
	}
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
		fout << " <= 0.05;\n";
	}
	for (int i = 0; i < coplanes.size(); ++ i)
	{
		for (int j = 0; j < coplanarPoints[i].size(); ++ j)
		{
			fout << "subject to coplanar" << i << j << ": "
				 << generateCoplanar(i, coplanarPoints[i][j])
				 << " <= 0.05;\n";
		}
	}
    fout.close();
}

void Optimization::generateRUN(string file)
{
	ofstream fout(file.data());
#if defined(_WIN32)
	fout << "reset;\n"
		 << "option ampl_include 'E:\\reconstruction\\point_cloud\\PointContour\\Release';\n"
		 << "option solver knitroampl;\n"
		 << "option knitro_options \"alg=1 bar_feasible=1 honorbnds=1 ms_enable=0 par_numthreads=4\";\n\n"
		 << "model test.mod;\n"
		 << "data test.dat;\n"
		 << "solve;\n"
		 << "printf {i in 0..N} \"%f %f %f\\n\", "
		 << "p[i, 1], p[i, 2], p[i, 3] "
		 << "> E:\\reconstruction\\point_cloud\\PointContour\\Release\\result.out;\n";
#elif defined(__APPLE__)
    fout << "reset;\n"
		 << "option ampl_include '/Users/Winmad/Projects/PointContour/ampl';\n"
		 << "option solver knitroampl;\n"
		 << "option knitro_options \"alg=0 bar_feasible=1 honorbnds=1 ms_enable=1 ms_maxsolves=50 par_numthreads=6 ma_maxtime_real=20\";\n\n"
		 << "model test.mod;\n"
		 << "data test.dat;\n"
		 << "solve;\n"
		 << "printf {i in 0..N} \"%f %f %f\\n\", "
		 << "p[i, 1], p[i, 2], p[i, 3] "
		 << "> /Users/Winmad/Projects/PointContour/ampl/result.out;\n";
#endif
	fout.close();
}

void Optimization::generateBAT(string file)
{
	ofstream fout(file.data());
#if defined(_WIN32)
	fout << "E:\n"
		 << "cd E:\\reconstruction\\AMPLcml\n"
		 << "ampl.exe E:\\reconstruction\\point_cloud\\PointContour\\Release\\test.run\n";
		 //<< "pause\n";
#elif defined(__APPLE__)
    fout << "cd /Users/Winmad/AMPL_win\n"
		 << "wine ampl.exe /Users/Winmad/Projects/PointContour/ampl/test.run\n";
#endif
	fout.close();
}

void Optimization::run(CurveNet *net)
{

#if defined(_WIN32)
	generateDAT("test.dat");
	generateMOD("test.mod");
	generateRUN("test.run");
	generateBAT("test.bat");
	system("test.bat");
	ifstream fin("result.out");
#elif defined(__APPLE__)
    timer.PushCurrentTime();
    string fileroot = "/Users/Winmad/Projects/PointContour/ampl/";
    generateDAT(fileroot + "test.dat");
	generateMOD(fileroot + "test.mod");
	generateRUN(fileroot + "test.run");
	generateBAT(fileroot + "test.sh");
    timer.PopAndDisplayTime("\n\n***** file i/o time = %.6fs *****\n\n");
    string cmd = "chmod u+x " + fileroot + "test.sh";
    system(cmd.c_str());
    cmd = fileroot + "test.sh";
	system(cmd.c_str());
	ifstream fin(fileroot + "result.out");
#endif

    timer.PushCurrentTime();
    std::vector<vec3d> varbuff;
    for (int i = 0; i < vars.size(); i++)
    {
        vec3d pos;
        fin >> pos.x >> pos.y >> pos.z;
        varbuff.push_back(pos);
    }

    for (int i = 0; i < net->numPolyLines; i++)
    {
        if (net->curveType[i] == -1) continue;
        int ci[2] = {0 , net->bsplines[i].ctrlNodes.size() - 1};
        for (int j = 0; j < 2; j++)
        {
            int ni = net->getNodeIndex(net->bsplines[i].ctrlNodes[ci[j]]);
            int vi = getOptVarIndex(OptVariable(0 , ni));
            net->bsplines[i].ctrlNodes[ci[j]] = varbuff[vi];
        }
    }

    for (int i = 0; i < vars.size(); ++ i)
	{
		if (vars[i].type == 0)
		{
			net->nodes[vars[i].ni] = varbuff[i];
		}
		else
		{
			net->bsplines[vars[i].ni].ctrlNodes[vars[i].ci] = varbuff[i];
		}
	}

    for (int i = 0; i < net->numPolyLines; i++)
    {
        if (net->curveType[i] == -1) continue;
        if (net->bsplines[i].ctrlNodes.size() > 2 && net->bsplines[i].knots.size() == 0)
        {
            printf("!!!!! (%lu , %lu) !!!!!\n" , net->bsplines[i].ctrlNodes.size() ,
            net->bsplines[i].knots.size());
        }
        resampleBsp(net->bsplines[i] , net->polyLines[i]);
    }
	fin.close();
    timer.PopAndDisplayTime("\n\n***** post processing time = %.6f *****\n\n");
}

string Optimization::generateSamePoint(int u, int v)
{
	stringstream ss;
	ss << "sum {t in Dim3} "
	   << "abs(p[" << u << ",t] - p[" << v << ",t])";
	return ss.str();
}

string Optimization::generateLineOrtho(int u1, int u2, int v1, int v2)
{
	stringstream ss;
	ss << "abs(sum {t in Dim3}"
	   << "((p[" << u1 << ",t] - p[" << u2 << ",t])"
	   << " * "
	   << "(p[" << v1 << ",t] - p[" << v2 << ",t])))"
	   << " / "
	   << "sqrt(sum {t in Dim3}"
	   << "(p[" << u1 << ",t] - p[" << u2 << ",t]) ^ 2)"
	   << " / "
	   << "sqrt(sum {t in Dim3}"
	   << "(p[" << v1 << ",t] - p[" << v2 << ",t]) ^ 2)";
	return ss.str();
}

string Optimization::generateLineParallel(int u1, int u2, int v1, int v2)
{
	stringstream ss;
	ss << "abs(abs(sum {t in Dim3}"
	   << "((p[" << u1 << ",t] - p[" << u2 << ",t])"
	   << " * "
	   << "(p[" << v1 << ",t] - p[" << v2 << ",t])))"
	   << " / "
	   << "sqrt(sum {t in Dim3}"
	   << "(p[" << u1 << ",t] - p[" << u2 << ",t]) ^ 2)"
	   << " / "
	   << "sqrt(sum {t in Dim3}"
	   << "(p[" << v1 << ",t] - p[" << v2 << ",t]) ^ 2)"
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
	   << generateLineParallel(u1, u2, u1, v1);
	return ss.str();
}

string Optimization::generateCoplanar(int plane, int point)
{
	stringstream ss;
	ss << "abs((sum{i in Dim3} (plane[" << plane << ", i] * p[" << point << ", i]))"
	   << " + plane[" << plane << ", 4]) / "
	   << "sqrt(sum{i in Dim3} (plane[" << plane << ", i] ^ 2))";
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
	if (vars[p].type == 0) pos[3] = net->nodes[vars[s].ni];
	else pos[3] = net->bsplines[vars[s].ni].ctrlNodes[vars[s].ci];
    vec3d n = (pos[0]-pos[1]).cross(pos[0]-pos[2]);
	Plane plane(pos[0], n);
	bool exist = false;
	double coplanarThr = 0.1;
	for (int i = 0; i < coplanes.size(); ++ i)
	{
		if (coplanes[i].dist(plane) < coplanarThr)
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