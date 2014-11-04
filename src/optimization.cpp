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

    numCons = 0;
    cons.clear();

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
            addOptVariable(OptVariable(1 , i  , j));
        }
    }
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
                    cons.push_back(OptConstraints(u1 , u2 , v1 , v2 , st_coplanar));
                }
            }
        }
    }

    for (int i = 0; i < net->numPolyLines; i++)
    {
        if (net->curveType[i] == 2 || net->curveType[i] == -1) continue;
        for (int j = i + 1; j < net->numPolyLines; j++)
        {
            if (net->curveType[j] == 2 || net->curveType[i] == -1) continue;
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
                        cons.push_back(OptConstraints(u1 , u2 , v1 , v2 , st_coplanar));
                    }
                }
            }
        }
    }

    // ortho & tangent
    for (int i = 0; i < net->numPolyLines; i++)
    {
        if (net->curveType[i] == -1) continue;
        for (int j = i; j < net->numPolyLines; j++)
        {
            if (net->curveType[i] == -1) continue;
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
	fout.close();
}

void Optimization::generateMOD(string file)
{
	ofstream fout(file.data());
	/*vector<OptConstraints> consCollinear;
	vector<OptConstraints> consCoplanar;
	vector<OptConstraints> consParallel;
	vector<OptConstraints> consOrtho;
	vector<OptConstraints> consTangent;
	for (int i = 0; i < cons.size(); ++ i)
	{
		switch(cons[i].type)
		{
		case ConstraintsType::st_collinear:
			consCollinear.push_back(cons[i]);
			break;
		case ConstraintsType::st_coplanar:
			consCoplanar.push_back(cons[i]);
			break;
		case ConstraintsType::st_ortho:
			consOrtho.push_back(cons[i]);
			break;
		case ConstraintsType::st_parallel:
			consParallel.push_back(cons[i]);
			break;
		case ConstraintsType::st_tangent:
			consTangent.push_back(cons[i]);
			break;
		default:
			break;
		}
	}*/
	fout << "# parameters\n";
	fout << "set Dim3 = 1..3;\n";
	fout << "param N;\n";
	fout << "param init_p {i in 0..N, Dim3};\n";
	
	fout << "\n# variables\n";
	fout << "var p {i in 0..N, t in Dim3} >= init_p[i, t] - 0.1, <= init_p[i, t] + 0.1, := init_p[i, t] - 0.1;\n";

	fout << "\n# intermediate variables\n";

	fout << "\n# objective\n";
	fout << "minimize total_cost:\n";
	fout << "100 * (sum {i in 0..N, t in Dim3} "
		 << "(p[i, t] - init_p[i, t]) ^ 2)\n";
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
	fout << ";\n\n";

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
		fout << " <= 0.005;\n";
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
		 << "option knitro_options \"alg=0 bar_feasible=1 honorbnds=1 ms_enable=1 ms_maxsolves=20 par_numthreads=8\";\n\n"
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
    string fileroot = "/Users/Winmad/Projects/PointContour/ampl/";
    generateDAT(fileroot + "test.dat");
	generateMOD(fileroot + "test.mod");
	generateRUN(fileroot + "test.run");
	generateBAT(fileroot + "test.sh");
    string cmd = "chmod u+x " + fileroot + "test.sh";
    system(cmd.c_str());
    cmd = fileroot + "test.sh";
	system(cmd.c_str());
	ifstream fin(fileroot + "result.out");
#endif

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
        resampleBsp(net->bsplines[i] , net->polyLines[i]);
    }
	fin.close();
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