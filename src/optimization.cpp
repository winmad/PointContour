#include <fstream>
#include <sstream>
#include <cstdlib>
#include "optimization.h"

using namespace std;

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
	fout << "param BN := " << (int)net->bsplines.size() - 1 << ";\n";
	fout << "param CN :=\n";
	for (int i = 0; i < net->bsplines.size(); ++ i)
	{
		fout << '\t' << i << ' ' << (int)net->bsplines[i].ctrlNodes.size() - 1 << endl;
	}
	fout << ";\n";
	
	fout << "param init_p :=\n";
	for (int i = 0; i < net->bsplines.size(); ++ i)
	{
		for (int j = 0; j < net->bsplines[i].ctrlNodes.size(); ++ j)
		{
			fout << "\t[" << i << ", " << j << ", *]";
			fout << " 1 " << net->bsplines[i].ctrlNodes[j].x
				 << " 2 " << net->bsplines[i].ctrlNodes[j].y
				 << " 3 " << net->bsplines[i].ctrlNodes[j].z
				 << endl;
		}
	}
	fout << ";\n";
	fout.close();
}

void Optimization::generateMOD(string file)
{
	ofstream fout(file.data());
	fout << "# parameters\n";
	fout << "set Dim3 = 1..3;\n";
	fout << "param BN;\n";
	fout << "param CN {0..BN};\n";
	fout << "param init_p {i in 0..BN, 0..CN[i], Dim3};\n";
	
	fout << "\n# variables\n";
	fout << "var p {i in 0..BN, j in 0..CN[i], t in Dim3} >= init_p[i, j, t] - 0.1, <= init_p[i, j, t] + 0.1, := init_p[i, j, t] - 0.1;\n";

	fout << "\n# intermediate variables\n";

	fout << "\n# objective\n";
	fout << "minimize total_cost:\n";
	fout << "(sum {i in 0..BN, j in 0..CN[i], t in Dim3} "
		 << "(p[i, j, t] - init_p[i, j, t]) ^ 2)\n";
	//ortho
	/*for (int i = 0; i < net->bsplines.size(); ++ i)
	{
		if (net->bsplines[i].ctrlNodes.size() != 2) continue;
		for (int j = i + 1; j < net->bsplines.size(); ++ j)
		{
			if (net->bsplines[i].ctrlNodes.size() != 2) continue;
			if (net->orthoSet.getMark(i, 0, j, 0) == 1)
			{
				fout << "+abs(" 
					 << generateLineOrtho(i, 0, j, 0)
					 << ")\n";
			}
		}
	}*/
	fout << ";\n\n";

	int num;
	//same point
	num = 0;
	for (int i = 0; i < net->bsplines.size(); ++ i)
	{
		if (net->bsplines[i].ctrlNodes.size() < 2) continue;
		vec3d &si = net->bsplines[i].ctrlNodes[0];
		vec3d &ei = net->bsplines[i].ctrlNodes[net->bsplines[i].ctrlNodes.size() - 1];
		for (int j = i + 1; j < net->bsplines.size(); ++ j)
		{
			if (net->bsplines[j].ctrlNodes.size() < 2) continue;
			vec3d &sj = net->bsplines[j].ctrlNodes[0];
			vec3d &ej = net->bsplines[j].ctrlNodes[net->bsplines[j].ctrlNodes.size() - 1];
			if (samepoint(si, sj))
			{
				++ num;
				fout << "subject to samePoint" << num << ": "
					 << generateSamePoint(i, 0, j, 0)
					 << " = 0;\n";
				continue;
			}
			if (samepoint(si, ej))
			{
				++ num;
				fout << "subject to samePoint" << num << ": "
					 << generateSamePoint(i, 0, j, net->bsplines[j].ctrlNodes.size() - 1)
					 << " = 0;\n";
				continue;
			}
			if (samepoint(ei, sj))
			{
				++ num;
				fout << "subject to samePoint" << num << ": "
					 << generateSamePoint(i, net->bsplines[i].ctrlNodes.size() - 1, j, 0)
					 << " = 0;\n";
				continue;
			}
			if (samepoint(ei, ej))
			{
				++ num;
				fout << "subject to samePoint" << num << ": "
					 << generateSamePoint(i, net->bsplines[i].ctrlNodes.size() - 1, j, net->bsplines[j].ctrlNodes.size() - 1)
					 << " = 0;\n";
				continue;
			}
		}
	}
    
	//ortho
	num = 0;
	for (int i = 0; i < net->bsplines.size(); ++ i)
	{
		if (net->bsplines[i].ctrlNodes.size() != 2) continue;
		for (int j = i + 1; j < net->bsplines.size(); ++ j)
		{
			if (net->bsplines[j].ctrlNodes.size() != 2) continue;
			if (net->orthoSet.getMark(i, 0, j, 0) == 1)
			{
				++ num;
				fout << "subject to lineOrtho" << num << ": "
					 << generateLineOrtho(i, 0, j, 0)
					 << " = 0;\n";
			}
		}
	}
	//coplanar
	num = 0;
	for (int i = 0; i < net->bsplines.size(); ++ i)
	{
		for (int j = 0; j < (int)net->bsplines[i].ctrlNodes.size() - 1; ++ j)
		{
			for (int q = j + 1; q < (int)net->bsplines[i].ctrlNodes.size() - 1; ++ q)
			{
				if (isLinked(i, j, i, q))continue;
				if (net->coplanarSet.getMark(i, j, i, q) == 1)
				{
					++ num;
					fout << "subject to lineCoplanar" << num << ": "
						 << generateLineCoplanar(i, j, i, q)
						 << " = 0;\n";
				}
			}
			for (int p = i + 1; p < net->bsplines.size(); ++ p)
			{
				for (int q = 0; q < (int)net->bsplines[p].ctrlNodes.size() - 1; ++ q)
				{
					if (isLinked(i, j, p, q))continue;
					if (net->coplanarSet.getMark(i, j, p, q) == 1)
					{
						++ num;
						fout << "subject to lineCoplanar" << num << ": "
							 << generateLineCoplanar(i, j, p, q)
							 << " = 0;\n";
					}
				}
			}
		}
	}
    
	//parallel
	num = 0;
	for (int i = 0; i < net->bsplines.size(); ++ i)
	{
		if (net->bsplines[i].ctrlNodes.size() != 2) continue;
		for (int j = i + 1; j < net->bsplines.size(); ++ j)
		{
			if (net->bsplines[j].ctrlNodes.size() != 2) continue;
			if (net->parallelSet.sameRoot(i, 0, j, 0))
			{
				++ num;
				fout << "subject to lineParallel" << num << ": "
					 << generateLineParallel(i, 0, j, 0)
					 << " = 0;\n";
			}
		}
	}
	//colinear
	num = 0;
	for (int i = 0; i < net->bsplines.size(); ++ i)
	{
		if (net->bsplines[i].ctrlNodes.size() != 2) continue;
		for (int j = i + 1; j < net->bsplines.size(); ++ j)
		{
			if (net->bsplines[j].ctrlNodes.size() != 2) continue;
			if (isLinked(i, 0, j, 0))continue;
			if (net->collinearSet.sameRoot(i, 0, j, 0))
			{
				++ num;
				fout << "subject to lineColinear" << num << ": "
					 << generateLineColinear(i, 0, j, 0)
					 << " = 0;\n";
			}
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
		 << "printf {i in 0..BN, j in 0..CN[i]} \"%f %f %f\\n\", "
		 << "p[i, j, 1], p[i, j, 2], p[i, j, 3] "
		 << "> E:\\reconstruction\\point_cloud\\PointContour\\Release\\result.out;\n";
#elif defined(__APPLE__)
    fout << "reset;\n"
		 << "option ampl_include '/Users/Winmad/Projects/PointContour/ampl';\n"
		 << "option solver knitroampl;\n"
		 << "option knitro_options \"alg=1 bar_feasible=3 honorbnds=1 ms_enable=0 par_numthreads=4\";\n\n"
		 << "model test.mod;\n"
		 << "data test.dat;\n"
		 << "solve;\n"
		 << "printf {i in 0..BN, j in 0..CN[i]} \"%f %f %f\\n\", "
		 << "p[i, j, 1], p[i, j, 2], p[i, j, 3] "
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
	this->net = net;
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
    // change ctrl nodes and resample
	for (int i = 0; i < net->bsplines.size(); ++ i)
	{
		for (int j = 0; j < net->bsplines[i].ctrlNodes.size(); ++ j)
		{
			double x, y, z;
			fin >> x >> y >> z;
			net->bsplines[i].ctrlNodes[j].x = x;
			net->bsplines[i].ctrlNodes[j].y = y;
			net->bsplines[i].ctrlNodes[j].z = z;
		}
	}
	fin.close();
}

string Optimization::generateSamePoint(int i, int j, int p, int q)
{
	stringstream ss;
	ss << "sum {t in Dim3} "
	   << "abs(p[" << i << "," << j << ",t] - p[" << p << "," << q << ",t])";
	return ss.str();
}

string Optimization::generateLineOrtho(int i, int j, int p, int q)
{
	stringstream ss;
	ss << "(sum {t in Dim3}"
	   << "((p[" << i << "," << j+1 << ",t] - p[" << i << "," << j << ",t])"
	   << " * "
	   << "(p[" << p << "," << q+1 << ",t] - p[" << p << "," << q << ",t])))";
	   /*<< " / "
	   << "sqrt(sum {t in Dim3}"
	   << "(p[" << i << "," << j+1 << ",t] - p[" << i << "," << j << ",t]) ^ 2)"
	   << " / "
	   << "sqrt(sum {t in Dim3}"
	   << "(p[" << p << "," << q+1 << ",t] - p[" << p << "," << q << ",t]) ^ 2)";*/
	return ss.str();
}

string Optimization::generateLineParallel(int i, int j, int p, int q)
{
	stringstream ss;
	ss << "abs(sum {t in Dim3}"
	   << "((p[" << i << "," << j+1 << ",t] - p[" << i << "," << j << ",t])"
	   << " * "
	   << "(p[" << p << "," << q+1 << ",t] - p[" << p << "," << q << ",t])))"
	   << " - "
	   << "sqrt(sum {t in Dim3}"
	   << "(p[" << i << "," << j+1 << ",t] - p[" << i << "," << j << ",t]) ^ 2)"
	   << " * "
	   << "sqrt(sum {t in Dim3}"
	   << "(p[" << p << "," << q+1 << ",t] - p[" << p << "," << q << ",t]) ^ 2)";
	return ss.str();
}

string Optimization::generateLineColinear(int i, int j, int p, int q)
{
	stringstream ss;
	ss << "abs(sum {t in Dim3}"
	   << "((p[" << i << "," << j+1 << ",t] - p[" << i << "," << j << ",t])"
	   << " * "
	   << "(p[" << i << "," << j << ",t] - p[" << p << "," << q << ",t])))"
	   << " - "
	   << "sqrt(sum {t in Dim3}"
	   << "(p[" << i << "," << j+1 << ",t] - p[" << i << "," << j << ",t]) ^ 2)"
	   << " * "
	   << "sqrt(sum {t in Dim3}"
	   << "(p[" << i << "," << j << ",t] - p[" << p << "," << q << ",t]) ^ 2)";
	return ss.str();
}

string Optimization::generateLineCoplanar(int i, int j, int p, int q)
{
	stringstream ss;
	ss << "("
	   << "(p[" << i << "," << j+1 << ",2]-p[" << i << "," << j << ",2])"
	   << "*"
	   << "(p[" << p << "," << q+1 << ",3]-p[" << p << "," << q << ",3])"
	   << "-"
	   << "(p[" << i << "," << j+1 << ",3]-p[" << i << "," << j << ",3])"
	   << "*"
	   << "(p[" << p << "," << q+1 << ",2]-p[" << p << "," << q << ",2])"
	   << ")"
	   << "*"
	   << "(p[" << p << "," << q << ",1]-p[" << i << "," << j << ",1])"
	   << "+\n"
	   << "("
	   << "(p[" << i << "," << j+1 << ",3]-p[" << i << "," << j << ",3])"
	   << "*"
	   << "(p[" << p << "," << q+1 << ",1]-p[" << p << "," << q << ",1])"
	   << "-"
	   << "(p[" << i << "," << j+1 << ",1]-p[" << i << "," << j << ",1])"
	   << "*"
	   << "(p[" << p << "," << q+1 << ",3]-p[" << p << "," << q << ",3])"
	   << ")"
	   << "*"
	   << "(p[" << p << "," << q << ",2]-p[" << i << "," << j << ",2])"
	   << "+\n"
	   << "("
	   << "(p[" << i << "," << j+1 << ",1]-p[" << i << "," << j << ",1])"
	   << "*"
	   << "(p[" << p << "," << q+1 << ",2]-p[" << p << "," << q << ",2])"
	   << "-"
	   << "(p[" << i << "," << j+1 << ",2]-p[" << i << "," << j << ",2])"
	   << "*"
	   << "(p[" << p << "," << q+1 << ",1]-p[" << p << "," << q << ",1])"
	   << ")"
	   << "*"
	   << "(p[" << p << "," << q << ",3]-p[" << i << "," << j << ",3])";
	return ss.str();
}