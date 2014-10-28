#include <fstream>
#include <sstream>
#include <cstdlib>
#include "optimization.h"

using namespace std;

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
	fout << "var p {i in 0..BN, j in 0..CN[i], t in Dim3} := init_p[i, j, t];\n";

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

	//ortho
	int num = 0;
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
	fout << "reset;\n"
		 << "option ampl_include 'E:\\reconstruction\\point_cloud\\PointContour\\Release';\n"
		 << "model test.mod;\n"
		 << "data test.dat;\n"
		 << "solve;\n"
		 << "printf {i in 0..BN, j in 0..CN[i]} \"%f %f %f\\n\", "
		 << "p[i, j, 1], p[i, j, 2], p[i, j, 3] "
		 << "> E:\\reconstruction\\point_cloud\\PointContour\\Release\\result.out;\n";
	fout.close();
}

void Optimization::generateBAT(string file)
{
	ofstream fout(file.data());
	fout << "E:\n"
		 << "cd E:\\reconstruction\\AMPLcml\n"
		 << "ampl.exe E:\\reconstruction\\point_cloud\\PointContour\\Release\\test.run\n";
		 //<< "pause\n";
	fout.close();
}

void Optimization::run(CurveNet *net)
{
	this->net = net;
	printf("(%.6f,%.6f,%.6f)\n" , net->bsplines[0].ctrlNodes[0].x ,
		net->bsplines[0].ctrlNodes[0].y , net->bsplines[0].ctrlNodes[0].z);
	generateDAT("test.dat");
	generateMOD("test.mod");
	generateRUN("test.run");
	generateBAT("test.bat");
	system("test.bat");
	ifstream fin("result.out");
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

string Optimization::generateLineOrtho(int i, int j, int p, int q)
{
	stringstream ss;
	ss << "(sum {t in Dim3}"
	   << "((p[" << i << "," << j+1 << ",t] - p[" << i << "," << j << ",t])"
	   << " * "
	   << "(p[" << p << "," << q+1 << ",t] - p[" << p << "," << q << ",t])))"
	   << " / "
	   << "sqrt(sum {t in Dim3}"
	   << "(p[" << i << "," << j+1 << ",t] - p[" << i << "," << j << ",t]) ^ 2)"
	   << " / "
	   << "sqrt(sum {t in Dim3}"
	   << "(p[" << p << "," << q+1 << ",t] - p[" << p << "," << q << ",t]) ^ 2)";
	return ss.str();
}

string Optimization::generateLineParallel(int i, int j, int p, int q)
{
	stringstream ss;
	ss << "abs((sum {t in Dim3}"
	   << "((p[" << i << "," << j+1 << ",t] - p[" << i << "," << j << ",t])"
	   << " * "
	   << "(p[" << p << "," << q+1 << ",t] - p[" << p << "," << q << ",t])))"
	   << " / "
	   << "sqrt(sum {t in Dim3}"
	   << "(p[" << i << "," << j+1 << ",t] - p[" << i << "," << j << ",t]) ^ 2)"
	   << " / "
	   << "sqrt(sum {t in Dim3}"
	   << "(p[" << p << "," << q+1 << ",t] - p[" << p << "," << q << ",t]) ^ 2))"
	   << " -1";
	return ss.str();
}

string Optimization::generateLineColinear(int i, int j, int p, int q)
{
	stringstream ss;
	ss << "abs((sum {t in Dim3}"
	   << "((p[" << i << "," << j+1 << ",t] - p[" << i << "," << j << ",t])"
	   << " * "
	   << "(p[" << i << "," << j << ",t] - p[" << p << "," << q << ",t])))"
	   << " / "
	   << "sqrt(sum {t in Dim3}"
	   << "(p[" << i << "," << j+1 << ",t] - p[" << i << "," << j << ",t]) ^ 2)"
	   << " / "
	   << "sqrt(sum {t in Dim3}"
	   << "(p[" << i << "," << j << ",t] - p[" << p << "," << q << ",t]) ^ 2))"
	   << " -1";
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