#include <fstream>
#include <sstream>
#include "optimization.h"

using namespace std;

void Optimization::generateDAT(string file)
{
	ofstream fout(file.data());
	fout << "param BN := " << net->bsplines.size() - 1 << ";\n";
	fout << "param CN :=\n";
	for (int i = 0; i < net->bsplines.size(); ++ i)
	{
		fout << '\t' << net->bsplines[i].ctrlNodes.size() - 1 << endl;
	}
	fout << ";\n";
	
	fout << "param init_p :=\n";
	for (int i = 0; i < net->bsplines.size(); ++ i)
	{
		for (int j = 0; j < net->bsplines[i].ctrlNodes.size(); ++ j)
		{
			fout << "\t[" << i << ", " << j << ", *]";
			fout << " x " << net->bsplines[i].ctrlNodes[j].x
				 << " y " << net->bsplines[i].ctrlNodes[j].y
				 << " z " << net->bsplines[i].ctrlNodes[j].z
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
	fout << "set Point := x y z;\n";
	fout << "param BN;\n";
	fout << "param CN {0..BN};\n";
	fout << "param init_p {i in 0..BN, 0..CN[i], Point};\n";
	
	fout << "# variables\n";
	fout << "var p {i in 0..BN, 0..CN[i], Point} := init_p;\n";

	fout << "# intermediate variables\n";

	fout << "objective\n";
	fout << "minimize total_cost:\n";
	fout << "sum {i in 0..BN, j in 0..CN[i], t in Point}"
		 << "(p[i, j, t] - init_p[i, j, t]) ^ 2\n";
	fout << ";\n";

	//ortho
	int num = 0;
	for (int i = 0; i < net->bsplines.size(); ++ i)
	{
		if (net->bsplines[i].ctrlNodes.size() != 2) continue;
		for (int j = i; j < net->bsplines.size(); ++ j)
		{
			if (net->bsplines[i].ctrlNodes.size() != 2) continue;
			if (net->orthoSet.getMark(i, 0, j, 0) == 1)
			{
				++ num;
				fout << "subject to lineOrtho" << num << ": "
					 << generateLineOrtho(i, 0, j, 0) << endl;
			}
		}
	}
	fout.close();
}

void Optimization::generateRUN(string file)
{
	ofstream fout(file.data());
	fout << "reset;\n"
		 << "model test.mod;\n"
		 << "data test.dat;\n"
		 << "solve;\n"
		 << "printf {i in 0..BN, j in 0..CN[i]} \"%f %f %f\\n\", "
		 << "p[i, j, x] p[i, j, y] p[i, j, z] "
		 << ">> result.out;\n";
	fout.close();
}

void Optimization::generateBAT(string file)
{
	ofstream fout(file.data());
	fout << "E:\n"
		 << "cd E:\\reconstruction\\AMPLcml\n"
		 << "ampl.exe E:\\reconstruction\\point cloud\\PointContour\\test.run\n"
		 << "pause\n";
	fout.close();
}

void Optimization::run(CurveNet *net)
{
	this->net = net;
	generateDAT("test.dat");
	generateMOD("test.mod");
	generateRUN("test.run");
	generateBAT("test.bat");
}

string Optimization::generateLineOrtho(int i, int j, int p, int q)
{
	stringstream ss;
	ss << "(p[" << i << ", " << j+1 << ", x] - p[" << i << ", " << j << ", x])"
	   << " * "
	   << "(p[" << p << ", " << q+1 << ", x] - p[" << p << ", " << q << ", x])"
	   << " + "
	   << "(p[" << i << ", " << j+1 << ", y] - p[" << i << ", " << j << ", y])"
	   << " * "
	   << "(p[" << p << ", " << q+1 << ", y] - p[" << p << ", " << q << ", y])"
	   << " + "
	   << "(p[" << i << ", " << j+1 << ", z] - p[" << i << ", " << j << ", z])"
	   << " * "
	   << "(p[" << p << ", " << q+1 << ", z] - p[" << p << ", " << q << ", z])"
	   << " = 0;\n";
	return ss.str();
}