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

	fout << "\nobjective\n";
	fout << "minimize total_cost:\n";
	fout << "sum {i in 0..BN, j in 0..CN[i], t in Dim3} "
		 << "(p[i, j, t] - init_p[i, j, t]) ^ 2\n";
	fout << ";\n\n";

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
					 << generateLineOrtho(i, 0, j, 0);
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
		 << "p[i, j, 1] p[i, j, 2] p[i, j, 3] "
		 << ">> E:\\reconstruction\\point_cloud\\PointContour\\result.out;\n";
	fout.close();
}

void Optimization::generateBAT(string file)
{
	ofstream fout(file.data());
	fout << "E:\n"
		 << "cd E:\\reconstruction\\AMPLcml\n"
		 << "ampl.exe E:\\reconstruction\\point_cloud\\PointContour\\test.run\n"
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
	ss << "sum {t in Dim3}"
	   << "((p[" << i << "," << j+1 << ",t] - p[" << i << "," << j << ",t])"
	   << " * "
	   << "(p[" << p << "," << q+1 << ",t] - p[" << p << "," << q << ",t]))"
	   << " / "
	   << "sqrt(sum {t in Dim3}"
	   << "(p[" << i << "," << j+1 << ",t] - p[" << i << "," << j << ",t]) ^ 2)"
	   << " / "
	   << "sqrt(sum {t in Dim3}"
	   << "(p[" << p << "," << q+1 << ",t] - p[" << p << "," << q << ",t]) ^ 2)"
	   << " = 0;\n";
	return ss.str();
}

string Optimization::generateLineParallel(int i, int j, int p, int q)
{
	stringstream ss;
	ss << "abs(sum {t in Dim3}"
	   << "((p[" << i << "," << j+1 << ",t] - p[" << i << "," << j << ",t])"
	   << " * "
	   << "(p[" << p << "," << q+1 << ",t] - p[" << p << "," << q << ",t]))"
	   << " / "
	   << "sqrt(sum {t in Dim3}"
	   << "(p[" << i << "," << j+1 << ",t] - p[" << i << "," << j << ",t]) ^ 2)"
	   << " / "
	   << "sqrt(sum {t in Dim3}"
	   << "(p[" << p << "," << q+1 << ",t] - p[" << p << "," << q << ",t]) ^ 2))"
	   << " -1 = 0;\n";
	return ss.str();
}