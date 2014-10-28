#ifndef OPTIMIZATION_H
#define OPTIMIZATION_H

#include "curveNet.h"

class Optimization
{
public:
	Optimization():net(NULL){}
	void generateDAT(std::string file);
	void generateMOD(std::string file);
	void generateRUN(std::string file);
	void generateBAT(std::string file);
	void run(CurveNet *net);
	
private:
	std::string generateLineOrtho(int, int, int, int);
	std::string generateLineParallel(int, int, int, int);
	std::string generateLineCoplanar(int, int, int, int);
	std::string generateLineColinear(int, int, int, int);
	std::string generateSamePoint(int, int, int, int);
	bool isLinked(int, int, int, int);
	CurveNet *net;
};

#endif //OPTIMIZATION_H