#ifndef OPTIMIZATION_H
#define OPTIMIZATION_H

#include "curveNet.h"

class Optimization
{
public:
	void generateDAT(std::string file);
	void generateMOD(std::string file);
	void generateRUN(std::string file);
	void generateBAT(std::string file);
	void run(CurveNet *net);
private:
	std::string generateLineOrtho(int, int, int, int);
	std::string generateLineParallel(int, int, int, int);
	CurveNet *net = NULL;
};

#endif //OPTIMIZATION_H