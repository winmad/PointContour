#ifndef OPTIMIZATION_H
#define OPTIMIZATION_H

#include "curveNet.h"

struct OptVariable
{
    // type 0: ni = node index
    // type 1: ni = bspline index, ci = ctrlNodes index
    int type , ni , ci;

    OptVariable() {}
    
    OptVariable(int _type , int _ni)
    {
        type = _type; ni = _ni; ci = 0;
    }

    OptVariable(int _type , int _ni , int _ci)
    {
        type = _type; ni = _ni; ci = _ci;
    }
};

enum ConstraintsType
{
    st_collinear = 0,
    st_parallel,
    st_coplanar,
    st_ortho,
    st_tangent
};

struct OptConstraints
{
    int u1 , u2;
    int v1 , v2;
    ConstraintsType type;

    OptConstraints() {}
    
    OptConstraints(int _u1 , int _u2 , int _v1 , int _v2 ,
        ConstraintsType _type)
    {
        u1 = _u1; u2 = _u2; v1 = _v1; v2 = _v2; type = _type;
    }
};

class Optimization
{
public:
    Optimization();
    void init(CurveNet *net);
	void generateDAT(std::string file);
	void generateMOD(std::string file);
	void generateRUN(std::string file);
	void generateBAT(std::string file);
	void run(CurveNet *net);

private:
	std::string generateLineOrtho(int, int, int, int);
	std::string generateLineParallel(int, int, int, int);
	std::string generateLineCoplanar(int, int, int, int);
	std::string generateLineCollinear(int, int, int, int);
	std::string generateLineTangent(int, int, int, int);
	std::string generateSamePoint(int, int);
	std::string generateCoplanar(int, int);
	void addCoplanar(int, int, int, int);

	bool isLinked(int, int, int, int);

    void addOptVariable(OptVariable optVar);
    int getOptVarIndex(const OptVariable& optVar);
    double var2double(const OptVariable& v);
    std::pair<OptVariable , OptVariable> bsp2var(int bspIndex , int curveIndex , int numCtrlCurves);

    CurveNet *net;

	int tmpVarNum;
    int numVars;
    std::vector<OptVariable> vars;
    std::map<double , int> double2vi;

    int numCons;
    std::vector<OptConstraints> cons;
	std::vector<Plane> coplanes;
	std::vector<std::vector<int> > coplanarPoints;
};

#endif //OPTIMIZATION_H