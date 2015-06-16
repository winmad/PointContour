#ifndef OPTIMIZATION_H
#define OPTIMIZATION_H

#include "curveNet.h"
#include "TimeManager.h"
#include "Plane.h"

class PointCloudUtils;

struct OptVariable
{
    // type 0: ni = node index
    // type 1: ni = bspline index, ci = ctrlNodes index
    int type , ni , ci;
    double weight;

    bool isVar;
    int index; // index of var or const

    OptVariable() {}
    
    OptVariable(int _type , int _ni , double _weight = 0)
    {
        type = _type; ni = _ni; ci = 0; weight = _weight;
    }

    OptVariable(int _type , int _ni , int _ci , double _weight = 0)
    {
        type = _type; ni = _ni; ci = _ci; weight = _weight;
    }
};

enum ConstraintsType
{
    st_collinear = 0,
    st_parallel,
    st_coplanar,
    st_ortho,
    st_tangent,
	st_symmetric
};

struct OptConstraints
{
    int u1 , u2;
    int v1 , v2;
    double weight;
    ConstraintsType type;

    OptConstraints() {}
    
    OptConstraints(int _u1 , int _u2 , int _v1 , int _v2 ,
        ConstraintsType _type , double _weight = 1.0)
    {
        u1 = _u1; u2 = _u2; v1 = _v1; v2 = _v2; type = _type;
        weight = _weight;
    }
};

class Optimization
{
public:
    Optimization();
    ~Optimization();
    void init(CurveNet *net , PointCloudUtils *pcUtils);
	void generateDAT(std::string file);
	void generateMOD(std::string file);
	void generateRUN(std::string file);
	void generateBAT(std::string file);
	void run(CurveNet *net);

    std::string amplExePath;
    std::string amplIncludePath;
    std::string amplResultPath;
    int numStartPoints;
    double maxRealTime;
    double largeBound , smallBound;

    void addParallelConstraint(int l1 , int l2 , double weight = 1.0);

private:
    std::string var2str(int varIndex , int k);
    std::string generateStraightLineDist(double weight);
    std::string generateBsplineDist(double weight);
	std::string generateLineOrtho(int, int, int, int, double weight = 1.0);
	std::string generateLineParallel(int, int, int, int, double weight = 1.0);
	std::string generateLineCoplanar(int, int, int, int, double weight = 1.0);
	std::string generateLineCollinear(int, int, int, int, double weight = 1.0);
	std::string generateLineTangent(int, int, int, int, double weight = 1.0);
	std::string generateSamePoint(int, int);
	std::string generateCoplanar(int, int, double weight = 1.0);
	std::string generateSymmetryLine(int, std::pair<int, int>);
	std::string generateSymmetryPoint(int, int, int);
	std::string generateSelfSymmPoint(int, int);
	void addCoplanar(int, int, int, int);
    void addCoplanar(int varIndex , Plane& plane);

	bool isLinked(int, int, int, int);

    void addOptVariable(OptVariable optVar);
    int getOptVarIndex(const OptVariable& optVar);
    double var2double(const OptVariable& v);
    std::pair<OptVariable , OptVariable> bsp2var(int bspIndex , int curveIndex , int numCtrlCurves);
    bool isAllConst(int u1 , int u2 , int v1 , int v2);

    CurveNet *net;
    PointCloudUtils *pcUtils;

	int tmpVarNum;
    int numVars; // numVars = numConsts + numVaris;
    int numConsts , numVaris;
    std::vector<OptVariable> vars;
    std::map<double , int> double2vi;

	std::vector<std::vector<int> > straightlines;
	std::vector<std::vector<int> > bsplines;
	std::vector<std::vector<int> > featurelines;
	std::vector<int> straightlinesIdx;
	std::vector<int> bsplinesIdx;
	std::vector<int> curveIdx;
	std::vector<bool> lastDraw;

    int numCons;
    std::vector<OptConstraints> cons;
	std::vector<Plane> coplanes;
	std::vector<std::vector<int> > coplanarPoints;

    // 0: collinear, 1: parallel, 2: orthogonal, 3: tangent, 4: coplane
    std::vector<int> constraintTypeCount;
    TimeManager timer;
};

#endif //OPTIMIZATION_H