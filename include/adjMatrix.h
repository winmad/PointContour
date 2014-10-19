#ifndef ADJ_MATRIX_H
#define ADJ_MATRIX_H

#include "smallUtils.h"
#include <map>
#include <utility>
#include <vector>

class AdjMatrix
{
public:
    AdjMatrix();
    
    void clear();
    
    void newCurve(int bspIndex , int curveIndex);
    void connect(int bspIndex1 , int curveIndex1 , int bspIndex2 , int curveIndex2 ,
        int mark);
    int getMark(int bspIndex1 , int curveIndex1 , int bspIndex2 , int curveIndex2);

    int forHash(int bspIndex , int curveIndex);
    std::pair<int , int> backHash(int index);

    int numCurves;
    std::map<int , int> curveDict;
    std::vector<int> curveId;
    int maxN;
    int **mat;
};

#endif