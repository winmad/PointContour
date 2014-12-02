
//FOR LIFAN

#ifndef _COARSESUF_H
#define _COARSESUF_H

#include "DMWT.h"
#include <stdio.h>

namespace coarseSuf
{

int coarseSuf (
	// input
    std::vector<int> &numPoints,
	std::vector<double*> &inCurves,
	std::vector<double*> &inNorms,
	bool useDelaunay,
	bool useMinSet,
	bool useNormal,
	float areaWeight,
	float edgeWeight,
	float dihedralWeight,
	float boundaryNormalWeight,
	// output
	int &_numofpoints, 
	int &_numoftilingtris, 
	float** _pPositions, 
	float** _pNormals, 
	int** _pFaceIndices

	){

	try{	
		// init
		DMWT myDMWT (numPoints, inCurves, inNorms, useDelaunay, useMinSet, useNormal);
		myDMWT.setWeights(areaWeight, edgeWeight, dihedralWeight, boundaryNormalWeight);

		// prepare datastructures
		myDMWT.buildList();
	
		// tiling
		myDMWT.tile();
	
		// save results
		myDMWT.saveResults(_numofpoints, _numoftilingtris, _pPositions, _pNormals, _pFaceIndices );
		/* myDMWT.saveTilingObj("surface.OBJ"); */

	}catch(int e){
		cout<<"DMWT: Error!! Exception Nr. "<<e<<endl;
		return 1; 
	}

	return 0;
}

}
#endif