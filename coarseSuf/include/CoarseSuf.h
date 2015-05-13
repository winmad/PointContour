//FOR LIFAN

#ifndef _COARSESUF_H
#define _COARSESUF_H

#include "DMWT.h"
#include "nvVector.h"
#include "Utility.h"
#include <stdio.h>

using namespace std;

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


int coarseSuf (
	// input
	std::vector<int> &numPoints,
	std::vector<double*> &inCurves,
	std::vector<double*> &inNorms,
	Volume *inGrid,
	bool useDelaunay,
	bool useMinSet,
	bool useNormal,
	float areaWeight,
	float edgeWeight,
	float dihedralWeight,
	float boundaryNormalWeight,
	float ptWeight,
	// output
	int &_numofpoints, 
	int &_numoftilingtris, 
	float** _pPositions, 
	float** _pNormals, 
	int** _pFaceIndices
	){

		try{	
			// init
			DMWT myDMWT (numPoints, inCurves, inNorms, inGrid, useDelaunay, useMinSet, useNormal);
			myDMWT.setWeights(areaWeight, edgeWeight, dihedralWeight, boundaryNormalWeight, ptWeight);

			// prepare datastructures
			myDMWT.buildList();
			myDMWT.buildTriPtScores();

			// tiling
			myDMWT.tile();

			// save results
			myDMWT.saveResults(_numofpoints, _numoftilingtris, _pPositions, _pNormals, _pFaceIndices );
			//myDMWT.saveTilingObj("init_triangulation.obj");

#if (COARSESUF_OUTPUT)
			cout << "optScore = " << myDMWT.optCost << endl;
#endif

		}catch(int e){
			cout<<"DMWT: Error!! Exception Nr. "<<e<<endl;
			return 1; 
		}

		return 0;
}

int genInitTriangulation(
	// input
	std::vector<int> &numPoints,
	std::vector<double*> &inCurves,
	std::vector<double*> &inNorms,
	std::vector<int>& sizeField,
	std::vector<double>& extXval,
	std::vector<double>& extYval,
	std::vector<double>& extZval,
	double*** distField,
	bool useDelaunay,
	bool useMinSet,
	bool useNormal,
	float areaWeight,
	float edgeWeight,
	float dihedralWeight,
	float boundaryNormalWeight,
	float ptWeight,
	// output
	int &_numofpoints, 
	int &_numoftilingtris, 
	float** _pPositions, 
	float** _pNormals, 
	int** _pFaceIndices
	)
{
	try
	{
		// -------------------------- read dist-field, do triangulation ---------------------------//
		Volume * inGrid = new Volume();
#if (COARSESUF_OUTPUT)
		cout << "start reading dist field..." << endl;
#endif
		//Utility::readLiFanGrid(distfile, inGrid);
		Utility::loadLifanGrid(sizeField , distField , extXval , extYval , extZval ,
			inGrid);
		//inGrid->writeVolume("out_dist_field.txt");
		//inGrid->toMRCFile(argc[Utility::OUTMRC]);
#if (COARSESUF_OUTPUT)
		cout << "\nFinished reading! Start triangulation!" << endl;
#endif

		coarseSuf(	
			// input
			numPoints,
			inCurves,
			inNorms,
			inGrid,
			useDelaunay,
			useMinSet,
			useNormal,
			areaWeight,
			edgeWeight,
			dihedralWeight,
			boundaryNormalWeight,
			ptWeight,
			// output
			_numofpoints, 
			_numoftilingtris, 
			_pPositions, 
			_pNormals, 
			_pFaceIndices
		);
#if (COARSESUF_OUTPUT)
		cout << "DONE!" << endl;
#endif
	}
	catch (int e)
	{
		cout<<"DMWT: Error!! Exception Nr. "<<e<<endl;
		return 1; 
	}
	return 0;
}

}

#endif