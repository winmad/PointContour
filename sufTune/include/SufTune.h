#ifndef SURFACE_TUNE_H
#define SURFACE_TUNE_H

#include "Utility.h"
#include "meshFair.h"

namespace SufTune
{
	int sufTune(
		// input
		int numPoints,
		int numTris,
		float* pPositions,
		int* pFaceIndices,
		std::vector<int>& sizeField,
		std::vector<double>& extXval,
		std::vector<double>& extYval,
		std::vector<double>& extZval,
		double*** distField,
		double alpha0, double alphan, 
		double ratio12, double ratio3, 
		int m_stime_bef, int m_time, int m_stime, 
		bool m_doSwap,
		// output
		int& _numPoints,
		int& _numTris,
		float** _pPositions,
		int** _pFaceIndices
		)
	{
		try
		{
			double delta = (alphan - alpha0)/(m_time==0?1:m_time);
			double alpha = alpha0;

			//double ratio = 0.5;

			vector<double> m_verLu;
			intvector m_faceLu;
			intvector m_ctrmedgeLu;

			vector<vector<double> > sufV;
			vector<vector<int> > sufF;
			vector<vector<int> > sufSeg;
			//Utility::readTilingObj(objfile, sufV, sufF);
			Utility::loadTilingObj(numPoints , numTris , pPositions , pFaceIndices , sufV , sufF);
			// for triangulation of single curve, its input vertices is the curve (constrained segs)
			if(true){
				sufSeg.resize(sufV.size(), vector<int>(2));
				for(int i=0; i<(int)sufV.size(); ++i){
					sufSeg[i][0] = i;
					sufSeg[i][1] = i+1;
				}
				if (sufSeg.size() >= 1) sufSeg[sufSeg.size()-1][1] = 0;
			} else {
				//Utility::readAnArrayOfPairs(segfile,sufSeg);
			}
			//cout << "convert data ..." << endl;
			Utility::flattenVectorsOfSameSize(sufV, m_verLu);
			Utility::flattenVectorsOfSameSize(sufF, m_faceLu);
			Utility::flattenVectorsOfSameSize(sufSeg, m_ctrmedgeLu);

			Volume * inGrid = new Volume();
#if (SUFTUNE_OUTPUT)
			cout << "start reading dist field..." << endl;
#endif
			//Utility::readLiFanGrid(distfile, inGrid);
			Utility::loadLifanGrid(sizeField , distField , extXval , extYval , extZval ,
				inGrid);
#if (SUFTUNE_OUTPUT)
			cout << "finished reading dist field." << endl;
#endif

			//Utility::printVector(m_verLu);
			//Utility::printVector(m_faceLu);

			//Utility::saveTilingObj("test.obj",m_verLu,m_faceLu);
#if (SUFTUNE_OUTPUT)
			cout << "start JuFair ... " << endl;
			cout << "#V = " << sufV.size() << endl;
			cout << "#F = " << sufF.size() << endl;
			cout << "#Seg = " << sufSeg.size() << endl;
			cout << "alpha0 = " << alpha0 << endl;
			cout << "alphan = " << alphan << endl;
			cout << "ratio12 = " << ratio12 << endl;
			cout << "ratio3 = " << ratio3 << endl;
			cout << "m_time = " << m_time << endl;
			cout << "m_stime = " << m_stime << endl;
			cout << "m_stime_bef = " << m_stime_bef << endl;
			cout << "grid size = " << inGrid->getGridSize(0) << " " << inGrid->getGridSize(1) << " " << inGrid->getGridSize(2) << endl;
#endif
			LuMesh m_meshFair;
			//entry, include vertices, faces, and controalEdges. doSwap is true or false;
			//cout << "JuFair InputData..." << endl;
			m_meshFair.InputData(m_verLu,m_faceLu,m_ctrmedgeLu, m_doSwap);

			m_meshFair.JUFair( ratio12, ratio3, m_stime_bef, inGrid);

			//m_meshFair.LaplacianSmooth(ratio, m_stime_bef);
			//m_meshFair.SDUmbraFair(m_stime_bef);
			//cout << "JuFair Go..." << endl;
			for( int i = 0; i < m_time; i ++)
			{
#if (SUFTUNE_OUTPUT)
				cout<<endl<<"smooth:"<<i<<endl;
#endif

				alpha += delta;

				m_meshFair.LiepaRefine(alpha);
				m_meshFair.JUFair( ratio12, ratio3, m_stime, inGrid);
			}

			//cout << "JuFair OutputData..." << endl;
			m_verLu.clear();
			m_faceLu.clear();
			m_ctrmedgeLu.clear();
			m_meshFair.OutputData(m_verLu,m_faceLu);

			//cout << "Save Obj..." << endl;
			//Utility::saveTilingObj("sufTune_res.obj",m_verLu,m_faceLu);
			Utility::saveResults(m_verLu, m_faceLu, 
				_numPoints , _numTris , _pPositions , _pFaceIndices);
		}
		catch (int e)
		{
			cout<<"SufTune: Error!! Exception Nr. "<<e<<endl;
			return 1; 
		}
		return 0;
	}
}

#endif