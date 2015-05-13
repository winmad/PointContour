#ifndef _SUFTUNE_UTILITY_H_
#define _SUFTUNE_UTILITY_H_

#ifndef _DEBUG_MING
#define _DEBUG_MING 1
#endif

#ifndef _DEBUG_MING_MORE
#define _DEBUG_MING_MORE 0
#endif

#ifndef _DEBUG_MING_TETGEN
#define _DEBUG_MING_TETGEN 1
#endif

#ifndef VOLCELLN
#define VOLCELLN 256
#endif

#ifndef STDBBOXMINSCALE
#define STDBBOXMAXSCALE 80
#endif

#ifndef POLYMENDERENLARGERATIO
#define POLYMENDERENLARGERATIO 1.25
#endif

#ifndef MIXISOFAKEVALUE
#define MIXISOFAKEVALUE -0.5f
#endif

#ifndef INFINITY
#define INFINITY 99999999999999
#endif

#include <vector>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <cmath>

using namespace std;

namespace SufTune
{

extern int db_celli;

class Volume{
public:
	Volume():minValue(-1), maxValue(0),meanValue(0){}
	~Volume(){}
	void inline init(int x, int y, int z) {
		data.resize(x, vector<vector<float> >(y, vector<float>(z)));
		gridSize.resize(3);
		gridSize[0] = x;
		gridSize[1] = y;
		gridSize[2] = z;
	}
	void inline clear(){
		data.clear();
	}
	bool inline empty(){
		return data.empty();
	}
	void inline setDataAt(int i, int j, int k, float d){
		data[i][j][k] = d;
	}
	void inline setMinMaxValue(const float min, const float max){
		minValue = min;
		maxValue = max;
	}
	float inline getDataAt(int i, int j, int k){
		return data[i][j][k];
	}
	float inline getUnit(int i){
		return unitXYZ[i];
	}
	float inline getLowerCorner(int i){
		return lowCornerXYZ[i];
	}
	int inline getGridSize(int i){
		return gridSize[i];
	}
	void inline setLowerCornerXYZ(float x, float y, float z){
		lowCornerXYZ.resize(3);
		lowCornerXYZ[0] = x;
		lowCornerXYZ[1] = y;
		lowCornerXYZ[2] = z;
	}
	void inline setUnitXYZ(float x, float y, float z){
		unitXYZ.resize(3);
		unitXYZ[0] = x;
		unitXYZ[1] = y;
		unitXYZ[2] = z;
	}
	void inline normalize(float min, float max){
		float ratio = (max-min)/(maxValue-minValue);
		float trans = min + minValue*ratio;
		for(int i=0; i<gridSize[0]; ++i){
			for(int j=0; j<gridSize[1]; ++j){
				for(int k=0; k<gridSize[2]; ++k){
					data[i][j][k] = ratio * data[i][j][k] + trans;
				}
			}
		}
	}
	void writeVol(const char* fname){
		int x = data.size();
		int y = data[0].size();
		int z = data[0][0].size();
		ofstream ofs(fname, std::ofstream::out);
		ofs << "{";
		for (int i = 0; i < x; ++i){
			ofs << "{";
			for (int j = 0; j < y; ++j){
				ofs << "{";
				for (int k = 0; k < z; ++k){
					ofs << data[i][j][k];
					if (k < z - 1){
						ofs << ",";
					}
				}
				if (j < y - 1){
					ofs << "},";
				}
				else{
					ofs << "}";
				}
			}
			if (i < x - 1){
				ofs << "},";
			}
			else{
				ofs << "}";
			}
		}
		ofs << "}";
		ofs.close();
	}
	float getVolValAtPos(float x, float y, float z){

		float xyz2 [3];
		//xyz2[0] = xyz[0];
		//xyz2[1] = xyz[1];
		//xyz2[2] = xyz[2];
		xyz2[0] = x;
		xyz2[1] = y;
		xyz2[2] = z;
		int xyz0 [3];
		int xyz1 [3];
		float xyzD1 [3];
		float xyzD0 [3];
		for(int i=0; i<3; ++i){
			float val = 1.0f*(xyz2[i]-getLowerCorner(i))/getUnit(i);
			val = val < 0.0f ? 0.0f : val; //need this, because of numerical issue
			xyz0[i] = std::floor(val);
			xyz1[i] = xyz0[i] == (getGridSize(i)-1) ? (getGridSize(i)-1) : xyz0[i]+1;
			xyzD1[i] = val - xyz0[i];
			xyzD0[i] = 1 - xyzD1[i];
		}
		float val = 
			getDataAt(xyz0[0],xyz0[1],xyz0[2])*xyzD0[0]*xyzD0[1]*xyzD0[2]
		+   getDataAt(xyz1[0],xyz0[1],xyz0[2])*xyzD1[0]*xyzD0[1]*xyzD0[2]
		+   getDataAt(xyz1[0],xyz1[1],xyz0[2])*xyzD1[0]*xyzD1[1]*xyzD0[2]
		+   getDataAt(xyz0[0],xyz1[1],xyz0[2])*xyzD0[0]*xyzD1[1]*xyzD0[2]
		+   getDataAt(xyz0[0],xyz0[1],xyz1[2])*xyzD0[0]*xyzD0[1]*xyzD1[2]
		+   getDataAt(xyz1[0],xyz0[1],xyz1[2])*xyzD1[0]*xyzD0[1]*xyzD1[2]
		+   getDataAt(xyz1[0],xyz1[1],xyz1[2])*xyzD1[0]*xyzD1[1]*xyzD1[2]
		+   getDataAt(xyz0[0],xyz1[1],xyz1[2])*xyzD0[0]*xyzD1[1]*xyzD1[2];

		return val;
	}


	/* Function to write out to a MRC file
	 *
	 * fname: output filename
	 * 
	 * data: 3D volume to be written out
	 * getDataAt: function to retrieve value in data
	 * sizex,sizey,sizez: dimension of
	 */
	void toMRCFile( const char* fname )
	{
		FILE* fout = fopen( fname, "wb" ) ;

		// Write header
		fwrite( &gridSize[0], sizeof( int ), 1, fout ) ;
		fwrite( &gridSize[1], sizeof( int ), 1, fout ) ;
		fwrite( &gridSize[2], sizeof( int ), 1, fout ) ;

		int mode = 2 ;
		fwrite( &mode, sizeof ( int ), 1, fout ) ;
		
		int off[3] = {0,0,0} ;
		int intv[3] = { gridSize[0] - 1, gridSize[1] - 1, gridSize[2] - 1 } ;
		fwrite( off, sizeof( int ), 3, fout ) ;
		fwrite( intv, sizeof( int ), 3, fout ) ;

		float cella[3] = {intv[0]*unitXYZ[0],intv[1]*unitXYZ[1],intv[2]*unitXYZ[2]} ;
		float cellb[3] = {90,90,90} ;
		fwrite( cella, sizeof( float ), 3, fout ) ;
		fwrite( cellb, sizeof( float ), 3, fout ) ;

		int cols[3] = {1,2,3} ;
		fwrite( cols, sizeof( int ), 3, fout ) ;

		float ds[3] = {minValue, maxValue, meanValue} ;
		fwrite( ds, sizeof( float ), 3, fout ) ;

		int zero = 0 ;
		for ( int i = 22 ; i < 256 ; i ++ )
		{
			fwrite( &zero, sizeof( int ), 1, fout ) ;
		}

		// Write contents
		for ( int z = 0 ; z < gridSize[2] ; z ++ ){
			for ( int y = 0 ; y < gridSize[1] ; y ++ ){
				for ( int x = 0 ; x < gridSize[0] ; x ++ )
				{
					float neg = -data[x][y][z];
					//fwrite( &data[x][y][z], sizeof( float ), 1, fout ) ;
					fwrite( &neg, sizeof( float ), 1, fout ) ;
				}
			}
		}

		fclose( fout ) ;
	}

private:
	vector<vector<vector<float> > > data;
	vector<float> lowCornerXYZ;
	vector<float> unitXYZ;
	vector<int> gridSize;
	float minValue;
	float maxValue;
	float meanValue;
};

class Utility{
public:
	enum INOUTSIDE {OUTSIDE = 0, INSIDE = 1, UNKNOWN = -2};
	enum INPARA {EXE = 0, INCONTOUR, INVOL, INBBOX, ININSIDEMAT, OUTDIR, TETLIMIT, RANDWB, NFAIR, NJUFAIR, FIRSTGENUS};
	enum INPUTPARA {LIFANEXE = 0, INSUF, INCURVE, INDIST, OUTSUF, ALPHA0, ALPHAN, RATIO12, RATIO3, TIMEBEF, TIMEN, TIMEIN};

	template <class T>
	static void writeVector(vector<vector<T> >& vec, ofstream& ofs);

	template <class T>
	static void writeVector(vector<T>& vec, ofstream& ofs);

	template <class T>
	static void writeVector(T& vec, const char* filename);

	template <class T>
	static void writeVectorForm(vector<vector<T> >& vec, ofstream& ofs);

	template <class T>
	static void writeVectorForm(vector<T>& vec, ofstream& ofs);

	template <class T>
	static void writeVectorForm(T& vec, const char* filename);

	template <class T>
	static void printVector(vector<vector<T> >& in);

	template <class T>
	static void printVector(vector<T>& in);

	template <class T>
	static string toStr(T& in);

	template <class T>
	static void flattenVectorsOfSameSize(const vector<vector<T> >& vec, vector<T> & res);

	// purely for debug
	template <class T>
	static void writeOutOneVector(const char* filename, T &vec); 

	inline static bool isFileReadable (const char* filename) {
		if (FILE *file = fopen(filename, "r")) {
			fclose(file);
			return true;
		} else {
			return false;
		} 
	}

	static void readLiFanGrid(const char* filename, Volume* inGrid){
		ifstream ifs(filename, ofstream::in);
		if (!ifs.good()) {
			cout << "Can not read file " << filename << endl;
			return;
		}

		int dimx, dimy, dimz;
		vector<float> lowCorner(3);
		vector<float> highCorner(3);
		float x, y, z, value;
		ifs >> dimx >> dimy >> dimz;
		//inGrid.resize(dimx, vector<vector<T> >(dimy, vector<T>(dimz)));
		inGrid->init(dimx, dimy, dimz);
		// read each curve
		ifs >> x >> y >> z >> value;
		inGrid->setDataAt(0,0,0,value);
		inGrid->setLowerCornerXYZ(x, y, z);
		
		for(int i=0; i<dimx; i++){
			for(int j=0; j<dimy; j++){
				for(int k=0; k<dimz; k++){
					if(((i==0)&&(j==0)&&(z==0)) || ((i==(dimx-1))&&(j==(dimy-1))&&(z==(dimz-1)))){
						continue;
					}
					ifs >> x >> y >> z >> value;
					inGrid->setDataAt(i,j,k,value);
					/*if(i<10 && j<10 && k<10){
						cout << i<< " "<< j << " "<< k<< " "<< value << " " << inGrid->getDataAt(i,j,k)<<endl;
					}*/
				}
			}
		}

		ifs >> x >> y >> z >> value;
		inGrid->setDataAt(dimx-1, dimy-1, dimz-1,value);
		inGrid->setUnitXYZ((x-inGrid->getLowerCorner(0))/(dimx-1), (y-inGrid->getLowerCorner(1))/(dimy-1), (z-inGrid->getLowerCorner(2))/(dimz-1));
		
		ifs.close();
	}


	// added by Lifan
	static void loadLifanGrid(std::vector<int>& sizeField, double*** distField, 
		std::vector<double>& extXval, std::vector<double>& extYval, std::vector<double>& extZval, 
		Volume* inGrid) 
	{
		int dimx, dimy, dimz;
		vector<float> lowCorner(3);
		vector<float> highCorner(3);
		float x, y, z, value;
		dimx = sizeField[0]; dimy = sizeField[1]; dimz = sizeField[2];
		//inGrid.resize(dimx, vector<vector<T> >(dimy, vector<T>(dimz)));
		inGrid->init(dimx, dimy, dimz);
		// read each curve
		x = extXval.front(); y = extYval.front(); z = extZval.front();
		value = distField[0][0][0];
		inGrid->setDataAt(0,0,0,value);
		inGrid->setLowerCornerXYZ(x, y, z);
		
		for(int i=0; i<dimx; i++){
			for(int j=0; j<dimy; j++){
				for(int k=0; k<dimz; k++){
					if(((i==0)&&(j==0)&&(z==0)) || ((i==(dimx-1))&&(j==(dimy-1))&&(z==(dimz-1)))){
						continue;
					}
					x = extXval[i];
					y = extYval[j];
					z = extZval[k];
					value = distField[i][j][k];
					inGrid->setDataAt(i,j,k,value);
					/*if(i<10 && j<10 && k<10){
						cout << i<< " "<< j << " "<< k<< " "<< value << " " << inGrid->getDataAt(i,j,k)<<endl;
					}*/
				}
			}
		}

		x = extXval.back(); y = extYval.back(); z = extZval.back();
		value = distField[dimx - 1][dimy - 1][dimz - 1];
		inGrid->setDataAt(dimx-1, dimy-1, dimz-1, value);
		inGrid->setUnitXYZ((x-inGrid->getLowerCorner(0))/(dimx-1), (y-inGrid->getLowerCorner(1))/(dimy-1), (z-inGrid->getLowerCorner(2))/(dimz-1));
		
	}


	static void findOrderedPairs(vector<vector<int> > &in_Pairs, vector<int> &in_PairsInd, int in_startPairID, int in_startEleID, vector<int>& out_PairIDs, vector<int>&out_EleIDs){
		out_PairIDs.push_back(in_PairsInd[in_startPairID]);
		out_EleIDs.push_back(in_startEleID);



		vector<vector<int> > leftPairs = in_Pairs;
		vector<int> leftPairInds = in_PairsInd;
		leftPairs.erase(leftPairs.begin()+in_startPairID);
		leftPairInds.erase(leftPairInds.begin()+in_startPairID);
		int preEleID = in_startEleID;

		while(leftPairs.size()!=0){ 
			int curPairID = 0;
			for(curPairID=0; curPairID<leftPairs.size(); curPairID++){
				if (leftPairs[curPairID][0]==preEleID){
					preEleID = leftPairs[curPairID][1];
					break;
				}
				else if (leftPairs[curPairID][1]==preEleID){
					preEleID = leftPairs[curPairID][0];
					break;
				}

			}
			// cout<<leftPairInds[curPairID]<<endl;
			// cout<<preEleID<<endl;
			out_PairIDs.push_back(leftPairInds[curPairID]);
			out_EleIDs.push_back(preEleID);
			leftPairs.erase(leftPairs.begin()+curPairID);
			leftPairInds.erase(leftPairInds.begin()+curPairID);
		}
	}


	static void findMergingOfTwoLists(vector<float> &list1, vector<float> &list2, vector<vector<int> > &merge1, vector<vector<int> > &merge2, float err){
		int n1 = (int)list1.size();
		int n2 = (int)list2.size();
		int i1=0, i2=0;
		merge1.resize(n1+1);
		merge2.resize(n2+1);
		while (i1<n1 && i2<n2){
			if (abs(list1[i1]-list2[i2]) < err){
				i1++; i2++;
			} else if(list1[i1]<list2[i2]){
				merge2[i2].push_back(i1);
				i1++;
			} else if(list1[i1]>list2[i2]){
				merge1[i1].push_back(i2);
				i2++;
			} 
		}
		if (i1==n1){
			for (int i=i2; i<n2; ++i){
				merge1[n1].push_back(i);
			}
		}else if (i2==n2){
			for (int i=i1; i<n1; ++i){
				merge2[n2].push_back(i);
			}
		}
	}

	static inline void readSkipNWord(ifstream &ifs, int n){
		string word;
		for(int i=0; i<n; ++i){
			ifs >> word;
		}
	}
	template <class T>
	static void read2DArrayVector(const char* filename, vector<vector<T> > &data){
		ifstream ifs(filename, ofstream::in);
		if (!ifs.good()) {
			cout << "Can not read file " << filename << endl;
			return;
		}

		int num;
		ifs >> num;
		data.resize(num);

		// read vertices
		for(int i=0; i<data.size(); i++){
			ifs >> num;
			data[i].resize(num);
			for(int j=0; j<num; ++j){
				ifs >> data[i][j];
			}
		}
		ifs.close();
	}
	template <class T>
	static void readAnArrayOfPairs(const char* filename, vector<vector<T> > &data){
		ifstream ifs(filename, ofstream::in);
		if (!ifs.good()) {
			cout << "Can not read file " << filename << endl;
			return;
		}

		int num;
		ifs >> num;
		data.resize(num);

		// read vertices
		for(int i=0; i<data.size(); i++){
			data[i].resize(2);
			for(int j=0; j<2; ++j){
				ifs >> data[i][j];
			}
		}
		ifs.close();
	}

	static bool readScaleFile(const char* filename, vector<float> &vec, float &scale){
		ifstream ifs(filename, ofstream::in);
		if (!ifs.good()) {
			cout << "No Scale file is provided or the file is not readable." << filename << endl;
			return false;
		}
		vec.resize(3);

		for(int i=0; i<3; i++){
			ifs >> vec[i];
		}
		ifs >> scale;
		ifs.close();

		return true;
	}
	static void scaleDnVers(vector<float> & ctrvers, const float scale, const vector<float> & translate){
		for(int j=0; j<ctrvers.size()/3; ++j){
			ctrvers[3*j] = ctrvers[3*j]/scale - translate[0];
			ctrvers[3*j+1] = ctrvers[3*j+1]/scale - translate[1];
			ctrvers[3*j+2] = ctrvers[3*j+2]/scale - translate[2];
		}
	}
	static void readTilingObj(const char* tilefile, vector<vector<double> > &sufV, vector<vector<int> > &sufF){
		ifstream ifs(tilefile, ofstream::in);
		if (!ifs.good()) 
		{
			cout << "Can not read OBJ file " << tilefile << endl;
			return;
		}
		string line;
		string line1;
		int nV, nF;
		// head
		/*ifs >> line;
		cout << line << endl;*/
		readSkipNWord(ifs, 6);
		// #V #F
		ifs >> line >> line1 >> nV;
		//cout << line << endl;
		//cout << line1 << endl;
		ifs >> line >> line1 >> nF;
		//cout << line << endl;
		//cout << line1 << endl;
		sufV.resize(nV, vector<double>(3));
		sufF.resize(nF, vector<int>(3));

		// read vertices
		for(int i=0; i<nV; i++){
			ifs >> line >> sufV[i][0] >>  sufV[i][1] >> sufV[i][2];
		}
		// read faces
		int v1, v2, v3;
		for(int i=0; i<nF; i++){
			ifs >> line >> v1 >>  v2 >> v3;
			sufF[i][0] = v1-1;
			sufF[i][1] = v2-1;
			sufF[i][2] = v3-1;
		}
		ifs.close();
	}

	// added by Lifan
	static void loadTilingObj(int numPoints, int numTris, float* pPositions, int* pFaceIndices, 
		vector<vector<double> > &sufV, vector<vector<int> > &sufF)
	{
		int nV, nF;
		nV = numPoints;
		nF = numTris;
		sufV.resize(nV, vector<double>(3));
		sufF.resize(nF, vector<int>(3));

		// read vertices
		for(int i=0; i<nV; i++){
			sufV[i][0] = pPositions[3 * i + 0];
			sufV[i][1] = pPositions[3 * i + 1];
			sufV[i][2] = pPositions[3 * i + 2];
		}
		// read faces
		for(int i=0; i<nF; i++){
			sufF[i][0] = pFaceIndices[3 * i + 0];
			sufF[i][1] = pFaceIndices[3 * i + 1];
			sufF[i][2] = pFaceIndices[3 * i + 2];
		}
	}

	static void saveTilingObj(const char* tilefile, vector<vector<double> > &sufV, vector<vector<int> > &sufF){
		ofstream writer(tilefile, ofstream::out);
		if (!writer.good()) {
			cout << "Can not write OBJ file " << tilefile << endl;
			return;
		}
		writer << "# OBJ File Generated by CycleGrouping\n";
		writer << "# Vertices: " << sufV.size() << "\n";
		writer << "# Faces: " << sufF.size() << "\n";
		// write vertices
		for(int i=0; i<sufV.size(); i++){
			writer << "v "<< sufV[i][0] << " " << sufV[i][1] << " " << sufV[i][2] <<"\n";
		}
		// write faces
		for (int i=0; i<sufF.size(); i++){
			writer << "f "<< sufF[i][0]+1 << " " << sufF[i][1]+1 << " " << sufF[i][2]+1 <<"\n";
		}
		writer.close();
	}

	static void saveTilingObj(const char* tilefile, vector<double> &sufV, vector<int> &sufF){
		ofstream writer(tilefile, ofstream::out);
		if (!writer.good()) {
			cout << "Can not write OBJ file " << tilefile << endl;
			return;
		}
		writer << "# OBJ File Generated by CycleGrouping\n";
		writer << "# Vertices: " << sufV.size()/3 << "\n";
		writer << "# Faces: " << sufF.size()/3 << "\n";
		// write vertices
		for(int i=0; i<sufV.size()/3; i++){
			writer << "v "<< sufV[i*3+0] << " " << sufV[i*3+1] << " " << sufV[i*3+2] <<"\n";
		}
		// write faces
		for (int i=0; i<sufF.size()/3; i++){
			writer << "f "<< sufF[i*3+0]+1 << " " << sufF[i*3+1]+1 << " " << sufF[i*3+2]+1 <<"\n";
		}
		writer.close();
	}

	static void saveResults(vector<double> &sufV, vector<int> &sufF ,
		int& _numPoints, int& _numTris, float** _pPositions, int** _pFaceIndices)
	{
		_numPoints = sufV.size() / 3;
		_numTris = sufF.size() / 3;
		float *pPos = new float [_numPoints * 3];
		int *pFace = new int [_numTris * 3];
		// write vertices
		for(int i=0; i<sufV.size(); i++){
			pPos[i] = sufV[i];
		}
		// write faces
		for (int i=0; i<sufF.size(); i++){
			pFace[i] = sufF[i];
		}
		*_pPositions = pPos;
		*_pFaceIndices = pFace;
	}

	// WRONG: specifically for scaling up input contours, pts is (3*# of points, i.e. {Xs, Ys, Zs}
	// RIGHT: ctrvers has the size of either contours or planes (doesn't matter), and each of the inner vector stores xyz of each vertex
	static void scaleUpVers(vector<vector<float> > & ctrvers, float scale, float translate []){
		for(int i=0; i<ctrvers.size(); ++i){
			for(int j=0; j<ctrvers[i].size()/3; ++j){
				ctrvers[i][3*j] = (ctrvers[i][3*j] + translate[0])*scale;
				ctrvers[i][3*j+1] = (ctrvers[i][3*j+1] + translate[1])*scale;
				ctrvers[i][3*j+2] = (ctrvers[i][3*j+2] + translate[2])*scale;
			}
		}
	}
	static void scaleUpBBox(float  bbox [], float scale, float translate []){
		for(int i=0; i<2; ++i){
			for(int j=0; j<3; ++j){
				bbox[i*3+j] = (bbox[i*3+j] + translate[j])*scale;
			}
		}
	}
	static void scaleUpParas(vector<float > & paras, float scale, float translate []){
		for(int i=0; i<paras.size()/4; ++i){
			paras[i*4+3] = (paras[i*4+3] 
			+ paras[i*4]*translate[0]
			+ paras[i*4+1]*translate[1]
			+ paras[i*4+2]*translate[2])*scale;
		}
	}
};

template <class T>
void Utility::writeVector(vector<vector<T> >& vec, ofstream& ofs){
	for (unsigned int i = 0; i < vec.size(); ++i){
		if (i == 0)	ofs << "{";
		else        ofs << ",{";
		for (unsigned int j = 0; j < vec[i].size(); ++j){
			if (j == 0)	ofs << vec[i][j];
			else        ofs << "," << vec[i][j];
		}
		ofs << "}";
	}
}

template <class T>
void Utility::writeVector(vector<T>& vec, ofstream& ofs){
	for (unsigned int j = 0; j < vec.size(); ++j){
		if (j == 0)	ofs << vec[j];
		else        ofs << "," << vec[j];
	}
}

template <class T>
void Utility::writeVector(T & vec, const char* filename){
	ofstream ofs(filename, ofstream::out);
	ofs << "{";
	writeVector(vec, ofs);
	ofs << "}";
	ofs.close();
}

template <class T>
void Utility::writeVectorForm(vector<vector<T> >& vec, ofstream& ofs){
	ofs << vec.size() << "\n";
	for (unsigned int i = 0; i < vec.size(); ++i){
		writeVectorForm(vec[i], ofs);
	}
}

template <class T>
void Utility::writeVectorForm(vector<T>& vec, ofstream& ofs){
	ofs << vec.size() << "\n";
	for (unsigned int j = 0; j < vec.size(); ++j){
		ofs << vec[j] << "\n";
	}
}

template <class T>
void Utility::writeVectorForm(T & vec, const char* filename){
	ofstream ofs(filename, ofstream::out);
	writeVectorForm(vec, ofs);
	ofs.close();
}

template <class T>
void Utility::printVector(vector<vector<T> >& in){
	for (int i = 0; i < in.size(); ++i){
		cout << "        <" << i << "> : ";
		for (T ele : in[i]){
			cout << ele << " ";
		}
		cout << endl;
	}
}

template <class T>
void Utility::printVector(vector<T>& in){
	for (T ele : in){
		cout << ele << " ";
	}
	cout << endl;
}

template <class T> 
string Utility::toStr(T& in){ 
	ostringstream os;
	os << in;
	return os.str();
}



template <class T>
void Utility::writeOutOneVector(const char* filename, T &vec){
	ofstream ofs(filename, ofstream::out);
	ofs << "{"; Utility::writeVector(vec, ofs); ofs << "}";
	ofs.close();
}

template <class T>
void Utility::flattenVectorsOfSameSize(const vector<vector<T> >& vec, vector<T> &res) {
	if(vec.size()==0) return;
    res.reserve(vec.size()*vec[0].size());
    //for (const auto& sub : vec)
        //res.insert(res.end(), sub.begin(), sub.end());
	for (int i = 0; i < vec.size(); i++)
	{
		res.insert(res.end() , vec[i].begin() , vec[i].end());
	}
}

}


#endif //_UTILITY_H_
