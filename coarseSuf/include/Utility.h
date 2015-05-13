#ifndef _COARSESUF_UTILITY_H_
#define _COARSESUF_UTILITY_H_

#include <vector>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>

#include "nvVector.h"
using namespace std;

namespace coarseSuf
{

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
	void normalize(float min, float max){
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

	void writeVolume(const char* fname)
	{
		int x = data.size();
		int y = data[0].size();
		int z = data[0][0].size();
		FILE *fp = fopen(fname , "w");
		fprintf(fp , "%d %d %d\n" , x , y , z);
		for (int i = 0; i < x; i++)
		{
			for (int j = 0 ; j < y; j++)
			{
				for (int k = 0; k < z; k++)
				{
					fprintf(fp , "%.6f %.6f %.6f %.6f\n" , 
						lowCornerXYZ[0] + i * unitXYZ[0] ,
						lowCornerXYZ[1] + j * unitXYZ[1] ,
						lowCornerXYZ[2] + k * unitXYZ[2] ,
						data[i][j][k]);
				}
			}
		}
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
		cout << "\nSaving Volume data to mrc file: " << fname << "..." << endl;
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

	enum INPARA {EXE = 0, INCURVENORM, INGRID, INPTS, INTRANS, OUTPC, OUTMRC, OUTSUF, W1, W2, W3, W4, W5};

	static void readLiFanCurveNorm(const char* filename, vector<vector<vec3d> >& inCurves, vector<vector<vec3d> >& inNorms){
		ifstream ifs(filename, ofstream::in);
		if (!ifs.good()) {
			cout << "Can not read file " << filename << endl;
			return;
		}
		cout << "\nReading curve-norm file from " << filename << " ..." << endl;
		int nCurve;
		ifs >> nCurve;
		inCurves.resize(nCurve);
		inNorms.resize(nCurve);
		// read each curve
		for(int i=0; i<nCurve; i++){
			int nPt;
			ifs >> nPt;
			inCurves[i].resize(nPt);
			inNorms[i].resize(nPt);
			for(int j=0; j<nPt; ++j){
				ifs >> inCurves[i][j].x >> inCurves[i][j].y >> inCurves[i][j].z >> inNorms[i][j].x >> inNorms[i][j].y >> inNorms[i][j].z;
			}
		}
		ifs.close();
	}

	static void readLiFanGrid(const char* filename, Volume* inGrid){
		ifstream ifs(filename, ofstream::in);
		if (!ifs.good()) {
			cout << "Can not read file " << filename << endl;
			return;
		}
		cout << "\nReading distance field file from " << filename << " ..." << endl;
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

	static void readLiFanNPTS(const char* filename, vector<vector<float> > &Pts){
		ifstream ifs(filename, ofstream::in);
		if (!ifs.good()) {
			cout << "Can not read file " << filename << endl;
			return;
		}
		cout << "\nReading npts file from " << filename << " ..." << endl;
		int nPt;
		float x, y, z, nx, ny, nz;
		Pts.reserve(10000);
		vector<float> onePt(3);
		string line;
		while(getline(ifs, line)){
			stringstream ss(line);
			ss >> x >> y >> z >> nx >> ny >> nz;
			onePt[0] = x;
			onePt[1] = y;
			onePt[2] = z;
			Pts.push_back(onePt);
		}
		ifs.close();
		cout << "  "<< Pts.size()  << " points are loaded from NPTS. "<< endl;
	}
	static void readLiFanTRANS(const char* filename, vector<float> &trans, float& scale){
		ifstream ifs(filename, ofstream::in);
		if (!ifs.good()) {
			cout << "Can not read file " << filename << endl;
			return;
		}
		cout << "\nReading translation-scale file from " << filename << " ..." << endl;
		trans.resize(3);
		ifs >> trans[0] >> trans[1] >> trans[2];
		ifs >> scale;
		ifs.close();
		cout << "  trans = (" << trans[0] << ", " << trans[1] << ", " << trans[2] << ")" << endl;
		cout << "  scale = " << scale << endl;
	}
	static void saveMeshObj(const char* tilefile, vector<vector<float> > &sufV, vector<vector<int> > &sufF){
		
		ofstream writer(tilefile, ofstream::out);
		if (!writer.good()) {
			cout << "Can not write OBJ file " << tilefile << endl;
			return;
		}
		cout << "\nWriting obj file to " << tilefile << " ..." << endl;
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
	static float getVolValAtPos(float xyz[], Volume * volInfo){

		float xyz2 [3];
		xyz2[0] = xyz[0];
		xyz2[1] = xyz[1];
		xyz2[2] = xyz[2];

		int xyz0 [3];
		int xyz1 [3];
		float xyzD1 [3];
		float xyzD0 [3];
		for(int i=0; i<3; ++i){
			//float val = 1.0f*VOLCELLN*(xyz[i]-cellFrameLowerCorner[i])/cellFrameScale[i];
			//float val = 1.0f*VOLCELLN*(xyz[i]-volLowerCorner[i])/volScale[i];
			//float val = 1.0f*(xyz[i]-volInfo.getLowerCorner(i))/volInfo.getUnit(i);
			float val = 1.0f*(xyz2[i]-volInfo->getLowerCorner(i))/volInfo->getUnit(i);
			val = val < 0.0f ? 0.0f : val; //need this, because of numerical issue
			xyz0[i] = floor(val);
			//xyz0[i] = xyz0[i] < 0 ? 0 : xyz0[i]; 
			//xyz1[i] = xyz0[i] == VOLCELLN ? VOLCELLN : xyz0[i]+1;
			xyz1[i] = xyz0[i] == (volInfo->getGridSize(i)-1) ? (volInfo->getGridSize(i)-1) : xyz0[i]+1;
			xyzD1[i] = val - xyz0[i];
			xyzD0[i] = 1 - xyzD1[i];
		}
		float val = 
			volInfo->getDataAt(xyz0[0],xyz0[1],xyz0[2])*xyzD0[0]*xyzD0[1]*xyzD0[2]
		+   volInfo->getDataAt(xyz1[0],xyz0[1],xyz0[2])*xyzD1[0]*xyzD0[1]*xyzD0[2]
		+   volInfo->getDataAt(xyz1[0],xyz1[1],xyz0[2])*xyzD1[0]*xyzD1[1]*xyzD0[2]
		+   volInfo->getDataAt(xyz0[0],xyz1[1],xyz0[2])*xyzD0[0]*xyzD1[1]*xyzD0[2]
		+   volInfo->getDataAt(xyz0[0],xyz0[1],xyz1[2])*xyzD0[0]*xyzD0[1]*xyzD1[2]
		+   volInfo->getDataAt(xyz1[0],xyz0[1],xyz1[2])*xyzD1[0]*xyzD0[1]*xyzD1[2]
		+   volInfo->getDataAt(xyz1[0],xyz1[1],xyz1[2])*xyzD1[0]*xyzD1[1]*xyzD1[2]
		+   volInfo->getDataAt(xyz0[0],xyz1[1],xyz1[2])*xyzD0[0]*xyzD1[1]*xyzD1[2];

		return val;
	}
};

}

#endif 
