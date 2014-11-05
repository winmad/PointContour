// #include <math.h> 
#include <cmath>
#define _USE_MATH_DEFINES

#include <time.h>

#include <cstdlib>
#include <algorithm>
#include <float.h>
// #include "DrT.h"
#include <iostream>
#include <fstream>
#include <map>
#include <algorithm>

#include "cycleDiscovery.h"

#include <boost/config.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
using namespace boost;

namespace cycle{
//boost graph
typedef adjacency_list < vecS, vecS, undirectedS,
	no_property, property < edge_weight_t, int > > graph_t;
typedef graph_traits < graph_t >::vertex_descriptor vertex_descriptor;
typedef graph_traits < graph_t >::edge_descriptor edge_descriptor;
typedef std::pair<int, int> Edge;

#define USEBOOST 1

void cycleDiscovery(std::vector<std::vector<Point> > &curveNet, std::vector<std::vector<int> > &cycles, std::vector<Point>& pointCloud)
{
	cycleUtils* m_cycleUtil = new cycleUtils;
	m_cycleUtil->constructNetwork(curveNet);
	m_cycleUtil->constructRotationGraphbyPoleGraphEx();
	m_cycleUtil->constructCycles();
	m_cycleUtil->cycleBreaking();
	cycles.resize(m_cycleUtil->m_cycleSetBreaked.size());
	for(unsigned i=0;i<cycles.size();i++){
		for(unsigned j=0;j<m_cycleUtil->m_cycleSetBreaked[i].size();j++){
			cycles[i].push_back(m_cycleUtil->m_cycleSetBreaked[i][j].arcID);
		}
	}
}

void cycleTest()
{
    std::vector<std::vector<vec3d> > curves;
    std::vector<std::vector<int> > cycles;
    std::vector<vec3d> curve;
    curve.resize(2);
    curve[0] = vec3d(0 , 0 , 0); curve[1] = vec3d(0 , 0 , 1);
    curves.push_back(curve);
    curve[1] = vec3d(0 , 1 , 0);
    curves.push_back(curve);
    curve[1] = vec3d(1 , 0 , 0);
    curves.push_back(curve);
    curve[0] = vec3d(0 , 0 , 1); curve[1] = vec3d(0 , 1 , 0);
    curves.push_back(curve);
    curve[1] = vec3d(1 , 0 , 0);
    curves.push_back(curve);
    curve[0] = vec3d(0 , 1 , 0);
    curves.push_back(curve);
    cycle::cycleDiscovery(curves , cycles , curve);
    for (int i = 0; i < cycles.size(); i++)
    {
        printf("===== cycle %d =====\n" , i);
        for (int j = 0; j < cycles[i].size(); j++) printf("%d " , cycles[i][j]);
        printf("\n");
    }
}

GraphSearch::GraphSearch()
{
}
GraphSearch::~GraphSearch()
{
}

double GraphSearch::computeGeneralHamiltonGraph(const Capacity &org1,
												const ArcWithWeight &org2,
												const bool &org3,
												const bool &org4,
												AllMultipleArcs &tar,
												std::vector<double> &tar2)
{
	Capacity capacity=org1;
	ArcWithWeight arcsWithWeights=org2;
	bool connectRequire = org3;
	bool isFindAllSolutions = org4;
	MultipleArcs usedArcsID;
	AllMultipleArcs allUsedArcsID;
	std::vector<double> nodeWeights;
	usedArcsID.clear();

	State initState;
	std::vector<State > queueState;

	initState.usedArcs.clear();
	initState.availableCapacity = capacity;
	initState.nextArcID = 0;
	initState.weight = 0;

	queueState.push_back(initState);

	double minCost=FLT_MAX;
	while(true){
		double minWeight= FLT_MAX;
		int minID =-1;
		for(int i=0;i<queueState.size();i++){
			if(queueState[i].weight<minWeight){
				minID=i; minWeight=queueState[i].weight;
			}
		}
		if(minID==-1){
			usedArcsID.clear();
			break;}
		State currentState = queueState[minID];
		queueState.erase(queueState.begin()+minID);

		if(*(std::max_element(currentState.availableCapacity.begin(),
			currentState.availableCapacity.end()))==0){

				if(connectRequire){
					usedArcsID = currentState.usedArcs;
					std::vector<std::pair<int,int> > usedArcs;
					for(int i=0;i<usedArcsID.size();i++){
						int n1 = arcsWithWeights[usedArcsID[i].first].second.first;
						int n2 = arcsWithWeights[usedArcsID[i].first].second.second;
						usedArcs.push_back(std::pair<int,int>(n1,n2));
					}
					std::vector<int> usedNodes;
					usedNodes.push_back(usedArcs[0].first);
					usedNodes.push_back(usedArcs[0].second);
					usedArcs.erase(usedArcs.begin());
					while(!usedArcs.empty()){
						bool isChanged = false;
						int nodeSize=usedNodes.size();
						for(int j=0;j<nodeSize;j++){
							for(int k=0;k<usedArcs.size();k++){
								if(usedNodes[j]==usedArcs[k].first){
									if(find(usedNodes.begin(),usedNodes.end(),usedArcs[k].second)
										== usedNodes.end())
										usedNodes.push_back(usedArcs[k].second);
									usedArcs.erase(usedArcs.begin()+k);
									isChanged=true;
									break;
								}
								if(usedNodes[j]==usedArcs[k].second){
									if(find(usedNodes.begin(),usedNodes.end(),usedArcs[k].first)
										== usedNodes.end())
										usedNodes.push_back(usedArcs[k].first);
									usedArcs.erase(usedArcs.begin()+k);
									isChanged=true;
									break;
								}
							}
						}
						if(usedNodes.size()==capacity.size())
							break;
						if(!isChanged)break;
					}
					if(usedNodes.size()!=capacity.size())
						continue;
				}
				usedArcsID = currentState.usedArcs;
				allUsedArcsID.push_back(usedArcsID);
				nodeWeights.push_back(currentState.weight);
				if(isFindAllSolutions==false){
					minCost = currentState.weight;
					break;
				}
		}
		else{
			//expand current state;
			int nodeID[2];
			while(currentState.nextArcID<arcsWithWeights.size()){
				nodeID[0]=arcsWithWeights[currentState.nextArcID].second.first;
				nodeID[1]=arcsWithWeights[currentState.nextArcID].second.second;
				if(	currentState.availableCapacity[nodeID[0]]>=1
					&&currentState.availableCapacity[nodeID[1]]>=1)
					break;
				currentState.nextArcID++;
			}
			if(currentState.nextArcID<arcsWithWeights.size()){
				int degree = currentState.availableCapacity[nodeID[0]]<=
					currentState.availableCapacity[nodeID[1]]?
					currentState.availableCapacity[nodeID[0]]:
				currentState.availableCapacity[nodeID[1]];
				State saveState = currentState;
				int usedArcsID,isUsed=0;
				//currentState.weight += arcsWithWeights[currentState.nextArcID].first;
				currentState.nextArcID++;
				queueState.push_back(currentState);
				currentState=saveState;
				while(degree>0){
					currentState.availableCapacity[nodeID[0]]--;
					currentState.availableCapacity[nodeID[1]]--;
					//	int usedArcsID,isUsed=0;
					for(usedArcsID=0;usedArcsID<currentState.usedArcs.size();usedArcsID++){
						if(currentState.usedArcs[usedArcsID].first == (currentState.nextArcID)){
							isUsed=1;break;}
					}
					if(isUsed==1)
						currentState.usedArcs[usedArcsID].second++;
					else
						currentState.usedArcs.push_back(std::pair<int,int>(currentState.nextArcID,1));

					currentState.weight += arcsWithWeights[currentState.nextArcID].first;
					currentState.nextArcID++;
					queueState.push_back(currentState);
					currentState.nextArcID--;
					degree--;
				}
				currentState.nextArcID++;
			}
		}//end of else
	}//end of while

	tar = allUsedArcsID;
	tar2 = nodeWeights;
	return minCost;
}

double GraphSearch::computeGeneralHamiltonGraph(const Capacity &org1,
								 const ArcWithWeight &org2, const PairArcsWithWeight & org3,
								 const double org4, const bool &org5, const bool &org6,
								 AllMultipleArcs &tar, std::vector<double> &tar2)
{
	Capacity capacity=org1;
	ArcWithWeight arcsWithWeights=org2;
	PairArcsWithWeight pairArcsWithWeights= org3;
	double balance = org4;
	bool connectRequire = org5;
	bool isFindAllSolutions = org6;
	MultipleArcs usedArcsID;
	AllMultipleArcs allUsedArcsID;
	std::vector<double> nodeWeights;
	usedArcsID.clear();

	StateExtend initState;
	std::vector<StateExtend > queueState;

	initState.usedArcs.clear();
	initState.availableCapacity = capacity;
	initState.nextArcID = 0;
	initState.totalWeight = 0;
	initState.dihedralWeights.assign(capacity.size(),0);
	initState.adjacentArcs.resize(capacity.size());

	queueState.push_back(initState);

	double minCost=FLT_MAX;
	while(true){
		double minWeight= FLT_MAX;
		int minID =-1;
		for(int i=0;i<queueState.size();i++){
			if(queueState[i].totalWeight<minWeight){
				minID=i; minWeight=queueState[i].totalWeight;
			}
		}
		if(minID==-1){
			usedArcsID.clear();	break;}

		StateExtend currentState = queueState[minID];
		queueState.erase(queueState.begin()+minID);

		if(*(std::max_element(currentState.availableCapacity.begin(),
			currentState.availableCapacity.end()))==0){

				if(connectRequire){
					usedArcsID = currentState.usedArcs;
					std::vector<std::pair<int,int> > usedArcs;
					for(int i=0;i<usedArcsID.size();i++){
						int n1 = arcsWithWeights[usedArcsID[i].first].second.first;
						int n2 = arcsWithWeights[usedArcsID[i].first].second.second;
						usedArcs.push_back(std::pair<int,int>(n1,n2));
					}
					std::vector<int> usedNodes;
					usedNodes.push_back(usedArcs[0].first);
					usedNodes.push_back(usedArcs[0].second);
					usedArcs.erase(usedArcs.begin());
					while(!usedArcs.empty()){
						bool isChanged = false;
						int nodeSize=usedNodes.size();
						for(int j=0;j<nodeSize;j++){
							for(int k=0;k<usedArcs.size();k++){
								if(usedNodes[j]==usedArcs[k].first){
									if(find(usedNodes.begin(),usedNodes.end(),usedArcs[k].second)
										== usedNodes.end())
										usedNodes.push_back(usedArcs[k].second);
									usedArcs.erase(usedArcs.begin()+k);
									isChanged=true;
									break;
								}
								if(usedNodes[j]==usedArcs[k].second){
									if(find(usedNodes.begin(),usedNodes.end(),usedArcs[k].first)
										== usedNodes.end())
										usedNodes.push_back(usedArcs[k].first);
									usedArcs.erase(usedArcs.begin()+k);
									isChanged=true;
									break;
								}
							}
						}
						if(usedNodes.size()==capacity.size())
							break;
						if(!isChanged)break;
					}
					if(usedNodes.size()!=capacity.size())
						continue;
				}
				usedArcsID = currentState.usedArcs;
				allUsedArcsID.push_back(usedArcsID);
				nodeWeights.push_back(currentState.totalWeight);
				if(isFindAllSolutions==false){
					minCost = currentState.totalWeight;
					break;
				}
		}
		else{
			//expand current state;
			int nodeID[2];
			while(currentState.nextArcID<arcsWithWeights.size()){
				nodeID[0]=arcsWithWeights[currentState.nextArcID].second.first;
				nodeID[1]=arcsWithWeights[currentState.nextArcID].second.second;
				if(	currentState.availableCapacity[nodeID[0]]>=1
					&&currentState.availableCapacity[nodeID[1]]>=1)
					break;
				currentState.nextArcID++;
			}
			if(currentState.nextArcID<arcsWithWeights.size()){
				int degree = currentState.availableCapacity[nodeID[0]]<=
							currentState.availableCapacity[nodeID[1]]?
							currentState.availableCapacity[nodeID[0]]:
							currentState.availableCapacity[nodeID[1]];
				StateExtend saveState = currentState;
				currentState.nextArcID++;
				queueState.push_back(currentState);
				currentState=saveState;
				while(degree>0){
					currentState.availableCapacity[nodeID[0]]--;
					currentState.availableCapacity[nodeID[1]]--;
					//	int usedArcsID,isUsed=0;
					int usedArcsID,isUsed=0;
					for(usedArcsID=0;usedArcsID<currentState.usedArcs.size();usedArcsID++){
						if(currentState.usedArcs[usedArcsID].first == (currentState.nextArcID)){
							isUsed=1;break;}
					}
					if(isUsed==1)
						currentState.usedArcs[usedArcsID].second++;
					else
						currentState.usedArcs.push_back(std::pair<int,int>(currentState.nextArcID,1));

					currentState.totalWeight += arcsWithWeights[currentState.nextArcID].first;

					//update weight;
					for(int n=0;n<2;n++){
						double w=0;
						for(int i=0;i<currentState.adjacentArcs[nodeID[n]].size();i++){
							if(w<pairArcsWithWeights[currentState.nextArcID][currentState.adjacentArcs[nodeID[n]][i]])
								w=pairArcsWithWeights[currentState.nextArcID][currentState.adjacentArcs[nodeID[n]][i]];
						}
						if(w>currentState.dihedralWeights[nodeID[n]]){
							currentState.totalWeight+=balance*(w-currentState.dihedralWeights[nodeID[n]]);
						}
						else{
							currentState.dihedralWeights[nodeID[n]]=w;
						}
						currentState.adjacentArcs[nodeID[n]].push_back(currentState.nextArcID);
					}

					currentState.nextArcID++;
					queueState.push_back(currentState);
					currentState.nextArcID--;
					degree--;
				}
				currentState.nextArcID++;
			}
		}//end of else
	}//end of while

	tar = allUsedArcsID;
	tar2 = nodeWeights;
	return minCost;
}


std::vector<std::vector<int> >  enumSets(const std::vector<int> &capacity,
										 const int &k)
{
	std::vector<std::vector<int> > res;
	if(capacity.size()==1){
		res.push_back(std::vector<int>(1,k));
		return res;
	}
	else{
		std::vector<int> ncaps = capacity;
		ncaps.erase(ncaps.begin());
		if(capacity.front()==0){
			res = enumSets(ncaps,k);
			for(int i=0;i<res.size();i++)
				res[i].insert(res[i].begin(),0);
			return res;
		}
		else{
			int l=0;
			for(int i=0;i<ncaps.size();i++)
				l+=ncaps[i];
			l = std::max(0,k-l);
			int h = std::min(k,capacity.front());
			if(l>h){
				int t = l;
				l = h;
				h = t;
			}
			for(int i=l;i<=h;i++){
				std::vector<std::vector<int> > subres = enumSets(ncaps,k-i);
				for(int j=0;j<subres.size();j++){
					subres[j].insert(subres[j].begin(),i);
					res.push_back(subres[j]);
				}
			}
			return res;
		}
	}
}
void GraphSearch::computeGeneralHamiltonGraph(	const Capacity &capacity,
												const std::vector<std::pair<int,int> > &allArcs,
												const std::vector<double> &weights,
												const int K, const bool isConnect,
												const std::vector<std::pair<int,int> > &predefinedArcs,
												AllMultipleArcs &tar,
												std::vector<double> &tar2)
{
	MultipleArcs usedArcsID;
	AllMultipleArcs allUsedArcsID;
	std::vector<double> nodeWeights;

	//build up adjacency;
	int lenN = capacity.size();
	int lenA = allArcs.size();
	std::vector<std::vector<int> > adjnodes(lenN);
	std::vector<std::vector<int> > adjarcs(lenN);
	std::vector<std::vector<int> > adjnodeinds(lenN);
	std::vector<std::vector<double> > adjcosts(lenN);
	for(int i=0;i<lenA;i++){
		int n1 = allArcs[i].first;
		int n2 = allArcs[i].second;
		adjarcs[n1].push_back(i);
		adjarcs[n2].push_back(i);
		adjcosts[n1].push_back(weights[i]);
		adjcosts[n2].push_back(weights[i]);
		adjnodes[n1].push_back(n2);
		adjnodes[n2].push_back(n1);
		adjnodeinds[n1].push_back(adjnodes[n2].size()-1);
		adjnodeinds[n2].push_back(adjnodes[n1].size()-1);
	}

	double minArcCost = *std::min_element(weights.begin(),weights.end());

	StateEx2 initState;
	std::vector<StateEx2 > queueState;

	initState.usage = adjnodes;
	for(int i=0;i<adjnodes.size();i++){
		for(int j=0;j<adjnodes[i].size();j++){
			initState.usage[i][j] = 0;
		}
	}
	initState.availableCapacity = capacity;
	initState.cost = 0;
	initState.hcost = 0;

	std::vector<std::pair<int,int> > preArcs = predefinedArcs;
	for(int i=0;i<preArcs.size();i++){
		int n1=preArcs[i].first;
		int n2=preArcs[i].second;
		initState.availableCapacity[n1]--;
		initState.availableCapacity[n2]--;

		int n12 = std::find(adjnodes[n1].begin(),adjnodes[n1].end(),n2)-adjnodes[n1].begin();
		int n21 = std::find(adjnodes[n2].begin(),adjnodes[n2].end(),n1)-adjnodes[n2].begin();
		initState.usage[n1][n12]++;
		initState.usage[n2][n21]++;

		/*add weight to predefined arcs; */
		//
	}

	queueState.push_back(initState);

	int count=0;
	while(true){
		if(queueState.empty())
			break;
		count++;
		double minWeight= FLT_MAX;
		double validWeight= FLT_MAX;
		int minID =-1;
		for(int i=0;i<queueState.size();i++){
			if(queueState[i].hcost<minWeight){
				minID=i; minWeight=queueState[i].hcost;
			}
		}
		StateEx2 currentState = queueState[minID];
		queueState.erase(queueState.begin()+minID);

		if(*(std::max_element(currentState.availableCapacity.begin(),
			currentState.availableCapacity.end()))==0){
			usedArcsID.clear();
			for(int i=0;i<lenN;i++){
				for(int j=0;j<adjnodes[i].size();j++){
					if(adjnodes[i][j]>i && currentState.usage[i][j]>0)
						usedArcsID.push_back(std::pair<int,int>(adjarcs[i][j],
							currentState.usage[i][j]));
				}
			}
			allUsedArcsID.push_back(usedArcsID);
			nodeWeights.push_back(currentState.cost);
			if(allUsedArcsID.size()==K)
				break;
			else
				continue;
		}
		else{
			//expand current state;
			std::vector<int> partialNodes;
			std::vector<int> newNode;
			if(count==1)
				partialNodes.push_back(0);
			else{ 
				for(int i=0;i<lenN;i++){
					if(currentState.availableCapacity[i]>0 &&
						currentState.availableCapacity[i]<capacity[i])
						partialNodes.push_back(i);
					else{
						if(currentState.availableCapacity[i]>0)
							newNode.push_back(i);
					}
				}
			}
/*
			if(partialNodes.empty())
				continue;
*/
			int nodeind;
			if(partialNodes.empty()){
				if(isConnect||newNode.empty())
					continue;
				else
					nodeind = newNode.front();
			}
			else
				nodeind = partialNodes.front();

			std::vector<int> adjcaps;
			for(int i=0;i<adjnodes[nodeind].size();i++){
/*	
				if(currentState.usage[nodeind][i]>0) //can't do this if. it fails for jetfighter, when arc(1,2,3) with cap(2,2,4)
													// and a constraint(arc1,arc3).  Crz, in this case no connect between arc(1,3),
													//but the only state will start by using arc(1,3);
					adjcaps.push_back(0);
				else
*/
					adjcaps.push_back(currentState.availableCapacity[adjnodes[nodeind][i]]);
			}
			int totalcap=0;
			for(int i=0;i<adjcaps.size();i++)
				totalcap+=adjcaps[i];
			if(totalcap<currentState.availableCapacity[nodeind])
				continue;

			std::vector<std::vector<int> > nusage;
			nusage = enumSets(adjcaps,currentState.availableCapacity[nodeind]);

			currentState.availableCapacity[nodeind]=0;
			StateEx2 tstate =currentState;
			for(int i=0;i<nusage.size();i++){
				currentState = tstate;
				for(int j=0;j<nusage[i].size();j++)
					currentState.usage[nodeind][j]+=nusage[i][j];

				for(int j=0;j<nusage[i].size();j++){
					if(nusage[i][j]>0){
						currentState.availableCapacity[adjnodes[nodeind][j]]
							-=nusage[i][j];
						currentState.usage[adjnodes[nodeind][j]][adjnodeinds[nodeind][j]]
							+=nusage[i][j];
					}
				}
				for(int j=0;j<nusage[i].size();j++)
					currentState.cost+=nusage[i][j]*adjcosts[nodeind][j];
				int totalavail=0;
				for(int j=0;j<currentState.availableCapacity.size();j++)
					totalavail+=currentState.availableCapacity[j];
				currentState.hcost=currentState.cost+minArcCost*totalavail/2;

				queueState.push_back(currentState);				
			}
		}
	}
	tar = allUsedArcsID;
	tar2 = nodeWeights;
}


std::vector<std::vector<int> > ordering(const std::vector<std::vector<int> > &data)
{
	std::vector<std::vector<int> > rank(data.size(),std::vector<int>(data.size(),0));
	for(int i=0;i<data.size();i++){
		std::vector<int> tdat = data[i];
		for(int j=0;j<tdat.size();j++){
			int minInd = j;
			for(int k=0;k<tdat.size();k++){
				if(tdat[minInd]>tdat[k])
					minInd = k;
			}
			rank[i][minInd]=j;
			tdat[minInd] = INT_MAX;
		}
	}
	return rank;
}

void GraphSearch::stableMatching(const std::vector<std::vector<int> > &firstGroup,
					const std::vector<std::vector<int> > &secondGroup,
					const std::vector<std::pair<int,int> > &predefinedPairs, std::vector<int> &res)
{
	int size = firstGroup.size();
	std::vector<std::vector<int> > rank1 = ordering(firstGroup);
	std::vector<std::vector<int> > rank2 = ordering(secondGroup);
	std::vector<int> mate1(size,-1);
	std::vector<int> mate2(size,-1);

	std::vector<std::vector<int> > unproposed = firstGroup;
	std::vector<int> freelist(size);
	for(int i=0;i<size;i++)
		freelist[i]=i;

	std::vector<std::pair<int,int> > prePairs = predefinedPairs;
	for(int i=0;i<prePairs.size();i++){
		int m = prePairs[i].first;
		int w = prePairs[i].second;

		freelist.erase(freelist.begin()+(std::find(freelist.begin(),freelist.end(),m)-freelist.begin()));
		unproposed[m].clear();
		for(int j=0;j<unproposed.size();j++){
			if(unproposed[j].empty())
				continue;
			unproposed[j].erase(unproposed[j].begin()+(std::find(unproposed[j].begin(),unproposed[j].end(),w)-unproposed[j].begin()));
		}
		mate2[w]=m;
		mate1[m]=w;
	}

	while(!freelist.empty()){
		int m1 = freelist.front();
		int w = unproposed[m1].front();
		unproposed[m1].erase(unproposed[m1].begin());
		int m2 = mate2[w];
		if(m2==-1){
			mate2[w]=m1;
			mate1[m1]=w;
			freelist.erase(freelist.begin());
		}
		else{
			if(rank2[w][m1]<rank2[w][m2]){
				mate2[w]=m1;
				mate1[m1]=w;
				freelist.erase(freelist.begin());
				freelist.push_back(m2);
			}
		}
	}
	res = mate1;
}

cycleUtils::cycleUtils()
{
	m_pointCloseness=0.001;
	m_pointSnapThreshold=0.003;
	m_curveCloseness=0.003;
	m_isComputeIntersection=true;
	m_isSmothing=true;
	m_isDeleteDuplicateArc=true;
	m_isDeleteBranch=true;

	m_curveNetworkOriginal.clear();
	m_curveCapacitys.clear();
	m_boundingBoxMin.x=m_boundingBoxMin.y=m_boundingBoxMin.z=0;
	m_boundingBoxMax.x=m_boundingBoxMax.y=m_boundingBoxMax.z=0;
	m_curveNet.nodes.clear();
	m_curveNet.arcs.clear();

	m_twistNormNum=30;
	m_angleWeight=1;
	m_twistWeight=1;
	m_curveWeight=0;
	m_nodeWeight=0;
	m_bestNeightboreNum=1;
	m_rotationGraphNum=10;
	m_dihedralWeight=.5;
	m_stateNum=100;
	m_stateUnLimited=false;
	m_isRoGraphConnect = false;
	m_isDoCycleBreak = true;
	m_weightTri=m_weightEdge=0;
	m_weightBiTri=m_weightTriBd=1;
	m_weightWorsDih=0;
	m_SurfaceSmooth=true;
	m_subdivisonSmooth=1;
	m_laplacianSmooth=3;

	m_twistTables.clear();
	m_twistTablesConfidence.clear();
	m_twistTablesIndex.clear();
	m_selectArcList.clear();
	m_selectArcList.resize(1);
	m_userDefinedPairsInNode.clear();
	m_userDefinedPairsInArc.clear();
	m_latestUpdateNodes.clear();
	m_latestUpdateArcs.clear();
	m_latestUpdateArcsCapacity.clear();
	m_suggestCapacity.clear();
	m_errorNode.clear();
	m_poleGraphNodes.clear();
	m_poleGraphNodeWeight.clear();
	m_poleGraphArcsWeight.clear();
	m_poleGraphArcsMatch.clear();
	m_expandPoleSequence.clear();
	m_selectedNodeInPole.clear();

	m_rotationGraph.clear();
	m_rotationGraphUpdate.clear();

	m_isCycleBreak=false;
	m_isCycleCorrect=true;
	m_cycleSet.clear();
	m_cycleSetBreaked.clear();
	m_breakNum=0;

	m_cyclesCost.clear();
	m_arcsCost.clear();

	m_normalsTable.clear();
	m_triangleSurface.clear();
	m_triangleSurfaceNormal.clear();
	m_cycleNormal.clear();
	m_isDihedral=false;

	m_newPoints.clear();
	m_newNormals.clear();
	m_newPointNum.clear();
}
cycleUtils::~cycleUtils()
{
}
std::vector<int> ordering(const std::vector<double> &data)
{
	std::vector<int> rank(data.size());
	std::vector<double> tdat = data;
	for(int j=0;j<tdat.size();j++){
		int minInd = j;
		for(int k=0;k<tdat.size();k++){
			if(tdat[minInd]>tdat[k])
				minInd = k;
		}
		rank[minInd]=j;
		tdat[minInd] = FLT_MAX;
	}
	return rank;
}

void cycleUtils::constructNetwork(std::vector<std::vector<Point> > &curveNet)
{
	m_curveNetworkOriginal=curveNet;
	m_curveCapacitys.resize(m_curveNetworkOriginal.size(),2);

	LinearCurveNet curves = m_curveNetworkOriginal;
	if(m_isSmothing){
		std::vector<std::vector<Point> > fairCurve = curves;
		for(int times=0;times<3;times++){
			for(int i=0;i<curves.size();i++){
				if(curves[i].size()<3)continue;
				for(int j=1;j<curves[i].size()-1;j++){
					Point point = curves[i][j];
					Point leftPoint = curves[i][j-1];
                    Point rightPoint= curves[i][j+1];
					Point centrePoint = (leftPoint+rightPoint)/2.0;
					point = (point+centrePoint)/2.0;
					fairCurve[i][j]=point;
				}
			}
			curves=fairCurve;
		}
		fairCurve.clear();
	}

	m_curveNet.nodes.clear();
	m_curveNet.arcs.clear();
	Graph& net = m_curveNet;

	std::vector<int>& capacity = m_curveCapacitys;

	std::map< std::vector<double>,int> mapPointToIndex;
	std::map< std::vector<double>,int>::iterator itMap;

	int index=0;
	int len = curves.size();
	std::vector<double> stdPoint(3);
	for(int i=0;i<len;i++)
	{
		Curve curve = curves[i];
		std::vector<Point> endNodes; endNodes.push_back(curve.front()); endNodes.push_back(curve.back());
		curve.erase(curve.begin());
		int sizeCurve = curve.size();
		curve.erase(curve.begin()+sizeCurve-1);

		int currentIndex[2];
		for(int j=0;j<2;j++){
			for(unsigned k=0;k<3;k++) stdPoint[k] = endNodes[j][k];
			itMap = mapPointToIndex.find(stdPoint);
			if (itMap != mapPointToIndex.end()){
				currentIndex[j] = itMap->second;
				net.nodes[currentIndex[j]].arcID.push_back(i);
				net.nodes[currentIndex[j]].arcDirection.push_back(j+1);
			}
			else {
				mapPointToIndex.insert(std::pair< std::vector<double>,int>(stdPoint,index));
				currentIndex[j] = index;
				index++;
				GraphNode node;
				node.pos = endNodes[j];
				node.arcID.push_back(i);
				node.arcDirection.push_back(j+1);
				net.nodes.push_back(node);
			}
		}
		GraphEdge arc;
		arc.vertexList = curve;
		arc.capacity=capacity[i];
		arc.endNodesID.first=currentIndex[0];
		arc.endNodesID.second=currentIndex[1];
		if(currentIndex[0]==currentIndex[1])
			arc.posInNode.first = net.nodes[currentIndex[0]].arcID.size()-2;
		else
			arc.posInNode.first = net.nodes[currentIndex[0]].arcID.size()-1;
		arc.posInNode.second = net.nodes[currentIndex[1]].arcID.size()-1;
		net.arcs.push_back(arc);
	}
}
void cycleUtils::deleteNodeWithTwoDegree()
{
	if(m_curveNetworkOriginal.empty()||m_curveNet.arcs.empty())
		return;

	LinearCurveNet curves = m_curveNetworkOriginal;
	Graph net = m_curveNet;

	for(int i=0;i<net.nodes.size();i++){
		if(net.nodes[i].arcID.size()==2){
			int firstArc = net.nodes[i].arcID[0];
			int secondArc = net.nodes[i].arcID[1];
			if(firstArc==secondArc)
				continue;

			Curve firstCurve=net.arcs[firstArc].vertexList;
			Point p = net.nodes[net.arcs[firstArc].endNodesID.first].pos;
			firstCurve.insert(firstCurve.begin(),p);
			p = net.nodes[net.arcs[firstArc].endNodesID.second].pos;
			firstCurve.push_back(p);
			if(i==net.arcs[firstArc].endNodesID.first)
				reverse(firstCurve.begin(),firstCurve.end());

			Curve secondCurve=net.arcs[secondArc].vertexList;
			p = net.nodes[net.arcs[secondArc].endNodesID.first].pos;
			secondCurve.insert(secondCurve.begin(),p);
			p = net.nodes[net.arcs[secondArc].endNodesID.second].pos;
			secondCurve.push_back(p);

			int adjacentNode,posInNode;
			if(i==net.arcs[secondArc].endNodesID.second){
				reverse(secondCurve.begin(),secondCurve.end());
				adjacentNode=net.arcs[secondArc].endNodesID.first;
				posInNode = net.arcs[secondArc].posInNode.first;
			}
			else{
				adjacentNode=net.arcs[secondArc].endNodesID.second;
				posInNode = net.arcs[secondArc].posInNode.second;
			}

			secondCurve.erase(secondCurve.begin());
			firstCurve.insert(firstCurve.begin()+firstCurve.size(),
				secondCurve.begin(),secondCurve.end());
			curves[firstArc]=firstCurve;
			curves[secondArc].clear();

			firstCurve.erase(firstCurve.begin());
			firstCurve.erase(firstCurve.begin()+firstCurve.size()-1);
			net.nodes[adjacentNode].arcID[posInNode] = firstArc;
			if(i==net.arcs[firstArc].endNodesID.first){
				reverse(firstCurve.begin(),firstCurve.end());
				net.arcs[firstArc].endNodesID.first = adjacentNode;}
			else{
				net.arcs[firstArc].endNodesID.second = adjacentNode;}
			net.arcs[firstArc].vertexList=firstCurve;
		}
	}
	LinearCurveNet tempCurves;
	for(int i=0;i<curves.size();i++){
		if(curves[i].empty())
			continue;
		tempCurves.push_back(curves[i]);
	}
	m_curveNetworkOriginal = tempCurves;
	m_curveNet = net;
}
void cycleUtils::computeCurveNormal(const std::vector<Point> &org, Point &tar)
{
	std::vector<double> weights(org.size()-1);
	double size = double(org.size());
	for(int i=0;i<org.size()-1;i++)
		weights[i]=pow(1-double(i)/((size-1.0)==0?1:(size-1.0)),5);
	double sum=0.;
	for(int i=0;i<weights.size();i++)
		sum+=weights[i];
	for(int i=0;i<weights.size();i++)
		weights[i]/=sum;
	std::vector<Point> vector(org.size()-1);
	for(int i=0;i<org.size()-1;i++){
		vector[i]=(org[i+1]-org[i]);
		vector[i].normalize();
	}
	for(int i=0;i<3;i++){
		sum=0;
		for(int j=0;j<weights.size();j++)
			sum+= weights[j]*vector[j][i];
		tar[i]=sum;
	}
	tar.normalize();
}
void cycleUtils::computeTransportMatrix(const std::vector<Point> &org, std::vector<double> &tar)
{
	Point e1 = (org[1]-org[0]);
	e1.normalize();
	Point e2 = (org.back()-org[org.size()-2]);
	e2.normalize();
	Point randomVector;
	while(true){
		for(int i=0;i<3;i++)
			randomVector[i] = (double)(rand()%1000)/1000.;
		randomVector.normalize();
		if(randomVector.length()!=0 && randomVector!=e1 && -randomVector!=e1)
			break;
	}
	Point coord[2];
	coord[0] = (e1.cross(randomVector));
	coord[0].normalize();
	coord[1] = e1.cross(coord[0]);
	coord[1].normalize();
	Point coordTrans[]={coord[0],coord[1]};
	std::vector<Point> vector(org.size()-1);
	for(int i=0;i<org.size()-1;i++){
		vector[i]=(org[i+1]-org[i]);
		vector[i].normalize();
	}
	Point tangent;
	for(int j=0;j<vector.size()-1;j++){
		tangent = (vector[j]+vector[j+1]);
		tangent.normalize();
		for(int i=0;i<2;i++){
			coordTrans[i] -= 2*(coordTrans[i].dot(tangent))*tangent;
			coordTrans[i].normalize();
		}
	}
	Point leftMatrix[3],rightMatrix[3];
	for(int i=0;i<3;i++){
		leftMatrix[i].x=e1[i];leftMatrix[i].y=coord[0][i];leftMatrix[i].z=coord[1][i];
		rightMatrix[i].x=e2[i];rightMatrix[i].y=coordTrans[0][i];rightMatrix[i].z=coordTrans[1][i];
	}
	for(int i=0;i<3;i++){
		for(int j=0;j<3;j++){
			tar.push_back(leftMatrix[i].dot(rightMatrix[j]));
		}
	}
}
void cycleUtils::computeTransportMatrixAll(const std::vector<Point> &org, std::vector<std::vector<double> >  &tar)
{
	int sizeCurve = org.size();
	int sizeVector=sizeCurve-1;

	Point e1 = (org[1]-org[0]);
	e1.normalize();
	Point randomVector;
	randomVector.x=1;
	randomVector.y=exp(1.);
	randomVector.z=M_PI;
	while(true){
		if(randomVector.length()!=0 && randomVector!=e1 && -randomVector!=e1)
			break;
		for(int i=0;i<3;i++)
			randomVector[i] = (double)(rand()%1000)/1000.;
		randomVector.normalize();
	}
	std::vector<Point> coord(2);
	coord[0] = randomVector.cross(e1);
	coord[0].normalize();
	coord[1] = e1.cross(coord[0]);

	std::vector<std::vector<Point> > coordTrans(2);
	coordTrans[0].push_back(coord[0]);
	coordTrans[1].push_back(coord[1]);
	std::vector<Point> pointVector(sizeVector);
	for(int i=0;i<sizeVector;i++){
		pointVector[i]=org[i+1]-org[i];
		pointVector[i].normalize();
	}
	Point tangent;
	for(int j=0;j<sizeVector-1;j++){
		tangent = (pointVector[j]+pointVector[j+1]);
		tangent.normalize();
		Point tvec = coordTrans[0].back()-2*(coordTrans[0].back().dot(tangent))*tangent;
		tvec.normalize();
		coordTrans[0].push_back(tvec);
		tvec = pointVector[j+1].cross(tvec);
		coordTrans[1].push_back(tvec);
	}

	std::vector<Point> leftMatrix(3);
	for(int i=0;i<3;i++){
		leftMatrix[i].x=e1[i];
		leftMatrix[i].y=coord[0][i];
		leftMatrix[i].z=coord[1][i];
	}

	for(int v=0;v<pointVector.size();v++){
		Point rightMatrix[3];
		for(int i=0;i<3;i++){
			rightMatrix[i].x=pointVector[v][i];
			rightMatrix[i].y=coordTrans[0][v][i];
			rightMatrix[i].z=coordTrans[1][v][i];
		}
		std::vector<double> tmax;
		for(int i=0;i<3;i++){
			for(int j=0;j<3;j++){
				double temp=leftMatrix[i].dot(rightMatrix[j]);
				tmax.push_back(temp);
			}
		}
		tar.push_back(tmax);
	}
}
double cycleUtils::computeAngle(const Point &org1,const Point &org2)
{
	Point ang_str = org1;
	Point ang_end = org2;
	ang_str.normalize();
	ang_end.normalize();
	double res = ang_str.dot(ang_end);
	res = res < -1.0 ? -1.0 : res;
	res = res > 1.0 ? 1.0 : res;
	return acos(res);
}
double cycleUtils::computeAnglebetweenTwoCurves(const int &i,const int &j,const std::vector<Curve> &pointLists)
{
	Point v[2];
	computeCurveNormal(pointLists[i],v[0]);
	computeCurveNormal(pointLists[j],v[1]);
	return computeAngle(v[0],v[1]);
}
Point cycleUtils::computeNormalPlanebetweenTwoCurves(const int &i,const int &j,const std::vector<Curve> &pointLists)
{
	Point v[2];
	computeCurveNormal(pointLists[i],v[0]);
	computeCurveNormal(pointLists[j],v[1]);
	return v[0].cross(v[1]);
}
bool mySort1(std::pair<double,std::pair<int,int> > i,std::pair<double,std::pair<int,int> > j) 
{ 
	return (i.first<j.first); 
}
void cycleUtils::constructJointRotationGraph()
{
	Graph net = m_curveNet;
	RotationGraph roGraph=m_rotationGraph;

	for(int i=0;i<net.nodes.size();i++){
		std::vector<int> capacity;
		std::vector<std::pair<double,std::pair<int,int> > > arcsWithWeights;
		std::vector<Curve> pointLists;
		for(int j=0;j<net.nodes[i].arcID.size();j++){
			int arcID = net.nodes[i].arcID[j];
			capacity.push_back(net.arcs[arcID].capacity);
			Curve curve = net.arcs[arcID].vertexList;
			curve.push_back(net.nodes[net.arcs[arcID].endNodesID.second].pos);
			curve.insert(curve.begin(),net.nodes[net.arcs[arcID].endNodesID.first].pos);
			if(net.arcs[arcID].endNodesID.second==i)
				reverse(curve.begin(),curve.end());
			pointLists.push_back(curve);
		}
		std::vector<std::vector<int> > indices(capacity.size(),std::vector<int>(capacity.size(),0));
		for(int j=0;j<capacity.size()-1;j++){
			for(int k=j+1;k<capacity.size();k++){
				double weight = computeAnglebetweenTwoCurves(j,k,pointLists);
				arcsWithWeights.push_back(std::pair<double,std::pair<int,int> >(weight,std::pair<int,int>(j,k)));
				indices[j][k]=(arcsWithWeights.size()-1);
				indices[k][j]=indices[j][k];
			}
		}
		std::vector<std::pair<int,int> > usedArcsID;
		std::vector<std::vector<std::pair<int,int> > > allUsedArcsID;
		std::vector<double> nodeWeights;
		if(m_isDihedral){
			std::vector<std::vector<double> > pairArcsWithWeight(arcsWithWeights.size(),std::vector<double>(arcsWithWeights.size(),0));
			std::vector<Point> norm;

			for(int j=0;j<arcsWithWeights.size();j++){
				Point normal_ = computeNormalPlanebetweenTwoCurves(arcsWithWeights[j].second.first,
					arcsWithWeights[j].second.second,pointLists);
				normal_.normalize();
				norm.push_back(normal_);
			}
			int count=0;
			for(int a=0;a<capacity.size();a++){
				for(int k=0;k<capacity.size();k++){
					for(int j=k;j<capacity.size();j++){
						if(j!=a&&k!=a){
							count++;
							if(k<a&&a<j){
								Point v[2];
								v[0] = norm[indices[a][k]];
								v[1] = norm[indices[a][j]];
								pairArcsWithWeight[indices[a][k]][indices[a][j]]=
									computeAngle(v[0],v[1]);
							}
							else{
								Point v[2];
								v[0] = norm[indices[a][k]];
								v[1] = -norm[indices[a][j]];
								pairArcsWithWeight[indices[a][k]][indices[a][j]]=
									computeAngle(v[0],v[1]);
							}
							pairArcsWithWeight[indices[a][j]][indices[a][k]]
							=pairArcsWithWeight[indices[a][k]][indices[a][j]];
						}
					}
				}
			}
			m_graphSearch->computeGeneralHamiltonGraph(capacity,
				arcsWithWeights, pairArcsWithWeight,10,true,
				false,allUsedArcsID,nodeWeights);
		}
		else{
			sort(arcsWithWeights.begin(),arcsWithWeights.end(),mySort1);
			m_graphSearch->computeGeneralHamiltonGraph(capacity,
				arcsWithWeights,true,false,allUsedArcsID,nodeWeights);
		}
		usedArcsID = allUsedArcsID.front();
		if(usedArcsID.empty()){
			roGraph.clear();
			return;
		}

		std::vector<std::vector<std::pair<int,int> > > jointRotationGraph;
		for(int j=0;j<net.nodes[i].arcID.size();j++){
			std::vector<std::pair<int,int> > emptyNode;
			jointRotationGraph.push_back(emptyNode);
		}
		for(int j=0;j<usedArcsID.size();j++){
			int optArcID = usedArcsID[j].first;
			int currentArcID[]={arcsWithWeights[optArcID].second.first,arcsWithWeights[optArcID].second.second};
			std::vector<std::pair<int,int> > segRotationGraph;
			for(int k=0;k<usedArcsID[j].second;k++){
				jointRotationGraph[currentArcID[0]].push_back(std::pair<int,int>
					(currentArcID[1],jointRotationGraph[currentArcID[1]].size()));
				jointRotationGraph[currentArcID[1]].push_back(std::pair<int,int>
					(currentArcID[0],jointRotationGraph[currentArcID[0]].size()-1));
			}
		}
		roGraph.push_back(jointRotationGraph);
	}
	m_rotationGraph = roGraph;
}
void cycleUtils::constructSegmentRotationGraph()
{
	if(m_rotationGraph.empty())
		return;

	Graph net = m_curveNet;
	RotationGraph roGraph=m_rotationGraph;
	//minimize twist
	for(int i=0;i<net.arcs.size();i++){
		int nodeID[]={net.arcs[i].endNodesID.first,net.arcs[i].endNodesID.second};
		int arcID[]={net.arcs[i].posInNode.first,net.arcs[i].posInNode.second};
		Curve pointList = net.arcs[i].vertexList;
		pointList.push_back(net.nodes[nodeID[1]].pos);
		pointList.insert(pointList.begin(),net.nodes[nodeID[0]].pos);
		Curve reversePointList = pointList;
		reverse(reversePointList.begin(),reversePointList.end());
		Point v[2];
		computeCurveNormal(pointList,v[0]);
		computeCurveNormal(reversePointList,v[1]);

		std::vector<std::vector<int> > adjacentArcID,adjacentArcIDAbsolute;
		adjacentArcID.resize(2);adjacentArcIDAbsolute.resize(2);
		std::vector<std::vector<Curve> > adjacentPointList;
		adjacentPointList.resize(2);
		std::vector<std::vector<Point> >adjacentVectors,adjacentPlanes;
		adjacentVectors.resize(2);adjacentPlanes.resize(2);
		std::vector<std::vector<double> > adjacentAreas;
		adjacentAreas.resize(2);

		for(int j=0;j<2;j++){
			for(int k=0;k<net.arcs[i].capacity;k++){
				adjacentArcID[j].push_back(roGraph[nodeID[j]][arcID[j]][k].first);
				adjacentArcIDAbsolute[j].push_back(net.nodes[nodeID[j]].arcID[adjacentArcID[j][k]]);

				Curve curve = net.arcs[adjacentArcIDAbsolute[j][k]].vertexList;
				curve.push_back(net.nodes[net.arcs[adjacentArcIDAbsolute[j][k]].endNodesID.second].pos);
				curve.insert(curve.begin(),net.nodes[net.arcs[adjacentArcIDAbsolute[j][k]].endNodesID.first].pos);
				if(nodeID[j]==net.arcs[adjacentArcIDAbsolute[j][k]].endNodesID.second)
					reverse(curve.begin(),curve.end());
				adjacentPointList[j].push_back(curve);

				Point normal;
				computeCurveNormal(adjacentPointList[j][k],normal);
				adjacentVectors[j].push_back(normal);

				if(j==0)
					normal=normal.cross(v[j]);
				else
					normal=v[j].cross(normal);

				if(normal.length()==0){
					Point temp(1.,1.,1.);
					normal = temp;}
				adjacentAreas[j].push_back(normal.length());
				normal.normalize();
				adjacentPlanes[j].push_back(normal);
			}
		}
		std::vector<double> transportMatrix;
		computeTransportMatrix(pointList,transportMatrix);
		std::vector<Point> transMatrix;
		for(int j=0;j<3;j++){
			Point row;
			for(int k=0;k<3;k++){
				row[k]= double(transportMatrix[j+k*3]);
			}
			transMatrix.push_back(row);
		}
		std::vector<int> capacity;
		std::vector<std::pair<double,std::pair<int,int> > > arcsWithWeights;
		capacity.assign(2*net.arcs[i].capacity,1);
		for(int j=0;j<net.arcs[i].capacity;j++){
			Point transPlane,transVector;
			for(int l=0;l<3;l++){
				//	transVector[l] = adjacentVectors[0][j].dot(transMatrix[l]);
				transPlane[l] = adjacentPlanes[0][j].dot(transMatrix[l]);
			}
			for(int k=0;k<net.arcs[i].capacity;k++){
				double weight = adjacentAreas[0][j]*adjacentAreas[1][k];
				weight*= computeAngle(transPlane,adjacentPlanes[1][k]);
				weight = pow(weight,2);
				arcsWithWeights.push_back(std::pair<double,std::pair<int,int> >(weight,std::pair<int,int>(j,k+net.arcs[i].capacity)));
			}
		}
		sort(arcsWithWeights.begin(),arcsWithWeights.end(),mySort1);

		std::vector<std::pair<int,int> > usedArcsID;
		std::vector<std::vector<std::pair<int,int> > > allUsedArcsID;
		std::vector<double> nodeWeights;
		double minCost = m_graphSearch->computeGeneralHamiltonGraph(capacity,arcsWithWeights,false,false,allUsedArcsID,nodeWeights);

		usedArcsID = allUsedArcsID.front();
		std::vector<int> instanceID;
		instanceID.resize(net.arcs[i].capacity);
		for(int j=0;j<usedArcsID.size();j++){
			std::pair<int,int> arc = arcsWithWeights[usedArcsID[j].first].second;
			instanceID[arc.first]=arc.second-net.arcs[i].capacity;
		}
		std::vector<std::pair<int,int> > instances;
		for(int j=0;j<net.arcs[i].capacity;j++){
			instances.push_back(roGraph[nodeID[1]][arcID[1]][instanceID[j]]);
		}
		roGraph[nodeID[1]][arcID[1]] = instances;
		for(int j=0;j<net.arcs[i].capacity;j++){
			std::pair<int,unsigned int> connectInstance = roGraph[nodeID[1]][arcID[1]][j];
			roGraph[nodeID[1]][connectInstance.first][connectInstance.second]
				= std::pair<int,unsigned int>(arcID[1],j);
		}

	}
	m_rotationGraph = roGraph;
}
void cycleUtils::constructRotationGraphbyAngleMetric()
{
	if(m_curveNet.arcs.empty())
		return;

	m_rotationGraph.clear();
	constructJointRotationGraph();
	constructSegmentRotationGraph();
}
void cycleUtils::constructRotationGraphbyAngleDihedralMetric()
{
	if(m_curveNet.arcs.empty())
		return;

	m_rotationGraph.clear();
	m_isDihedral = true;
	constructJointRotationGraph();
	constructSegmentRotationGraph();
	m_isDihedral = false;
}
Point cycleUtils::rotateNormal(const Point &normal, const Point &axis, const double &angle)
{
	double d = normal.dot(axis);
	double c = cos(angle);
	double s = sin(angle);
	Point res;
	res.x= axis.x*d*(1-c)+normal.x*c+(-axis.z*normal.y+axis.y*normal.z)*s;
	res.y= axis.y*d*(1-c)+normal.y*c+(axis.z*normal.x-axis.x*normal.z)*s;
	res.z= axis.z*d*(1-c)+normal.z*c+(-axis.y*normal.x+axis.x*normal.y)*s;
	return res;
}
double cycleUtils::twistBisectCostSym(const Point &e1,const Point &e2,const Point &n1)
{
	Point tang = e1+e2;
	Point l1 = n1.cross(tang);
	l1.normalize();
	return 2*computeAngle(-e1,l1); 
}
void cycleUtils::computeTwistTables()
{
	if(m_curveNet.arcs.empty())
		return;
#ifdef USEBOOST
	int num_nodes = m_curveNet.nodes.size();
	int num_arcs = m_curveNet.arcs.size();
	graph_t g(num_nodes);
	property_map<graph_t, edge_weight_t>::type weightmap = get(edge_weight, g);
	for (std::size_t i = 0; i < num_arcs; i++) {
		int nodeID[] = {m_curveNet.arcs[i].endNodesID.first,m_curveNet.arcs[i].endNodesID.second};
		graph_traits < graph_t >::edge_descriptor e; bool inserted;
		boost::tie(e, inserted) = add_edge(nodeID[0], nodeID[1], g);
		weightmap[e] = 0;
	}

/*
	std::vector<bool> isDoubleArc(m_curveNet.arcs.size());
	std::vector<std::vector<int> > tnodes(m_curveNet.arcs.size());
	for(int i=0;i<m_curveNet.arcs.size();i++){
		tnodes[i].push_back(m_curveNet.arcs[i].endNodesID.first);
		tnodes[i].push_back(m_curveNet.arcs[i].endNodesID.second);
		for(int j=0;j<i;j++){
			if((tnodes[i][0] == tnodes[j][0]&& tnodes[i][1] == tnodes[j][1])
				|| (tnodes[i][0] == tnodes[j][1] && tnodes[i][1] == tnodes[j][0])){
					isDoubleArc[i]=isDoubleArc[j]=true;
			}
		}
	}
*/
#endif

#ifndef USEBOOST
#define USEKSHORTESTPATH
		std::vector<int> nodes;
		std::vector<std::pair<int,int> > arcs;
		std::vector<double> arcWeights;
		std::vector<int> sources;
		std::vector<int> targets;
		std::vector<std::vector<int> > paths;
		nodes.resize(m_curveNet.nodes.size());
		for(int i=0;i<m_curveNet.arcs.size();i++){
			int nodeID[] = {m_curveNet.arcs[i].endNodesID.first,m_curveNet.arcs[i].endNodesID.second};
			arcs.push_back(std::pair<int,int>(nodeID[0],nodeID[1]));
			arcWeights.push_back(0);
			arcs.push_back(std::pair<int,int>(nodeID[1],nodeID[0]));
			arcWeights.push_back(0);
		}

/*
		std::vector<bool> isDoubleArc(m_curveNet.arcs.size());
		std::vector<std::vector<int> > tnodes(m_curveNet.arcs.size());
		for(int i=0;i<m_curveNet.arcs.size();i++){
			tnodes[i].push_back(m_curveNet.arcs[i].endNodesID.first);
			tnodes[i].push_back(m_curveNet.arcs[i].endNodesID.second);
			for(int j=0;j<i;j++){
				if((tnodes[i][0] == tnodes[j][0]&& tnodes[i][1] == tnodes[j][1])
					|| (tnodes[i][0] == tnodes[j][1] && tnodes[i][1] == tnodes[j][0])){
						isDoubleArc[i]=isDoubleArc[j]=true;
				}
			}
		}
*/
#endif

	bool connectRequired=true;

	double wAngle = m_angleWeight;
	double wTwist = m_twistWeight;
	double wCurve = m_curveWeight;
	int nTwistNorm = m_twistNormNum;
	//for each arc twist
	m_normalsTable.clear();
	for(int i=0;i<m_curveNet.arcs.size();i++){

		std::vector<Point> pts = m_curveNet.arcs[i].vertexList;
		int nodeID[] = {m_curveNet.arcs[i].endNodesID.first,m_curveNet.arcs[i].endNodesID.second};
		pts.push_back(m_curveNet.nodes[nodeID[1]].pos);
		pts.insert(pts.begin(),m_curveNet.nodes[nodeID[0]].pos);

		std::vector<std::vector<int> > adjArcs(2);
		std::vector<std::vector<int> > adjArcsDirection(2);
		std::vector<std::vector<std::vector<Point> > > adjArcsPts(2);
		for(int j=0;j<2;j++){
			adjArcs[j] = m_curveNet.nodes[nodeID[j]].arcID;
			adjArcsDirection[j] = m_curveNet.nodes[nodeID[j]].arcDirection;
			for(int k=0;k<adjArcs[j].size();k++){
				int tempArc = adjArcs[j][k];
				int tempNode[] = {m_curveNet.arcs[tempArc].endNodesID.first,m_curveNet.arcs[tempArc].endNodesID.second};
				std::vector<Point> tempPts = m_curveNet.arcs[adjArcs[j][k]].vertexList;
				tempPts.push_back(m_curveNet.nodes[tempNode[1]].pos);
				tempPts.insert(tempPts.begin(),m_curveNet.nodes[tempNode[0]].pos);
				if(adjArcsDirection[j][k]==2)
					reverse(tempPts.begin(),tempPts.end());
				adjArcsPts[j].push_back(tempPts);
			}
		}

		std::vector<std::vector<Point> > vecs(2);
		for(int j=0;j<2;j++){
			for(int k=0;k<adjArcsPts[j].size();k++){
				Point vec = adjArcsPts[j][k][1]-adjArcsPts[j][k][0];
				vec.normalize();
				vecs[j].push_back(vec);
			}
		}
		
		std::vector<Point> edgeVecs;
		for(int j=0;j<pts.size()-1;j++){
			Point vec = pts[j+1] - pts[j];
			vec.normalize();
			edgeVecs.push_back(vec);
		}

		Point v[2] = {edgeVecs.front(),edgeVecs.back()};
/*
		Point randomVector;
		while(true){
			for(int i=0;i<3;i++)
				randomVector[i] = (double)(rand()%1000)/1000.;
			randomVector.normalize();
			if(randomVector.length()!=0 && randomVector!=v[0])
				break;
		}
*/

		Point randomVector;
		randomVector.x=1;
		randomVector.y=exp(1.);
		randomVector.z=M_PI;

		Point initNormal = v[0].cross(randomVector);
		initNormal.normalize();

		//test about the matrices;
		std::vector<std::vector<double> > transportMatrices;
		computeTransportMatrixAll(pts,transportMatrices);
		std::vector<std::vector<Point> > transMatrices;
		for(int t=0;t<transportMatrices.size();t++){
			std::vector<Point> transMatrix;
			for(int j=0;j<3;j++){
				Point row;
				for(int k=0;k<3;k++){
					row[k]= transportMatrices[t][j+k*3];
				}
				transMatrix.push_back(row);
			}
			transMatrices.push_back(transMatrix);
		}

		std::vector<std::vector<std::vector<double> > > pairwiseTwistTable;
		std::vector<std::vector<std::vector<std::vector<Point> > > >  endNormals;
		std::vector<std::vector<Point> > midNormals;
		for(int j=0;j<nTwistNorm;j++){
			double angle = (j+1)*2*M_PI/double(nTwistNorm);
			Point nm = rotateNormal(initNormal,v[0],angle);
			std::vector<Point> nmlist;
			for(int k=0;k<transMatrices.size();k++){
				std::vector<Point> transMatrix = transMatrices[k];
				Point tnm;
				for(int l=0;l<3;l++){
					tnm[l] = nm.dot(transMatrix[l]);
				}
				nmlist.push_back(tnm);
			}

			midNormals.push_back(nmlist);

			double costAng=0;
			for(int k=0;k<edgeVecs.size()-1;k++){
				double tc=twistBisectCostSym(edgeVecs[k],edgeVecs[k+1],nmlist[k]);
				costAng+=tc;
			}
			costAng -= (pts.size()-2)*M_PI;

			double costTwt=0;
			for(int k=0;k<edgeVecs.size()-1;k++){
				double tc=computeAngle(nmlist[k],nmlist[k+1]);
				costTwt+=tc;
			}
			double costCurve = wAngle*costAng+wTwist*costTwt;
			costCurve *=wCurve;

			std::vector<std::vector<double> > pairwiseTwist(vecs[0].size(),
				std::vector<double>(vecs[1].size(),0));	

			std::vector<std::vector<std::vector<Point> > > pairNormals(vecs[0].size(),
				std::vector<std::vector<Point> >(vecs[1].size()));

			Point nm2 = nmlist.back();
			for(int k1=0;k1<vecs[0].size();k1++){
				for(int k2=0;k2<vecs[1].size();k2++){
					Point reflect = vecs[0][k1]-v[0];
					reflect.normalize();
					Point nmT = (-nm)-2*((-nm).dot(reflect))*reflect;

					reflect = v[1]+vecs[1][k2];
					reflect.normalize();
					Point nm2T = nm2-2*(nm2.dot(reflect))*reflect;

					//compute twistBisectCostSym;
					double costAng1 = twistBisectCostSym(-v[0],vecs[0][k1],-nm);
					double costAng2 = twistBisectCostSym(v[1],vecs[1][k2],nm2);

					//comput arc twist;
					double costTwt1 = computeAngle(-nm,nmT); 
					double costTwt2 = computeAngle(nm2,nm2T); 

					//whole cost;
					pairwiseTwist[k1][k2]= costCurve+wAngle*(costAng1+costAng2)/2+
						wTwist*(costTwt1+costTwt2)/2;

					pairNormals[k1][k2].push_back(nmT);
					pairNormals[k1][k2].push_back(nm2T);
				}
			}
			pairwiseTwistTable.push_back(pairwiseTwist);
			endNormals.push_back(pairNormals);
		}
		// make tables;
		std::vector<std::vector<std::vector<double> > >tTable(vecs[0].size(),
			std::vector<std::vector<double> >(vecs[1].size(),std::vector<double>(nTwistNorm,0)));	
		for(int k1=0;k1<vecs[0].size();k1++){
			for(int k2=0;k2<vecs[1].size();k2++){
				for(int n=0;n<nTwistNorm;n++){
					tTable[k1][k2][n] = pairwiseTwistTable[n][k1][k2];
				}
			}
		}
		std::vector<std::vector<double> > twistTable(vecs[0].size(),
			std::vector<double>(vecs[1].size(),0));	
		std::vector<std::vector<int> > twistTableIndex(vecs[0].size(),
			std::vector<int>(vecs[1].size(),0));	
		std::vector<std::vector<double> > twistTableConfidence(vecs[0].size(),
			std::vector<double>(vecs[1].size(),0));	
		std::vector<std::vector<std::vector<Point> > > normalTable(vecs[0].size(),
			std::vector<std::vector<Point> >(vecs[1].size()));
		for(int k1=0;k1<vecs[0].size();k1++){
			for(int k2=0;k2<vecs[1].size();k2++){
				std::vector<double> tc = tTable[k1][k2];
				std::vector<double>::iterator itc=std::min_element(tc.begin(),tc.end());
				twistTable[k1][k2] = *itc;
				int index = itc-tc.begin();
				twistTableIndex[k1][k2] = index;
				std::vector<Point> tnorm=midNormals[index];
				tnorm.insert(tnorm.begin(),endNormals[index][k1][k2][0]);				
				tnorm.push_back(endNormals[index][k1][k2][1]);	
				normalTable[k1][k2]=tnorm;
				itc = std::max_element(tc.begin(),tc.end());
				twistTableConfidence[k1][k2] = std::min(1.0,(*itc - twistTable[k1][k2])/M_PI);
			}
		}

#ifdef USEBOOST
		if(/*isDoubleArc[i]==false&&*/connectRequired==true){
			std::vector<int> sources;
			std::vector<int> targets;
			std::vector<std::pair<int,int> > tind((adjArcs[0].size())*(adjArcs[1].size()));
			for(int j=0;j<adjArcs[0].size();j++){
				int snd;
				if(adjArcsDirection[0][j]==1)
					snd=m_curveNet.arcs[adjArcs[0][j]].endNodesID.second;
				else
					snd=m_curveNet.arcs[adjArcs[0][j]].endNodesID.first;
				if(adjArcs[0][j]==i)
					snd=nodeID[0];
				sources.push_back(snd);
			}
			for(int k=0;k<adjArcs[1].size();k++){
				int tnd;
				if(adjArcsDirection[1][k]==1)
					tnd=m_curveNet.arcs[adjArcs[1][k]].endNodesID.second;
				else
					tnd=m_curveNet.arcs[adjArcs[1][k]].endNodesID.first;
				if(adjArcs[1][k]==i)
					tnd=nodeID[1];
				targets.push_back(tnd);
			}

			graph_traits < graph_t >::edge_descriptor e; bool sus;
			graph_traits < graph_t >::vertex_descriptor vi1,vi2;
			for(int j=0;j<2;j++){
				for(int k=0;k<adjArcs[j].size();k++){
					int nodeID[] = {m_curveNet.arcs[adjArcs[j][k]].endNodesID.first,
						m_curveNet.arcs[adjArcs[j][k]].endNodesID.second};
					vi1 = vertex(nodeID[0],g);
					vi2 = vertex(nodeID[1],g);
					boost::tie(e, sus) = edge(vi1,vi2,g);
					weightmap[e] = 1;
				}
			}
			std::vector<std::vector<bool> > goodPath(sources.size(),std::vector<bool>(targets.size(),true));

			if(sources.size()<=targets.size()){
				for(int j=0;j<sources.size();j++){
					int nod = sources[j];

					std::vector<vertex_descriptor> p(num_nodes);
					std::vector<int> d(num_nodes);
					vertex_descriptor s = vertex(nod, g);
					dijkstra_shortest_paths(g, s, predecessor_map(&p[0]).distance_map(&d[0]));
					for(int k=0;k<targets.size();k++){
						if(d[targets[k]]!=0)
							goodPath[j][k] = false;
					}	
				}
			}
			else{
				for(int j=0;j<targets.size();j++){
					int nod = targets[j];
					std::vector<vertex_descriptor> p(num_nodes);
					std::vector<int> d(num_nodes);
					vertex_descriptor s = vertex(nod, g);
					dijkstra_shortest_paths(g, s, predecessor_map(&p[0]).distance_map(&d[0]));
					for(int k=0;k<sources.size();k++){
						if(d[sources[k]]!=0)
							goodPath[k][j] = false;
					}	
				}
			}
			for(int j=0;j<2;j++){
				for(int k=0;k<adjArcs[j].size();k++){
					int nodeID[] = {m_curveNet.arcs[adjArcs[j][k]].endNodesID.first,
						m_curveNet.arcs[adjArcs[j][k]].endNodesID.second};
					vi1 = vertex(nodeID[0],g);
					vi2 = vertex(nodeID[1],g);
					boost::tie(e, sus) = edge(vi1,vi2,g);
					weightmap[e] = 0;
				}
			}

			for(int j=0;j<goodPath.size();j++){
				for(int k=0;k<goodPath[j].size();k++){
					if(!goodPath[j][k]){
						if(sources[j]==targets[k] || (sources[j]==nodeID[1]&& targets[k]==nodeID[0]))
							continue;
						twistTable[j][k] = FLT_MAX;
					}
				}
			}
		}//end of shortest path.
#endif

#ifdef USEKSHORTESTPATH
			if(/*isDoubleArc[i]==false&&*/connectRequired==true){
				sources.clear();
				targets.clear();
				paths.clear();
				std::vector<std::pair<int,int> > tind((adjArcs[0].size())*(adjArcs[1].size()));
				for(int j=0;j<adjArcs[0].size();j++){
					int snd;
					if(adjArcsDirection[0][j]==1)
						snd=m_curveNet.arcs[adjArcs[0][j]].endNodesID.second;
					else
						snd=m_curveNet.arcs[adjArcs[0][j]].endNodesID.first;
/*
					if(adjArcs[0][j]==i)
						snd=nodeID[0];
*/

					for(int k=0;k<adjArcs[1].size();k++){
						int tnd;
						if(adjArcsDirection[1][k]==1)
							tnd=m_curveNet.arcs[adjArcs[1][k]].endNodesID.second;
						else
							tnd=m_curveNet.arcs[adjArcs[1][k]].endNodesID.first;
/*
						if(adjArcs[1][k]==i)
							tnd=nodeID[1];
*/

						bool isExisted = false;
						for(int s=0;s<sources.size();s++){
							if(sources[s]==snd && targets[s]==tnd){
								isExisted=true; break;
							}
						}
						if(!isExisted){
							sources.push_back(snd);
							targets.push_back(tnd);
							tind[sources.size()-1]=std::pair<int,int>(j,k);
						}
					}
				}
				for(int j=0;j<2;j++){
					for(int k=0;k<adjArcs[j].size();k++){
						arcWeights[adjArcs[j][k]*2]=1;
						arcWeights[adjArcs[j][k]*2+1]=1;
					}
				}
				std::vector<double> cst;
				KShortestPaths(nodes,arcs,arcWeights,sources,targets,cst,paths);
				for(int j=0;j<2;j++){
					for(int k=0;k<adjArcs[j].size();k++){
						arcWeights[adjArcs[j][k]*2]=0;
						arcWeights[adjArcs[j][k]*2+1]=0;
					}
				}

				for(int j=0;j<paths.size();j++){
					if(sources[j]==targets[j] || (sources[j]==nodeID[1]&& targets[j]==nodeID[0]))
						continue;
					if(cst[j]!=0)
						twistTable[tind[j].first][tind[j].second] = FLT_MAX;
/*
					else{
						if(paths[j].size()==4){
							if((paths[j][1]==nodeID[0] && paths[j][2]==nodeID[1]) ||(paths[j][1]==nodeID[1] && paths[j][2]==nodeID[0]))
								twistTable[tind[j].first][tind[j].second] = FLT_MAX;
						}
					}
*/
				}
			}
#endif
		m_twistTables.push_back(twistTable);
		m_twistTablesConfidence.push_back(twistTableConfidence );
		m_twistTablesIndex.push_back(twistTableIndex);
		m_normalsTable.push_back(normalTable);
	}
}
void cycleUtils::constructJointRotationGraphbyPoleGraph()
{
	if(m_curveNet.arcs.empty() || m_twistTables.empty())
		return;

	Graph net = m_curveNet;
	int maxArcPairNum=m_bestNeightboreNum;
	int maxRoGraphNum=m_rotationGraphNum;
	double wNode =m_nodeWeight;
	bool isConnect = m_isRoGraphConnect;

	for(int i=0;i<net.nodes.size();i++){
		if(!m_latestUpdateNodes.empty ()){
			if(std::find(m_latestUpdateNodes.begin(),m_latestUpdateNodes.end(),i) == m_latestUpdateNodes.end())
				continue;
		} 

		Point pt = net.nodes[i].pos;
		std::vector<int> arcs = net.nodes[i].arcID;
		std::vector<int> dirs = net.nodes[i].arcDirection;
		std::vector<std::vector<std::pair<double,std::pair<int,int> > > > tab(arcs.size(),
				std::vector<std::pair<double,std::pair<int,int> > >(arcs.size(),
				std::pair<double,std::pair<int,int> >(FLT_MAX,std::pair<int,int>(0,0))));
		int id[2];
		//make cost table including each arc pairs;
		for(id[0]=0;id[0]<arcs.size()-1;id[0]++){
			for(id[1]=id[0]+1;id[1]<arcs.size();id[1]++){
				int arcID[]={arcs[id[0]],arcs[id[1]]};
				int dir[]={dirs[id[0]],dirs[id[1]]};
				double cost=0;
				for(int j=0;j<2;j++){
					std::vector<std::vector<double> > twistTable=m_twistTables[arcID[j]];
					if(dir[j]==1){
						std::vector<double> tempTwist = twistTable[id[(j+1)%2]];
						tempTwist.erase(tempTwist.begin()+net.arcs[arcID[j]].posInNode.second);
						double tc=*std::min_element(tempTwist.begin(),tempTwist.end());
						if(cost>=FLT_MAX || tc>=FLT_MAX)
							cost=FLT_MAX;
						else
							cost+=tc;
					}
					else{
						double tc=FLT_MAX;
						for(int t1=0;t1<twistTable.size();t1++){
							if(t1==net.arcs[arcID[j]].posInNode.first)
								continue;
							if(tc>twistTable[t1][id[(j+1)%2]])
								tc = twistTable[t1][id[(j+1)%2]];
						}
						if(cost>=FLT_MAX || tc>=FLT_MAX)
							cost=FLT_MAX;
						else
							cost+=tc;
					}
				}
				tab[id[0]][id[1]].first =tab[id[1]][id[0]].first =cost;//make it bigger than zero. then we can use zero cost for userdefine arc pairs;
				tab[id[0]][id[1]].second = std::pair<int,int>(id[0],id[1]);
				tab[id[1]][id[0]].second = std::pair<int,int>(id[1],id[0]);
			}
		}

		for(int j=0;j<arcs.size();j++)
			sort(tab[j].begin(),tab[j].end(),mySort1);

		std::vector<int> capacity;
		std::vector<std::pair<int,int> > allArcs;
		std::vector<double> weights;

		for(int j=0;j<net.nodes[i].arcID.size();j++){
			int arcID = net.nodes[i].arcID[j];
			capacity.push_back(net.arcs[arcID].capacity);
		}
		std::vector<std::vector<std::pair<int,double> > > rank(arcs.size(),std::vector<std::pair<int,double> >(arcs.size(),
			std::pair<int,double>(0,0)));
		for(int j=0;j<arcs.size();j++){
			for(int k=0;k<arcs.size();k++){
				id[0]=tab[j][k].second.first;
				id[1]=tab[j][k].second.second;
				if(id[0]==id[1]) id[0]=id[1]=j;
				rank[id[0]][id[1]].first = k;
				rank[id[0]][id[1]].second = tab[j][k].first;
			}
		}
		//set up arc pairs and weights and capacities;
		std::vector<std::pair<double,std::pair<int,int> > > arcsWithWeights;
		for(int j=0;j<arcs.size()-1;j++){
			for(int k=j+1;k<arcs.size();k++){
				if( rank[j][k].second == FLT_MAX)
					continue;
				if((rank[j][k].first<(capacity[j]+maxArcPairNum)) ||(rank[k][j].first<(capacity[k]+maxArcPairNum))){
					std::pair<double,std::pair<int,int> > tarcpair;
					tarcpair.first = rank[j][k].second;
					tarcpair.second = std::pair<int,int>(j,k);
					arcsWithWeights.push_back(tarcpair);
					std::pair<int,int> temp; temp.first=j; temp.second=k;
					allArcs.push_back(temp);
					weights.push_back(rank[j][k].second);
				}
			}
		}
		//add constraints 1)decrease capacity;
		std::vector<std::pair<int,int> > userDefinePairs = m_userDefinedPairsInNode[i];
		for(int j=0;j<userDefinePairs.size();j++){
			int u1 = userDefinePairs[j].first;
			int u2 = userDefinePairs[j].second;
			capacity[u1]--; capacity[u2]--;
		}
		if(*std::min_element(capacity.begin(),capacity.end())<0){
			m_poleGraphNodes.clear();
			m_poleGraphNodeWeight.clear();
			m_poleGraphArcsWeight.clear();
			m_poleGraphArcsMatch.clear();
			m_selectArcList.clear();
			m_selectArcList.resize(1);
			m_userDefinedPairsInArc.clear();
			m_userDefinedPairsInNode.clear();
		}

		// graph search;
		std::vector<std::vector<std::pair<int,int> > > allUsedArcsID;
		std::vector<double> nodeWeights;
		userDefinePairs.clear();
		m_graphSearch->computeGeneralHamiltonGraph(capacity,allArcs,weights,maxRoGraphNum,isConnect,userDefinePairs,allUsedArcsID,nodeWeights);

		if(allUsedArcsID.empty()){
			m_poleGraphNodes.clear();
			m_poleGraphNodeWeight.clear();
			m_poleGraphArcsWeight.clear();
			m_poleGraphArcsMatch.clear();
			m_selectArcList.clear();
			m_selectArcList.resize(1);
			m_userDefinedPairsInArc.clear();
			m_userDefinedPairsInNode.clear();
			return;
		}

		//use constraints 2)add arc pairs;
		userDefinePairs = m_userDefinedPairsInNode[i];
		std::vector<int> ind;
		for(int j=0;j<userDefinePairs.size();j++){
			int u1 = userDefinePairs[j].first;
			int u2 = userDefinePairs[j].second;
			bool isfind=false;
			for(int t=0;t<allArcs.size();t++){
				int r1 = allArcs[t].first;
				int r2 = allArcs[t].second;
				if((r1==u1&&r2==u2)	||(r1==u2&&r2==u1)){
					ind.push_back(t);
					isfind=true; break;
				}
			}
			if(isfind==false){
				allArcs.push_back(userDefinePairs[j]);
				ind.push_back(allArcs.size()-1);
			}
		}
		for(int j=0;j<userDefinePairs.size();j++){
			int u1 = userDefinePairs[j].first;
			int u2 = userDefinePairs[j].second;
			for(int t=0;t<allUsedArcsID.size();t++){
				bool isfind=false;
				for(int k=0;k<allUsedArcsID[t].size();k++){
					int r1 = allArcs[allUsedArcsID[t][k].first].first;
					int r2 = allArcs[allUsedArcsID[t][k].first].second;

					if((r1==u1&&r2==u2)	||(r1==u2&&r2==u1)){
						allUsedArcsID[t][k].second++; isfind=true; break;
					}
				}
				if(isfind==false){
					std::pair<int,int> temp; temp.first=ind[j]; temp.second=1;
					allUsedArcsID[t].push_back(temp);
				}
			}
		}

		//convert arc pairs to rotation graph;
		for(int j=0;j<nodeWeights.size();j++)
			nodeWeights[j] *=wNode;

		RotationGraph poleGraphNode;
		for(int a=0;a<allUsedArcsID.size();a++){
			std::vector<std::vector<std::pair<int,int> > > jointRotationGraph;
			for(int j=0;j<net.nodes[i].arcID.size();j++){
				std::vector<std::pair<int,int> > emptyNode;
				jointRotationGraph.push_back(emptyNode);
			}
			std::vector<std::pair<int,int> > usedArcsID = allUsedArcsID[a];
			for(int j=0;j<usedArcsID.size();j++){
				int optArcID = usedArcsID[j].first;
				int currentArcID[]={allArcs[optArcID].first,allArcs[optArcID].second};
				std::vector<std::pair<int,int> > segRotationGraph;
				for(int k=0;k<usedArcsID[j].second;k++){
					std::pair<int,int> temp; temp.first= currentArcID[1]; temp.second= jointRotationGraph[currentArcID[1]].size();
					jointRotationGraph[currentArcID[0]].push_back(temp);
					temp.first= currentArcID[0]; temp.second= jointRotationGraph[currentArcID[0]].size()-1;
					jointRotationGraph[currentArcID[1]].push_back(temp);
				}
			}
			poleGraphNode.push_back(jointRotationGraph);
		}
		if(m_poleGraphNodes.size()<=i){
			m_poleGraphNodes.push_back(poleGraphNode);
			m_poleGraphNodeWeight.push_back(nodeWeights);
		}
		else{
			m_poleGraphNodes[i] = poleGraphNode;
			m_poleGraphNodeWeight[i] = nodeWeights;
		}
	}
}
std::vector<std::vector<int> > ordering(const std::vector<std::vector<double> > &data)
{
	std::vector<std::vector<int> > rank(data.size(),std::vector<int>(data.size(),0));
	for(int i=0;i<data.size();i++){
		std::vector<double> tdat = data[i];
		for(int j=0;j<tdat.size();j++){
			tdat[j]/=2.;
		}
		for(int j=0;j<tdat.size();j++){
			int minInd = j;
			for(int k=0;k<tdat.size();k++){
				if(tdat[minInd]>tdat[k])
					minInd = k;
			}
			rank[i][minInd]=j;
			tdat[minInd] = FLT_MAX;
		}
	}
	return rank;
}
std::vector<int> ordering(const std::vector<int> &data)
{
	std::vector<int> rank(data.size());
	std::vector<int> tdat = data;
	for(int j=0;j<tdat.size();j++){
		int minInd = j;
		for(int k=0;k<tdat.size();k++){
			if(tdat[minInd]>tdat[k])
				minInd = k;
		}
		rank[minInd]=j;
		tdat[minInd] = INT_MAX;
	}
	return rank;
}
double getDihedralCostIndexed(const std::vector<int> &ns,int &twistNormNum)
{
	std::vector<int> ind = ns;
	sort(ind.begin(),ind.end());
	int len = ind.size();
	ind.push_back(ind.front());
	double c=0;
	double a=2*M_PI/twistNormNum;
	double expDih = 2;
	for(int i=0;i<len;i++){
		int d=abs(ind[i]-ind[i+1]);
		double tc;
		if((d*2)>twistNormNum)
			tc = a*(twistNormNum - d);
		else
			tc = a*d;
		c+=pow(M_PI - tc,expDih);	//dihedral between normal i and i+1 when expDih=1;	
	}
	return pow(c/len,1/expDih)*len-(len-2)*M_PI;
}
void cycleUtils::constructSegmentRotationGraphbyPoleGraph()
{
	if(m_curveNet.arcs.empty() || m_twistTables.empty() || m_poleGraphNodes.empty())
		return;

	Graph net = m_curveNet;
	double wDihedral =m_dihedralWeight;
	int nTwistNorm = m_twistNormNum;


	//minimize twist
	sort(m_latestUpdateArcs.begin(),m_latestUpdateArcs.end());
	for(int i=1;i<m_latestUpdateArcs.size();i++){
		if(m_latestUpdateArcs[i]==m_latestUpdateArcs[i-1]){
			m_latestUpdateArcs.erase(m_latestUpdateArcs.begin()+i);
			i--;
		}
	}
	for(int i=0;i<net.arcs.size();i++){
		if(!m_latestUpdateArcs.empty() && std::find(m_latestUpdateArcs.begin(),m_latestUpdateArcs.end(),i) == m_latestUpdateArcs.end())
			continue;

		int nodeID[]={net.arcs[i].endNodesID.first,net.arcs[i].endNodesID.second};
		int arcID[]={net.arcs[i].posInNode.first,net.arcs[i].posInNode.second};
		int cap = net.arcs[i].capacity;

		std::vector<std::vector<double> > twistTable = m_twistTables[i];	
		std::vector<std::vector<int> > twistTableIndex = m_twistTablesIndex[i];	
		std::vector<std::vector<double> > twistTableConfidence = m_twistTablesConfidence[i];	

		std::vector<std::vector<std::vector<std::pair<int,int> > > > matches(m_poleGraphNodes[nodeID[0]].size(),
			std::vector<std::vector<std::pair<int,int> > >(m_poleGraphNodes[nodeID[1]].size()));
		std::vector<std::vector<double> > poleGraphArcWeight(m_poleGraphNodes[nodeID[0]].size(),std::vector<double>(m_poleGraphNodes[nodeID[1]].size(),0));

		for(int k1=0;k1<poleGraphArcWeight.size();k1++){
			for(int k2=0;k2<poleGraphArcWeight[k1].size();k2++){

				std::vector<std::pair<int,int> > preArcs = m_userDefinedPairsInArc[i];
				if(preArcs.size()>cap){
					m_poleGraphNodes.clear();
					m_poleGraphNodeWeight.clear();
					m_poleGraphArcsWeight.clear();
					m_poleGraphArcsMatch.clear();
					m_selectArcList.clear();
					m_selectArcList.resize(1);
					m_userDefinedPairsInArc.clear();
					m_userDefinedPairsInNode.clear();
					return;
				}

				std::vector<std::pair<int,int> > preArcsNew;
				std::vector<std::vector<std::pair<int,int> > > rot1 = m_poleGraphNodes[nodeID[0]][k1];
				std::vector<std::vector<std::pair<int,int> > > rot2 = m_poleGraphNodes[nodeID[1]][k2];

				std::vector<std::vector<double> > costs(cap,std::vector<double>(cap));
				std::vector<std::vector<double> > tvars(cap,std::vector<double>(cap));
				std::vector<std::vector<int> > tnmids(cap,std::vector<int>(cap));
				std::vector<bool> useListL(cap,false);
				std::vector<bool> useListR(cap,false);
				for(int j=0;j<cap;j++){
					for(int k=0;k<cap;k++){
						if(useListL[j]==true || useListR[k]==true)
							continue;

						int inst1 = rot1[arcID[0]][j].first;
						int inst2 = rot2[arcID[1]][k].first;
						costs[j][k] = twistTable[inst1][inst2];
						tvars[j][k] = twistTableConfidence[inst1][inst2];
						tnmids[j][k] = twistTableIndex[inst1][inst2];


						for(int p=0;p<preArcs.size();p++){
							if(preArcs[p].first== inst1 && preArcs[p].second== inst2){
								preArcs.erase(preArcs.begin()+p);
								std::pair<int,int> temp; temp.first=j; temp.second=k; 
								preArcsNew.push_back(temp);

								useListL[j]=true; useListR[k]=true;
								break;
							}
						}
					}
				}
				if(!preArcs.empty()){
					continue;
				}
/*
				if(preArcsNew.size()==cap){
					std::vector<std::pair<int,int> > optpairs(cap);
					for(int j=0;j<cap;j++){
						optpairs[j].first=cap*j+preArcsNew[j].second;
						optpairs[j].second =1;
					}
					poleGraphArcWeight[k1][k2] = 0;
					matches[k1][k2] = optpairs;
					continue;
				}
*/
				std::vector<std::vector<double> > transposeCosts(cap,std::vector<double>(cap));
				for(int j=0;j<cap;j++){
					for(int k=0;k<cap;k++){
						transposeCosts[j][k] = costs[k][j];
					}
				}
				std::vector<std::pair<int,int> > trasposePairs;
				for(int j=0;j<preArcsNew.size();j++){
					std::pair<int,int> temp; temp.first=preArcsNew[j].second; temp.second=preArcsNew[j].first; 
					trasposePairs.push_back(temp);
				}

				std::vector<std::vector<int> > firstGroup = ordering(costs);
				std::vector<std::vector<int> > secondGroup = ordering(transposeCosts);

				std::vector<int> match1;
				std::vector<int> match2;
				m_graphSearch->stableMatching(firstGroup,secondGroup,preArcsNew,match1);
				m_graphSearch->stableMatching(secondGroup,firstGroup,trasposePairs,match2);

				double cost1,cost2;
				cost1 = cost2 =0;
				for(int j=0;j<cap;j++){
					cost1+= costs[j][match1[j]];
					cost2+= transposeCosts[j][match2[j]];
				}			

				std::vector<int> optmatch;
				std::vector<std::pair<int,int> > optpairs(cap);
				double optcost;

				if(cost1<cost2){
					optmatch = match1;
					optcost = cost1;
					for(int j=0;j<cap;j++){
						optpairs[j].first=cap*j+match1[j];
						optpairs[j].second =1;
					}
				}
				else{
					optmatch = ordering(match2);
					optcost = cost2;
					for(int j=0;j<cap;j++){
						optpairs[j].first=cap*match2[j]+j;
						optpairs[j].second =1;
					}
				}

				double var=0;
				std::vector<int> ns;
				for(int j=0;j<cap;j++){
					var+=tvars[j][optmatch[j]];
					ns.push_back(tnmids[j][optmatch[j]]);
				}
				var/=cap;

				optcost += wDihedral*(var*getDihedralCostIndexed(ns,nTwistNorm) + (1-var)*M_PI)*(cap-1);

				if(optcost>FLT_MAX)
					optcost=FLT_MAX;
				poleGraphArcWeight[k1][k2] = optcost;
				matches[k1][k2] = optpairs;
			}
		}
		if(poleGraphArcWeight.empty()){
			m_poleGraphNodes.clear();
			m_poleGraphNodeWeight.clear();
			m_poleGraphArcsWeight.clear();
			m_poleGraphArcsMatch.clear();
			m_selectArcList.clear();
			m_selectArcList.resize(1);
			m_userDefinedPairsInArc.clear();
			m_userDefinedPairsInNode.clear();
			return;
		}

		if(m_poleGraphArcsWeight.size()<=i){
			m_poleGraphArcsWeight.push_back(poleGraphArcWeight);
			m_poleGraphArcsMatch.push_back(matches);
		}
		else{
			m_poleGraphArcsWeight[i] = poleGraphArcWeight;
			m_poleGraphArcsMatch[i] = matches;
		}
	}
}
void cycleUtils::constructExpandSequence()
{
	if(m_curveNet.arcs.empty() || m_twistTables.empty() || m_poleGraphNodes.empty() || m_poleGraphArcsWeight.empty())
		return;
	
	int numOfPole = m_poleGraphNodeWeight.size();

	m_expandPoleSequence.clear();
	m_selectedNodeInPole.clear();

	std::vector<std::vector<int> > adjcentPoles(numOfPole);
	for(int i=0;i<m_curveNet.arcs.size();i++){
		int nodeID[]={m_curveNet.arcs[i].endNodesID.first,m_curveNet.arcs[i].endNodesID.second};
		adjcentPoles[nodeID[0]].push_back(nodeID[1]);
		adjcentPoles[nodeID[1]].push_back(nodeID[0]);
	}
	std::vector<int> unvisitedNeighboreNum(numOfPole);
	for(int i=0;i<numOfPole;i++)
		unvisitedNeighboreNum[i]=adjcentPoles[i].size();

	std::vector<int> unvisitedNodes(numOfPole);
	for(int i=0;i<numOfPole;i++)
		unvisitedNodes[i]=i;

	std::vector<long long> frontNodes;

	for(int i=0;i<numOfPole;i++){
		ExpandPole currentExpandPole;
		long long minStateNum = LLONG_MAX;
		int selectedPole=0;
		int selectedPoleIndexInUnvisitedNodes=0;
		std::vector<long long> newFront;
		std::vector<int> newUnvisitedNeighboreNum;

		for(int j=0;j<unvisitedNodes.size();j++){
			std::vector<int> tempUnvisitedNeighboreNum=unvisitedNeighboreNum;
			int currentPole = unvisitedNodes[j];
			for(int k=0;k<adjcentPoles[currentPole].size();k++){
				tempUnvisitedNeighboreNum[adjcentPoles[currentPole][k]]--;
			}
			std::vector<long long> tempFrontNodes = frontNodes;
			tempFrontNodes.push_back(currentPole);
			std::vector<long long> tempFrontNodes2;
			long long stateNum=1;
			for(int k=0;k<tempFrontNodes.size();k++){
				if(tempUnvisitedNeighboreNum[tempFrontNodes[k]]>0){
					tempFrontNodes2.push_back(tempFrontNodes[k]);
					stateNum*=m_poleGraphNodeWeight[tempFrontNodes[k]].size();
				}
			}
			if(stateNum <0){
			}
			if(stateNum<minStateNum){
				minStateNum=stateNum;
				selectedPole = currentPole;
				selectedPoleIndexInUnvisitedNodes = j;
				newFront = tempFrontNodes2;
				newUnvisitedNeighboreNum = tempUnvisitedNeighboreNum;
			}
		}

		frontNodes = newFront;
		unvisitedNeighboreNum = newUnvisitedNeighboreNum;
		unvisitedNodes.erase(unvisitedNodes.begin()+selectedPoleIndexInUnvisitedNodes);
		currentExpandPole.nodeID = selectedPole;
		currentExpandPole.front = frontNodes;
		currentExpandPole.stateNum = minStateNum;

		m_expandPoleSequence.push_back(currentExpandPole);
	}
}
void getBases(const std::vector<long long> &org, std::vector<long long> &tar)
{
	if(org.empty())
		return;
	tar = org;
	tar.erase(tar.begin());
	tar.push_back(1);
	for(int i=tar.size()-2;i>=0;i--)
		tar[i]*=tar[i+1];
}
long long indicesToIndex(const std::vector<long long> &org, const std::vector<long long> &org2)
{
	if(org.empty())
		return 0;
	long long index=0;
	for(int i=0;i<org.size();i++){
		index+= org[i]*org2[i];
	}
	return index;
}
void indexToIndices(const long long &org,const std::vector<long long> &org2, std::vector<long long> &tar)
{
	if(org2.empty())
		return;
	tar = org2;
	long long temp = org;
	for(int i=0;i<tar.size();i++){
		long long temp2=temp%tar[i];
		tar[i] = temp/tar[i];
		temp = temp2;
	}
}
std::vector<int> getMinValuesPos(const std::vector<double> &data, const int &n)
{
	int len = data.size();
	std::vector<int> ind;
	for(int i=0;i<len;i++){
		int count = 0;
		for(int j=0;j<len;j++){
			if(data[i]<data[j])
				count++;
		}
		if(count>=(len-n))
			ind.push_back(i);
		count =0;
	}
	return ind;
}
void cycleUtils::searchAlmostMinPoleGraph()
{
	if(m_curveNet.arcs.empty() || m_twistTables.empty() || m_poleGraphNodes.empty() 
		|| m_poleGraphArcsWeight.empty() || m_expandPoleSequence.empty())
		return;

	int numOfPole = m_poleGraphNodeWeight.size();

	long long nState = m_stateNum;
	std::vector<std::vector<int> > adjcentPoles(numOfPole);
	std::vector<std::vector<std::pair<int,int> > > adjcentArcs(numOfPole);
	for(int i=0;i<m_curveNet.arcs.size();i++){
		int nodeID[]={m_curveNet.arcs[i].endNodesID.first,m_curveNet.arcs[i].endNodesID.second};
		adjcentPoles[nodeID[0]].push_back(nodeID[1]);
		adjcentPoles[nodeID[1]].push_back(nodeID[0]);
		std::pair<int,int> temp; temp.first=i; temp.second=0; 
		adjcentArcs[nodeID[0]].push_back(temp);
		temp.second=1;
		adjcentArcs[nodeID[1]].push_back(temp);
	}
	std::vector<bool> visitedPole(numOfPole,false);
	std::vector<int> numOfNode(numOfPole);
	for(int i=0;i<numOfPole;i++)
		numOfNode[i]=m_poleGraphNodeWeight[i].size();

/*
	std::vector<std::vector<double> > costs(numOfPole);
	std::vector<std::vector<int> > prevs(numOfPole);
	std::vector<std::vector<int> > nodes(numOfPole);
	//the size of state may rise up to millions,that would fail.
	for(int i=0;i<numOfPole;i++){
		costs[i].resize(m_expandPoleSequence[i].stateNum,FLT_MAX);
		prevs[i].resize(m_expandPoleSequence[i].stateNum,-1);
		nodes[i].resize(m_expandPoleSequence[i].stateNum,-1);
	}

*/
	std::vector<std::vector<long long> > stateInd(numOfPole); //long long is _int64. Crash happens when a state index is larger than 2^64-1;
	std::vector<std::vector<double> > costsVal(numOfPole);
	std::vector<std::vector<long long> > prevsVal(numOfPole);
	std::vector<std::vector<int> > nodesVal(numOfPole);

	for(int i=0;i<numOfPole;i++){
		int tsn = std::min(nState,m_expandPoleSequence[i].stateNum);
		stateInd[i].resize(tsn,-1);			costsVal[i].resize(tsn,FLT_MAX);
		nodesVal[i].resize(tsn,-1);			prevsVal[i].resize(tsn,-1);
	}

	std::vector<long long> frontNodes = m_expandPoleSequence[0].front;
	int pole = m_expandPoleSequence[0].nodeID;
	visitedPole[pole]=true;
/*
	costs[0] = m_poleGraphNodeWeight[pole];
	for(int i=0;i<nodes[0].size();i++)
		nodes[0][i]=i;
*/
	//init first front state;
	std::vector<int> ind = getMinValuesPos(m_poleGraphNodeWeight[pole],stateInd[0].size());
	for(int i=0;i<stateInd[0].size();i++){
		stateInd[0][i] = double(ind[i]);
		costsVal[0][i] = m_poleGraphNodeWeight[pole][ind[i]];
		nodesVal[0][i] = ind[i];
	}

	std::vector<long long> bases;
	std::vector<long long> frontSize=frontNodes;
	for(int i=0;i<frontSize.size();i++)
		frontSize[i]=numOfNode[frontSize[i]];
	getBases(frontSize,bases);

	std::vector<long long> statelist =stateInd[0];
/*
	statelist.push_back(0);
*/
	for(int i=1;i<numOfPole;i++){

		std::vector<long long> prevfrontNodes = frontNodes;
		std::vector<long long> prevbases= bases;
		frontNodes = m_expandPoleSequence[i].front;
		pole = m_expandPoleSequence[i].nodeID;
		frontSize=frontNodes;
		for(int j=0;j<frontSize.size();j++)
			frontSize[j]=numOfNode[frontSize[j]];
		getBases(frontSize,bases);

		std::vector<long long> pos = frontNodes;
		std::vector<long long> temp = prevfrontNodes;
		temp.push_back(pole);
		for(int j=0;j<pos.size();j++){
			for(int k=0;k<temp.size();k++){
				if(pos[j]==temp[k]){
					pos[j]=k; break;
				}
			}
		}
		std::vector<int> prevfrontind(numOfPole,-1);
		for(int j=0;j<prevfrontNodes.size();j++)
			prevfrontind[prevfrontNodes[j]]=j;

		std::vector<std::pair<int,int> > tarcs;
		std::vector<int> tpoles;
		for(int j=0;j<adjcentPoles[pole].size();j++){
			if(visitedPole[adjcentPoles[pole][j]]==true){
				tarcs.push_back(adjcentArcs[pole][j]);
				tpoles.push_back(prevfrontind[adjcentPoles[pole][j]]);
			}
		}

/*
		std::vector<double> tempcosts(nState,FLT_MAX);
		std::vector<int> tempstates(nState,-1);
*/
		std::vector<double> tempcosts = costsVal[i];
		std::vector<long long> tempstates = stateInd[i];
		std::vector<long long> tempprevs = prevsVal[i];
		std::vector<int> tempnodes = nodesVal[i];

		for(int s=0;s<statelist.size();s++){
			long long state = statelist[s];
			if(state<0) continue;
			int stateid = s;
			std::vector<long long> previnds;
			indexToIndices(state,prevbases,previnds);

			for(int n=0;n<numOfNode[pole];n++){
				double c = costsVal[i-1][stateid]+m_poleGraphNodeWeight[pole][n];
				for(int j=0;j<tarcs.size();j++){
					if(tarcs[j].second==0){
						if(c>=FLT_MAX || m_poleGraphArcsWeight[tarcs[j].first][n][previnds[tpoles[j]]]>=FLT_MAX)
							c = FLT_MAX;
						else
							c+=m_poleGraphArcsWeight[tarcs[j].first][n][previnds[tpoles[j]]];
					}
					else{
						if(c>=FLT_MAX || m_poleGraphArcsWeight[tarcs[j].first][previnds[tpoles[j]]][n]>=FLT_MAX)
							c = FLT_MAX;
						else
							c+=m_poleGraphArcsWeight[tarcs[j].first][previnds[tpoles[j]]][n];
					}
				}
				std::vector<long long> indices;
				previnds.push_back((long long)(n));
				for(int j=0;j<pos.size();j++){
					indices.push_back(previnds[pos[j]]);
				}
				previnds.pop_back();
				long long index = indicesToIndex(indices,bases);
				
				std::vector<long long>::iterator pSt = std::find(tempstates.begin(),tempstates.end(),index);
				if(pSt==tempstates.end()){
					std::vector<double>::iterator imax = std::max_element(tempcosts.begin(),tempcosts.end());
					int maxid = imax - tempcosts.begin();
					if(c<*imax){
						tempcosts[maxid]=c;
						tempstates[maxid]=index;
						tempprevs[maxid]=state;
						tempnodes[maxid]=n;
					}

					if(c>=FLT_MAX && tempcosts[maxid]>=FLT_MAX){
						tempcosts[maxid]=100000;
						tempstates[maxid]=index;
						tempprevs[maxid]=state;
						tempnodes[maxid]=n;
					}
				}
				else{
					if(tempcosts[pSt-tempstates.begin()] > c){
						int ti = pSt-tempstates.begin();
						tempcosts[ti] = c;
						tempstates[ti]=index;
						tempprevs[ti]=state;
						tempnodes[ti]=n;
					}
				}
			}
		}
		visitedPole[pole]=true;//Very important! when put before get tars&tpoles, and in multi self-loop in the same pole case, it crash...
		costsVal[i]= tempcosts;
		stateInd[i]= tempstates;
		prevsVal[i]= tempprevs;
		nodesVal[i]= tempnodes;
		statelist = tempstates;
	}
	std::vector<int> res(numOfPole);
	long long state=0;

	for(int i=numOfPole-1;i>=0;i--){
		std::vector<long long>::iterator pstate = std::find(stateInd[i].begin(),stateInd[i].end(),state);
		int stateid = pstate - stateInd[i].begin();
		res[m_expandPoleSequence[i].nodeID]=nodesVal[i][stateid];
		state=prevsVal[i][stateid];
	}

	m_selectedNodeInPole=res;
}
void cycleUtils::searchMinPoleGraph()
{
	if(m_curveNet.arcs.empty() || m_twistTables.empty() || m_poleGraphNodes.empty() 
		|| m_poleGraphArcsWeight.empty() || m_expandPoleSequence.empty())
		return;

	int numOfPole = m_poleGraphNodeWeight.size();

	std::vector<std::vector<int> > adjcentPoles(numOfPole);
	std::vector<std::vector<std::pair<int,int> > > adjcentArcs(numOfPole);
	for(int i=0;i<m_curveNet.arcs.size();i++){
		int nodeID[]={m_curveNet.arcs[i].endNodesID.first,m_curveNet.arcs[i].endNodesID.second};
		adjcentPoles[nodeID[0]].push_back(nodeID[1]);
		adjcentPoles[nodeID[1]].push_back(nodeID[0]);
		std::pair<int,int> temp; temp.first=i; temp.second=0;
		adjcentArcs[nodeID[0]].push_back(temp);
		temp.second=1;
		adjcentArcs[nodeID[1]].push_back(temp);
	}
	std::vector<bool> visitedPole(numOfPole,false);
	std::vector<int> numOfNode(numOfPole);
	for(int i=0;i<numOfPole;i++)
		numOfNode[i]=m_poleGraphNodeWeight[i].size();

	std::vector<std::vector<double> > costs(numOfPole);
	std::vector<std::vector<int> > prevs(numOfPole);
	std::vector<std::vector<int> > nodes(numOfPole);
	for(int i=0;i<numOfPole;i++){
		costs[i].resize(m_expandPoleSequence[i].stateNum,FLT_MAX);
		prevs[i].resize(m_expandPoleSequence[i].stateNum,-1);
		nodes[i].resize(m_expandPoleSequence[i].stateNum,-1);
	}

	std::vector<long long> frontNodes = m_expandPoleSequence[0].front;
	int pole = m_expandPoleSequence[0].nodeID;
	visitedPole[pole]=true;
	costs[0] = m_poleGraphNodeWeight[pole];
	for(int i=0;i<nodes[0].size();i++)
		nodes[0][i]=i;

	std::vector<long long> bases;
	std::vector<long long> frontSize=frontNodes;
	for(int i=0;i<frontSize.size();i++)
		frontSize[i]=numOfNode[frontSize[i]];
	getBases(frontSize,bases);

	for(int i=1;i<numOfPole;i++){
		std::vector<long long> prevfrontNodes = frontNodes;
		std::vector<long long> prevbases= bases;
		frontNodes = m_expandPoleSequence[i].front;
		pole = m_expandPoleSequence[i].nodeID;
		visitedPole[pole]=true;
		frontSize=frontNodes;
		for(int j=0;j<frontSize.size();j++)
			frontSize[j]=numOfNode[frontSize[j]];
		getBases(frontSize,bases);

		std::vector<long long> pos = frontNodes;
		std::vector<long long> temp = prevfrontNodes;
		temp.push_back(pole);
		for(int j=0;j<pos.size();j++){
			for(int k=0;k<temp.size();k++){
				if(pos[j]==temp[k]){
					pos[j]=k; break;
				}
			}
		}
		std::vector<int> prevfrontind(numOfPole,-1);
		for(int j=0;j<prevfrontNodes.size();j++)
			prevfrontind[prevfrontNodes[j]]=j;

		std::vector<std::pair<int,int> > tarcs;
		std::vector<int> tpoles;
		for(int j=0;j<adjcentPoles[pole].size();j++){
			if(visitedPole[adjcentPoles[pole][j]]==true){
				tarcs.push_back(adjcentArcs[pole][j]);
				tpoles.push_back(prevfrontind[adjcentPoles[pole][j]]);
			}
		}

		int prevStateNum = m_expandPoleSequence[i-1].stateNum;
		for(int s=0;s<prevStateNum;s++){
			std::vector<long long> previnds;
			indexToIndices(s,prevbases,previnds);

			for(int n=0;n<numOfNode[pole];n++){
				double c = costs[i-1][s]+m_poleGraphNodeWeight[pole][n];
				//update c;
				for(int j=0;j<tarcs.size();j++){
					if(tarcs[j].second==0)
						c+=m_poleGraphArcsWeight[tarcs[j].first][n][previnds[tpoles[j]]];
					else
						c+=m_poleGraphArcsWeight[tarcs[j].first][previnds[tpoles[j]]][n];
				}
				std::vector<long long> indices;
				previnds.push_back(n);
				for(int j=0;j<pos.size();j++){
					indices.push_back(previnds[pos[j]]);
				}
				previnds.pop_back();
				long long index = indicesToIndex(indices,bases);
				if(c<costs[i][index]){
					costs[i][index]=c;
					prevs[i][index]=s;
					nodes[i][index]=n;
				}
			}
		}
	}
	std::vector<int> res(numOfPole);
	int state=0;
	for(int i=numOfPole-1;i>=0;i--){
		res[m_expandPoleSequence[i].nodeID]=nodes[i][state];
		state=prevs[i][state];
	}
	m_selectedNodeInPole=res;
}
bool mySort3(std::pair<int,int> i,std::pair<int,int> j) 
{ 
	return (i.second<j.second); 
}
void cycleUtils::updateRotationGraphbyPoleGraph()
{
	if(m_curveNet.arcs.empty() || m_twistTables.empty() || m_poleGraphNodes.empty() 
		|| m_poleGraphArcsWeight.empty() || m_expandPoleSequence.empty() || m_selectedNodeInPole.empty())
		return;

	m_rotationGraph.clear();
	for(int i=0;i<m_selectedNodeInPole.size();i++){
		m_rotationGraph.push_back(m_poleGraphNodes[i][m_selectedNodeInPole[i]]);
	}

	Graph net = m_curveNet;
	for(int i=0;i<net.arcs.size();i++){
		int nodeID[]={net.arcs[i].endNodesID.first,net.arcs[i].endNodesID.second};
		int arcID[]={net.arcs[i].posInNode.first,net.arcs[i].posInNode.second};
		int cap = net.arcs[i].capacity;

		std::vector<std::pair<int,int> > pairs;
		for(int j=0;j<cap;j++){
			std::pair<int,int> temp;
			for(int k=0;k<cap;k++){
				temp.first=j; temp.second=k+cap;
				pairs.push_back(temp);
			}
		}
		std::vector<std::pair<int,int> > optpair = m_poleGraphArcsMatch[i][m_selectedNodeInPole[nodeID[0]]][m_selectedNodeInPole[nodeID[1]]];
		std::vector<std::pair<int,int> > tpairs;
		for(int j=0;j<optpair.size();j++)
			tpairs.push_back(pairs[optpair[j].first]);
		sort(tpairs.begin(),tpairs.end(),mySort3);
		std::vector<int> order;
		for(int j=0;j<tpairs.size();j++)
			order.push_back(tpairs[j].first);

		tpairs=m_rotationGraph[nodeID[0]][arcID[0]];
		for(int j=0;j<order.size();j++)
			m_rotationGraph[nodeID[0]][arcID[0]][j]=tpairs[order[j]];

		for(int j=0;j<cap;j++){
			std::pair<int,int> temp; temp.first=arcID[0]; temp.second=j;
			m_rotationGraph[nodeID[0]][ m_rotationGraph[nodeID[0]][arcID[0]][j].first ]
			[m_rotationGraph[nodeID[0]][arcID[0]][j].second] = temp;
		}
	}
}
void cycleUtils::constructRotationGraphbyPoleGraph()
{
	if(m_curveNet.arcs.empty())
		return;

	/* test data  compare with ground truth, make sure getting right results*/

/*
	m_poleGraphNodeWeight.resize(4);
	m_poleGraphNodeWeight[0].push_back(.5);
	m_poleGraphNodeWeight[0].push_back(.5);
	m_poleGraphNodeWeight[1].push_back(.5);
	m_poleGraphNodeWeight[2].push_back(1.);
	m_poleGraphNodeWeight[2].push_back(1.);
	m_poleGraphNodeWeight[3].push_back(1.);
	m_poleGraphNodeWeight[3].push_back(2.);
*/

/*
	m_poleGraphArcs.resize(4);
	m_poleGraphArcs[0].resize(2);
	m_poleGraphArcs[0][0].push_back(.5);
	m_poleGraphArcs[0][1].push_back(.5);

	m_poleGraphArcs[2].resize(1);
	m_poleGraphArcs[2][0].push_back(.5);
	m_poleGraphArcs[2][0].push_back(0);

	m_poleGraphArcs[1].resize(2);
	m_poleGraphArcs[1][0].push_back(.5);
	m_poleGraphArcs[1][0].push_back(0);
	m_poleGraphArcs[1][1].push_back(.5);
	m_poleGraphArcs[1][1].push_back(0);

	m_poleGraphArcs[3].resize(2);
	m_poleGraphArcs[3][0].push_back(.5);
	m_poleGraphArcs[3][0].push_back(0);
	m_poleGraphArcs[3][1].push_back(.5);
	m_poleGraphArcs[3][1].push_back(0);
*/

/*
	int f1[4][4] ={{2, 1, 3, 4}, {4, 1, 2, 3}, {1, 3, 2, 4}, {2, 3, 1, 4}};
	int s2[4][4] ={{1, 3, 2, 4}, {3, 4, 1, 2}, {4, 2, 3, 1}, {3, 2, 1, 4}};
	std::vector<std::vector<int> > firstGroup(4);
	std::vector<std::vector<int> > secondGroup(4);
	for(int i=0;i<4;i++){
		for(int j=0;j<4;j++){
			firstGroup[i].push_back(f1[i][j]-1);
			secondGroup[i].push_back(s2[i][j]-1);
		}
	}
	std::vector<int> res;
	m_graphSearch.stableMatching(firstGroup,secondGroup,res);
*/

/*
	int capsi[]={2,1,3};
	std::vector<int> caps;
	for(int i=0;i<3;i++)
		caps.push_back(capsi[i]);
	std::vector<std::vector<int> > res= enumSets(caps,2);
*/

/*
	int tempNode[] = {m_curveNet.arcs[0].endNodesID.first,m_curveNet.arcs[0].endNodesID.second};
	std::vector<Point> tempPts = m_curveNet.arcs[0].vertexList;
	tempPts.push_back(m_curveNet.nodes[tempNode[1]].pos);
	tempPts.insert(tempPts.begin(),m_curveNet.nodes[tempNode[0]].pos);
	std::vector<std::vector<double> > transportMatrices;
	computeTransportMatrixAll(tempPts,transportMatrices);
*/
	m_twistTables.clear();
	m_twistTablesConfidence.clear();
	m_twistTablesIndex.clear();

	m_poleGraphNodes.clear();
	m_poleGraphNodeWeight.clear();
	m_poleGraphArcsWeight.clear();
	m_poleGraphArcsMatch.clear();
	m_expandPoleSequence.clear();
	m_selectedNodeInPole.clear();

	if(m_userDefinedPairsInNode.empty())
		m_userDefinedPairsInNode.resize(m_curveNet.nodes.size());
	if(m_userDefinedPairsInArc.empty())
		m_userDefinedPairsInArc.resize(m_curveNet.arcs.size());

	computeTwistTables();
	constructJointRotationGraphbyPoleGraph();
	constructSegmentRotationGraphbyPoleGraph();
	constructExpandSequence();
	searchMinPoleGraph();
	updateRotationGraphbyPoleGraph();
	clock_t  time_end = clock();
	m_stateUnLimited = true;
}
void cycleUtils::constructRotationGraphbyPoleGraphEx()
{
	if(m_curveNet.arcs.empty())
		return;

		m_twistTables.clear();
		m_twistTablesConfidence.clear();
		m_twistTablesIndex.clear();

		m_latestUpdateNodes.clear();
		m_latestUpdateArcs.clear();

		m_poleGraphNodes.clear();
		m_poleGraphNodeWeight.clear();
		m_poleGraphArcsWeight.clear();
		m_poleGraphArcsMatch.clear();
		m_expandPoleSequence.clear();
		m_selectedNodeInPole.clear();

	if(m_userDefinedPairsInNode.empty())
		m_userDefinedPairsInNode.resize(m_curveNet.nodes.size());
	if(m_userDefinedPairsInArc.empty())
		m_userDefinedPairsInArc.resize(m_curveNet.arcs.size());
	

	if(m_twistTables.empty()){
		clock_t  time_str = clock();
		computeTwistTables();
	}
	if(m_poleGraphNodeWeight.empty() || !m_latestUpdateNodes.empty()){
		constructJointRotationGraphbyPoleGraph();
		constructSegmentRotationGraphbyPoleGraph();
		constructExpandSequence();
		searchAlmostMinPoleGraph();
		updateRotationGraphbyPoleGraph();
		m_triangleSurface.clear();
	}
	else if(m_poleGraphArcsWeight.empty()|| !m_latestUpdateArcs.empty()){
		constructSegmentRotationGraphbyPoleGraph();
		constructExpandSequence();
		searchAlmostMinPoleGraph();
		updateRotationGraphbyPoleGraph();
		m_triangleSurface.clear();
	}
	else if(m_selectedNodeInPole.empty()){
		searchAlmostMinPoleGraph();
		updateRotationGraphbyPoleGraph();
		m_triangleSurface.clear();
	}
	m_stateUnLimited = false;
}
void cycleUtils::constructRandomRotationGraph()
{
	if(m_curveNet.arcs.empty())
		return;

	Graph net = m_curveNet;
	RotationGraph roGraph;

	//initialization
	std::vector<std::vector<std::vector<bool> > > usedList;
	for(int i=0;i<net.nodes.size();i++){
		std::vector<std::vector<bool> > usedNode;
		int nodeSize = net.nodes[i].arcID.size();
		for(int j=0;j<nodeSize;j++){
			std::vector<bool> usedNodeArcs;
			usedNodeArcs.assign(net.arcs[net.nodes[i].arcID[j]].capacity,false);
			usedNode.push_back(usedNodeArcs);
		}
		usedList.push_back(usedNode);
	}
	for(int i=0;i<net.nodes.size();i++){
		std::vector<std::vector<std::pair<int,int> > > jointRoGraph;
		for(int j=0;j<net.nodes[i].arcID.size();j++){
			std::vector<std::pair<int,int> > segRoGraph;
			segRoGraph.assign(net.arcs[net.nodes[i].arcID[j]].capacity,
								std::pair<int,int>(0,0));
			jointRoGraph.push_back(segRoGraph);
		}
		roGraph.push_back(jointRoGraph);
	}
	//end of initialization

	for(int i=0;i<net.nodes.size();i++){
		int nodeSize = net.nodes[i].arcID.size();
		std::vector<int> nodeArcsCapacity;
		for(int j=0;j<nodeSize;j++){
			nodeArcsCapacity.push_back(net.arcs[net.nodes[i].arcID[j]].capacity);
		}
		for(int j=0;j<nodeSize;j++){
			for(int k=0;k<nodeArcsCapacity[j];k++){
				if(usedList[i][j][k] == true)
					continue;
				usedList[i][j][k] = true;
				std::pair<int,unsigned int> connectSeg;
				//random {E,i}
				while(true){
					connectSeg.first = rand() % nodeSize;
					connectSeg.second = rand() % (nodeArcsCapacity[connectSeg.first]);
					if(!usedList[i][connectSeg.first][connectSeg.second]) break;
				}
				usedList[i][connectSeg.first][connectSeg.second] = true;
				roGraph[i][j][k] = connectSeg;
				roGraph[i][connectSeg.first][connectSeg.second]= std::pair<int ,unsigned int>(j,k);
			}
		}
	}
	m_rotationGraph = roGraph;
}
void cycleUtils::constructCycles()
{
	if(m_curveNet.arcs.empty() || m_twistTables.empty() || m_poleGraphNodes.empty()|| m_poleGraphArcsWeight.empty()
		|| m_expandPoleSequence.empty() || m_selectedNodeInPole.empty() || m_rotationGraph.empty())
		return;
	
	Graph net = m_curveNet;
	RotationGraph roGraph = m_rotationGraph;
	CycleSet cycles;

	std::vector<std::vector<bool> > usedList;
	for(int i=0;i<net.arcs.size();i++){
		std::vector<bool> usedInstances;
		usedInstances.assign(net.arcs[i].capacity,false);
		usedList.push_back(usedInstances);
	}

	for(int i=0;i<net.arcs.size();i++){
		for(int j=0;j<net.arcs[i].capacity;j++){
			if(usedList[i][j])
				continue;
			Cycle cycle;
			int currentArcID = i;
			int currentArcInstance = j;
			int currentDirection=1;
			while(!usedList[currentArcID][currentArcInstance]){
				usedList[currentArcID][currentArcInstance]=true;
				CycleSegment cycleSegment;
				cycleSegment.arcID = currentArcID;
				cycleSegment.instanceID = currentArcInstance;
				cycleSegment.strEndID = currentDirection;
				cycle.push_back(cycleSegment);

				int nodeID,arcID;
				if(currentDirection==1){
					nodeID=net.arcs[currentArcID].endNodesID.second;
					arcID=net.arcs[currentArcID].posInNode.second;
				}
				else{
					nodeID=net.arcs[currentArcID].endNodesID.first;
					arcID=net.arcs[currentArcID].posInNode.first;
				}
				int nextArcID;
				nextArcID=roGraph[nodeID][arcID][currentArcInstance].first;
				currentArcInstance=roGraph[nodeID][arcID][currentArcInstance].second;
				currentArcID = net.nodes[nodeID].arcID[nextArcID];
				currentDirection=net.nodes[nodeID].arcDirection[nextArcID];
			}
			cycles.push_back(cycle);
		}
	}
	m_cycleSet = cycles;
	m_isCycleBreak=false;
}

void cycleUtils::cycleBreaking()
{
	if(m_curveNet.arcs.empty() || m_twistTables.empty() || m_poleGraphNodes.empty()|| m_poleGraphArcsWeight.empty()
		|| m_expandPoleSequence.empty() || m_selectedNodeInPole.empty() || m_rotationGraph.empty() || m_cycleSet.empty())
		return;

	Graph net = m_curveNet;
	CycleSet cyclesOrig = m_cycleSet;
	RotationGraph roGraph = m_rotationGraph;
	m_rotationGraphUpdate = m_rotationGraph;
	m_cycleSetBreaked = m_cycleSet;

	bool found;
	bool isbreak = false;
	std::vector<int> tbad;
	m_breakNum=0;

		do{
			tbad.clear();
			found=false;
			for(int i=0;i<cyclesOrig.size();i++){
				std::vector<int > nodeID;
				std::vector<std::vector<std::pair<int,int> > > arcPairs;
				for(int j=0;j<cyclesOrig[i].size();j++){
					CycleSegment currentSeg = cyclesOrig[i][j];
					int currentPosition,arcID,instanceID;
					if (currentSeg.strEndID==1){
						currentPosition= net.arcs[currentSeg.arcID].endNodesID.second;
						arcID = net.arcs[currentSeg.arcID].posInNode.second;
					}
					else{
						currentPosition= net.arcs[currentSeg.arcID].endNodesID.first;
						arcID = net.arcs[currentSeg.arcID].posInNode.first;
					}
					int index = nodeID.size();
					for(int k=0;k<nodeID.size();k++){
						if(nodeID[k]==currentPosition){
							index = k;break;}
					}
					if(index==nodeID.size()){
						nodeID.push_back(currentPosition);
						std::vector<std::pair<int,int> > arcPair;
						arcPairs.push_back(arcPair);
					}
					std::pair<int,int> temp; temp.first=arcID; temp.second=currentSeg.instanceID;
					arcPairs[index].push_back(temp);
					int nextArcID = roGraph[currentPosition][arcID][currentSeg.instanceID].first;
					instanceID = roGraph[currentPosition][arcID][currentSeg.instanceID].second;
					temp.first=nextArcID; temp.second=instanceID;
					arcPairs[index].push_back(temp);
				}
				for(int j=0;j<nodeID.size();j++){
					arcPairs[j].push_back(arcPairs[j].front());
					arcPairs[j].erase(arcPairs[j].begin());
					for(int k=1;k<arcPairs[j].size();k++){
						if(arcPairs[j][k].first==arcPairs[j][k-1].first){
							arcPairs[j].clear();
							tbad.push_back(i);break;
						}
					}
				}
				int	size =0;
				int index=0;
				for(int j=0;j<nodeID.size();j++){
					if(size<arcPairs[j].size()){
						size = arcPairs[j].size();
						index = j;
					}
				}
				if(size<=2) continue;
				found = true;
				if(!m_isDoCycleBreak)
					break;

				for(int j=0;j<arcPairs[index].size();j+=2){
					roGraph[nodeID[index]][arcPairs[index][j].first][arcPairs[index][j].second]=arcPairs[index][j+1];
					roGraph[nodeID[index]][arcPairs[index][j+1].first][arcPairs[index][j+1].second]=arcPairs[index][j];
				}
			}

			if(!m_isDoCycleBreak)
				break;

			if(found){
				m_breakNum++;
				m_rotationGraphUpdate = m_rotationGraph;
				m_rotationGraph=roGraph;
				m_cycleSetBreaked = m_cycleSet;
				constructCycles();
				cyclesOrig=m_cycleSet;
				m_cycleSet=m_cycleSetBreaked;
				m_cycleSetBreaked=cyclesOrig;
				m_rotationGraph=m_rotationGraphUpdate;
				m_rotationGraphUpdate=roGraph;
				isbreak = true;
			}
		}while(found);


	m_isCycleBreak=isbreak;

//	computeArcCost();
	computeCycleCost();
	for (int i=0;i<tbad.size();i++){
		m_cyclesCost[tbad[i]]=1;
	}
}
void cycleUtils::computeArcCost()
{
	m_arcsCost.clear();
	double totalCost=0;
	int totalNum=0;
	double maxN=0;
	double minN=FLT_MAX;
	std::vector<double> possiWorses(m_curveNet.arcs.size(),0);
	for(int i=0;i<m_curveNet.arcs.size();i++){
		int nodeID[]={m_curveNet.arcs[i].endNodesID.first,m_curveNet.arcs[i].endNodesID.second};
		int pos[]={m_selectedNodeInPole[nodeID[0]],m_selectedNodeInPole[nodeID[1]]};
		double cost = m_poleGraphArcsWeight[i][pos[0]][pos[1]];
		m_arcsCost.push_back(cost);
		int cap = m_curveNet.arcs[i].capacity;
		double possiWorse = m_twistWeight*M_PI + m_angleWeight*M_PI + m_dihedralWeight*M_PI*2*(cap-1);
		possiWorses[i]=possiWorse;
		if(cost<FLT_MAX){
			totalCost+=cost;
			totalNum++;
			if(cost>maxN)
				maxN = cost;
			if(cost<minN)
				minN = cost;
		}
	}
//	double margin = maxN - minN;
//	double possiWorse = *std::max_element(possiWorses.begin(),possiWorses.end());
	double possiWorse = maxN - minN;
	for(int i=0;i<m_arcsCost.size();i++){
		if(m_arcsCost[i]<FLT_MAX)
            m_arcsCost[i] = std::min(1.0,(m_arcsCost[i]-minN)/possiWorse);
		else
			m_arcsCost[i] = 1;
//		m_arcsCost[i] = 1-m_arcsCost[i];
	}
}
void cycleUtils::computeCycleCost()
{
	m_cyclesCost.clear();
	double totalCost=0;
	int totalNum=0;
	double maxN=0;
	double minN=FLT_MAX;

//	std::vector<double> possiWorses(m_cycleSetBreaked.size(),0);
	for(int i=0;i<m_cycleSetBreaked.size();i++){
		double tc=0;
		Cycle tcycle = m_cycleSetBreaked[i];
/*
		tcycle.insert(tcycle.begin(),tcycle.back());
		tcycle.push_back(tcycle[1]);
*/
		for(int j=0;j<tcycle.size();j++){
			int arcID=tcycle[j].arcID;
			int nodeID[]={m_curveNet.arcs[arcID].endNodesID.first,
				m_curveNet.arcs[arcID].endNodesID.second};
			int pos[]={m_selectedNodeInPole[nodeID[0]],m_selectedNodeInPole[nodeID[1]]};
			if(tc==FLT_MAX || m_poleGraphArcsWeight[arcID][pos[0]][pos[1]]==FLT_MAX)
				tc = FLT_MAX;
			else
				tc+= m_poleGraphArcsWeight[arcID][pos[0]][pos[1]];

/*
			int a1 = tcycle[j-1].arcID;
			int a2 = tcycle[j+1].arcID;
			int aind1,aind2;
			if(m_curveNet.arcs[a1].endNodesID.first==nodeID[0]){
				aind1 = m_curveNet.arcs[a1].posInNode.first;
			}
			else if(m_curveNet.arcs[a1].endNodesID.second==nodeID[0]){
				aind1 = m_curveNet.arcs[a1].posInNode.second;
			}
			else if(m_curveNet.arcs[a2].endNodesID.first==nodeID[0]){
				aind1 = m_curveNet.arcs[a2].posInNode.first;
			}
			else if(m_curveNet.arcs[a2].endNodesID.second==nodeID[0]){
				aind1 = m_curveNet.arcs[a2].posInNode.second;
			}

			if(m_curveNet.arcs[a1].endNodesID.first==nodeID[1]){
				aind2 = m_curveNet.arcs[a1].posInNode.first;
			}
			else if(m_curveNet.arcs[a1].endNodesID.second==nodeID[1]){
				aind2 = m_curveNet.arcs[a1].posInNode.second;
			}
			else if(m_curveNet.arcs[a2].endNodesID.first==nodeID[1]){
				aind2 = m_curveNet.arcs[a2].posInNode.first;
			}
			else if(m_curveNet.arcs[a2].endNodesID.second==nodeID[1]){
				aind2 = m_curveNet.arcs[a2].posInNode.second;
			}			
			tc+=m_twistTables[arcID][aind1][aind2];
*/
		}
		double mincost = (tcycle.size()-2)*M_PI*m_angleWeight;
		m_cyclesCost.push_back(abs(tc)-mincost);

		if(abs(tc)<FLT_MAX){
			if(tc>maxN)
				maxN = tc;
			if(tc<minN)
				minN = tc;
		}
	}
//	double margin = maxN - minN;
//	double possiWorse = *std::max_element(possiWorses.begin(),possiWorses.end());
//	double possiWorse = *std::max_element(m_cyclesCost.begin(),m_cyclesCost.end());
	for(int i=0;i<m_cyclesCost.size();i++){
		if(m_cyclesCost[i]<FLT_MAX){
			m_cyclesCost[i] = std::min(1.0,(m_cyclesCost[i])/maxN);
			m_cyclesCost[i] = std::max(0.0,m_cyclesCost[i]);
		}
		else
			m_cyclesCost[i] = 1;
//		m_cyclesCost[i] = 1-m_cyclesCost[i];
	}
}

void cycleUtils::surfaceBuilding()
{
	if(m_curveNet.arcs.empty()/* || m_twistTables.empty() || m_poleGraphNodes.empty()|| m_poleGraphArcsWeight.empty()
		|| m_expandPoleSequence.empty() || m_selectedNodeInPole.empty() || m_rotationGraph.empty() || m_cycleSet.empty() */||m_cycleSetBreaked.empty())
		return;

	m_cycleNormal.clear();
	m_newPoints.clear();
	m_newNormals.clear();
	m_newPointNum.clear();

	Graph net = m_curveNet;
	CycleSet cycles = m_cycleSetBreaked;
	TriangleSurface surface;
	TriangleSurface norms;

	for(int c=0;c<cycles.size();c++){
		Cycle cycle = cycles[c];
		std::vector<int> arcs;
		LinearCurveNet curves;
		for(int i=0;i<cycle.size();i++){
			int arcID = cycle[i].arcID;			
			curves.push_back(net.arcs[arcID].vertexList);
			int strNode = m_curveNet.arcs[arcID].endNodesID.first;
			int endNode = m_curveNet.arcs[arcID].endNodesID.second;
			curves.back().insert(curves.back().begin(),m_curveNet.nodes[strNode].pos);
			curves.back().push_back(m_curveNet.nodes[endNode].pos);
			if(cycle[i].strEndID==2){
				reverse(curves.back().begin(),curves.back().end());
			}
		}
		Curve pointList;
		for(int i=0;i<curves.size();i++){
			for(int j=0;j<curves[i].size()-1;j++)
				pointList.push_back(curves[i][j]);
		}
		double* points;
		int point_num= pointList.size();
		points= new double[point_num*3];
		for(int i=0;i<pointList.size();i++){
			points[i*3+0]=pointList[i].x;
			points[i*3+1]=pointList[i].y;
			points[i*3+2]=pointList[i].z;
		}
		double* tile_list;
		int tileNum;
		float weights[]={float(m_weightTri),
			float(m_weightEdge),float(m_weightBiTri),float(m_weightTriBd),float(m_weightWorsDih)};

		int res; bool dosmooth=m_SurfaceSmooth; int subs=m_subdivisonSmooth;int laps=m_laplacianSmooth;
		double* newPoints;
		float* newNormals;
		int newPointNum;

		if(m_normalsTable.empty())
/*
			res=delaunayRestrictedTriangulation(points,point_num,&newPoints,&newPointNum,
			&tile_list,&tileNum,weights,dosmooth,subs,laps);
*/


//////////////////////////////////////
		if(!m_normalsTable.empty()){
			CycleSegment cyl = cycle.back();
			cycle.insert(cycle.begin(),cyl);
			cycle.push_back(cycle[1]);
			LinearCurveNet cycleNormal;
			LinearCurveNet endNorm;
			std::vector<double> variance;
			std::vector<int> dirts;
			for(int j=1;j<cycle.size()-1;j++){
				int arcID=cycle[j].arcID;
				int nodeID[]={m_curveNet.arcs[arcID].endNodesID.first,
					m_curveNet.arcs[arcID].endNodesID.second};
				int a1 = cycle[j-1].arcID;
				int a2 = cycle[j+1].arcID;
				int aind1=0;int aind2=0;
				if(m_curveNet.arcs[a1].endNodesID.first==nodeID[0]){
					aind1 = m_curveNet.arcs[a1].posInNode.first;
				}
				else if(m_curveNet.arcs[a1].endNodesID.second==nodeID[0]){
					aind1 = m_curveNet.arcs[a1].posInNode.second;
				}
				else if(m_curveNet.arcs[a2].endNodesID.first==nodeID[0]){
					aind1 = m_curveNet.arcs[a2].posInNode.first;
				}
				else if(m_curveNet.arcs[a2].endNodesID.second==nodeID[0]){
					aind1 = m_curveNet.arcs[a2].posInNode.second;
				}

				if(m_curveNet.arcs[a1].endNodesID.first==nodeID[1]){
					aind2 = m_curveNet.arcs[a1].posInNode.first;
				}
				else if(m_curveNet.arcs[a1].endNodesID.second==nodeID[1]){
					aind2 = m_curveNet.arcs[a1].posInNode.second;
				}
				else if(m_curveNet.arcs[a2].endNodesID.first==nodeID[1]){
					aind2 = m_curveNet.arcs[a2].posInNode.first;
				}
				else if(m_curveNet.arcs[a2].endNodesID.second==nodeID[1]){
					aind2 = m_curveNet.arcs[a2].posInNode.second;
				}

				cycleNormal.push_back(m_normalsTable[arcID][aind1][aind2]);

				if(cycle[j].strEndID==1){
					dirts.push_back(1);
				}
				else{
					dirts.push_back(2);
					reverse(cycleNormal.back().begin(),cycleNormal.back().end());
					for(int k=0;k<cycleNormal.back().size();k++){
						cycleNormal.back()[k] = -cycleNormal.back()[k];
					}
				}

				cycleNormal.back().pop_back();
				cycleNormal.back().erase(cycleNormal.back().begin());

				std::vector<Point> temp;
				temp.push_back(cycleNormal.back().front()); temp.push_back(cycleNormal.back().back());
				endNorm.push_back(temp);
				variance.push_back(m_twistTablesConfidence[arcID][aind1][aind2]);
			}

			std::vector<Point> tema = endNorm.front();
			std::vector<Point> temb = endNorm.back();
			endNorm.insert(endNorm.begin(),temb);
			endNorm.push_back(tema);
			double tvar = variance.back();
			variance.insert(variance.begin(),tvar);
			variance.push_back(variance[1]);

			for(int j=0;j<cycleNormal.size();j++){
				Point tempL = endNorm[j][1];
				Point tempR = endNorm[j+2][0];
				Curve pts = curves[j];

				std::vector<std::vector<double> > transportMatrices;
				computeTransportMatrixAll(pts,transportMatrices);
				std::vector<std::vector<Point> > transMatrices;
				for(int t=0;t<transportMatrices.size();t++){
					std::vector<Point> transMatrix;
					for(int j=0;j<3;j++){
						Point row;
						for(int k=0;k<3;k++){
							row[k]= transportMatrices[t][j+k*3];
						}
						transMatrix.push_back(row);
					}
					transMatrices.push_back(transMatrix);
				}
				std::vector<Point> nmlistL;
				for(int k=0;k<transMatrices.size();k++){
					std::vector<Point> transMatrix = transMatrices[k];
					Point tnm;
					for(int l=0;l<3;l++){
						tnm[l] = tempL.dot(transMatrix[l]);
					}
					tnm.normalize();
					nmlistL.push_back(tnm);
				}

				reverse(pts.begin(),pts.end());
				transportMatrices.clear();
				computeTransportMatrixAll(pts,transportMatrices);
				transMatrices.clear();
				for(int t=0;t<transportMatrices.size();t++){
					std::vector<Point> transMatrix;
					for(int j=0;j<3;j++){
						Point row;
						for(int k=0;k<3;k++){
							row[k]= transportMatrices[t][j+k*3];
						}
						transMatrix.push_back(row);
					}
					transMatrices.push_back(transMatrix);
				}
				std::vector<Point> nmlistR;
				for(int k=0;k<transMatrices.size();k++){
					std::vector<Point> transMatrix = transMatrices[k];
					Point tnm;
					for(int l=0;l<3;l++){
						tnm[l] = tempR.dot(transMatrix[l]);
					}
					tnm.normalize();
					nmlistR.push_back(tnm);
				}
				reverse(nmlistR.begin(),nmlistR.end());
				for(int k=0;k<cycleNormal[j].size();k++){
					Point tnm= variance[j]*nmlistL[k]+ variance[j+2]*nmlistR[k]+ cycleNormal[j][k];
					tnm.normalize();
					cycleNormal[j][k]=tnm;
				}
			}

			Curve normalList;
			for(int i=0;i<cycleNormal.size();i++){
				for(int j=0;j<cycleNormal[i].size();j++)
					normalList.push_back(cycleNormal[i][j]);
			}

			float* normals;
			int normal_num= normalList.size();
			normals= new float[normal_num*3];
			for(int i=0;i<normalList.size();i++){
				normals[i*3+0]=normalList[i].x;
				normals[i*3+1]=normalList[i].y;
				normals[i*3+2]=normalList[i].z;
			}

/*
			res=delaunayRestrictedTriangulation(points,normals,point_num,&newPoints,&newNormals,&newPointNum,&tile_list,&tileNum,weights,
				dosmooth,subs,laps);
*/
		
			LinearCurveNet cycleNormalForVis = cycleNormal;
			for(int j=0;j<cycleNormalForVis.size();j++){
				Point temp = Point(0,0,0);
				cycleNormalForVis[j].push_back(temp);
				if(dirts[j]==2)
					reverse(cycleNormalForVis[j].begin(),cycleNormalForVis[j].end());
			}
			m_cycleNormal.push_back(cycleNormalForVis);
	
			m_newPoints.push_back(newPoints);
			m_newNormals.push_back(newNormals);
			m_newPointNum.push_back(newPointNum);
		}
		///////////////////////////////////////

		TriangleCycle triangleCycle;
		TriangleCycle triangleCycleNorm;
		if(res==1){
			for ( int i=0; i<tileNum; i++){
				std::vector<Point> triangle;
				for (int j=0;j<3;j++){
					Point point;
					point.x = tile_list[i*9+j*3+0];
					point.y = tile_list[i*9+j*3+1];
					point.z = tile_list[i*9+j*3+2];
					triangle.push_back(point);
				}
				triangleCycle.push_back(triangle);
			}
			for ( int i=0; i<tileNum; i++){
				std::vector<Point> triangleNorm;
				for (int j=0;j<3;j++){
					Point norm;
					norm.x = newNormals[i*9+j*3+0];
					norm.y = newNormals[i*9+j*3+1];
					norm.z = newNormals[i*9+j*3+2];
					triangleNorm.push_back(norm);
				}
				triangleCycleNorm.push_back(triangleNorm);
			}
		}
		surface.push_back(triangleCycle);
		norms.push_back(triangleCycleNorm);
	}

/*  here, I'm trying make normals consistent along all patches; but something is wrong;
	std::vector<std::vector<unsigned> > adjList(m_curveNet.arcs.size());
	for(unsigned i=0;i<m_cycleSet.size();i++){
		for(unsigned j=0;j<m_cycleSet[i].size();j++){
			unsigned arcID = m_cycleSet[i][j].arcID;
			adjList[arcID].push_back(i);
		}
	}
	for(unsigned i=0;i<adjList.size();i++){
		for(unsigned j=0;j<adjList[i].size();j++)
			 cout<<"  "<<adjList[i][j];
		cout<<endl;
	}

	std::vector<std::vector<unsigned> > adjCycleList(m_cycleSet.size(),std::vector<unsigned>(1,m_cycleSet.size()));
	std::vector<std::vector<unsigned> > arcShared(m_cycleSet.size());
	for(unsigned i=0;i<adjList.size();i++){
		for(unsigned j=0;j<adjList[i].size()-1;j++){
			for(unsigned k=j+1;k<adjList[i].size();k++){
				if(std::find(adjCycleList[adjList[i][j]].begin(),adjCycleList[adjList[i][j]].end(),adjList[i][k])==
					adjCycleList[adjList[i][j]].end()){
					adjCycleList[adjList[i][j]].push_back(adjList[i][k]);
					arcShared[adjList[i][j]].push_back(i);
				}
				if(std::find(adjCycleList[adjList[i][k]].begin(),adjCycleList[adjList[i][k]].end(),adjList[i][j])==
					adjCycleList[adjList[i][k]].end()){
					adjCycleList[adjList[i][k]].push_back(adjList[i][j]);
					arcShared[adjList[i][k]].push_back(i);
				}
			}
		}
	}
	for(unsigned i=0;i<adjCycleList.size();i++){
		adjCycleList[i].erase(adjCycleList[i].begin());
	}
	for(unsigned i=0;i<adjCycleList.size();i++){
		for(unsigned j=0;j<adjCycleList[i].size();j++)
		 cout<<"  "<<adjCycleList[i][j];
		cout<<endl;
		for(unsigned j=0;j<arcShared[i].size();j++)
		 cout<<"  "<<arcShared[i][j];
		cout<<endl;
	}

	std::vector<bool> checkedList(m_cycleSet.size(),false);
	checkedList.front()=true;
	unsigned count=0;
	for(unsigned i=0;i<adjCycleList.size();i++){
		std::vector<unsigned> &adjCycles = adjCycleList[i];
		for(unsigned j=0;j<adjCycles.size();j++){
			if(checkedList[adjCycles[j]]==true) continue;
			unsigned adjArc = arcShared[i][j];
			std::vector<Point> p(3);
			if(!m_curveNet.arcs[adjArc].vertexList.empty())
				p[0] = m_curveNet.arcs[adjArc].vertexList.front();
			else
				p[0] = m_curveNet.nodes[m_curveNet.arcs[adjArc].endNodesID.first].pos; 

			unsigned edgeDirection[2]={3,3};
			std::vector<unsigned> ind;
			for(unsigned t=0;t<surface[i].size();t++){
				std::vector<Point> &triA = surface[i][t];
				unsigned tind=std::find(triA.begin(),triA.end(),p[0])-triA.begin();
				if(tind<3){
					ind.push_back(tind);ind.push_back((tind+1)%3);ind.push_back((tind+2)%3);
					p.push_back(triA[ind[1]]);p.push_back(triA[ind[2]]);
					cout<<"find"<<endl;
				}
			}
			std::vector<unsigned> ind2;
			for(unsigned t=0;t<surface[adjCycles[j]].size();t++){
				std::vector<Point> &triA = surface[adjCycles[j]][t];
				unsigned tind=std::find(triA.begin(),triA.end(),p[0])-triA.begin();
				if(tind<3){
					unsigned tind2=std::find(triA.begin(),triA.end(),p[1])-triA.begin();
					if(tind2<3){
						if((ind[1]-ind[0])==1 || (ind[0]-ind[1])==2)
							edgeDirection[0]=1;
						else
							edgeDirection[0]=2;
						if((tind-tind2)==1 || (tind-tind2)==2)
							edgeDirection[1]=1;
						else
							edgeDirection[1]=2;
						count++;
						break;
					}
					else{
						unsigned tind3=std::find(triA.begin(),triA.end(),p[2])-triA.begin();
						if(tind3<3){
							if((ind[1]-ind[2])==1 || (ind[0]-ind[2])==2)
								edgeDirection[0]=1;
							else
								edgeDirection[0]=2;
							if((tind-tind3)==1 || (tind-tind3)==2)
								edgeDirection[1]=1;
							else
								edgeDirection[1]=2;
							count++;
							break;					
						}
					}
				}

			}
			if(edgeDirection[0]==edgeDirection[1] && edgeDirection[0]!=3 &&edgeDirection[1]!=3 ){
				std::vector<std::vector<Point> > &triCyc = surface[adjCycles[j]];
				std::vector<std::vector<Point> > &normCyc = norms[adjCycles[j]];
				for(unsigned t=0;t<triCyc.size();t++){
					Point tp=triCyc[t].front();
					triCyc[t][0]=triCyc[t][1];
					triCyc[t][1]=tp;
					normCyc[t][0]=-normCyc[t][0];normCyc[t][1]=-normCyc[t][1];normCyc[t][2]=-normCyc[t][2];
				}
			}
			checkedList[adjCycles[j]]=true;
		}
	}

	cout<<"count:"<<count<<endl;
*/
	m_triangleSurface=surface;
	m_triangleSurfaceNormal=norms;
}

bool cycleUtils::updateConstraintList()
{
	if(m_curveNet.arcs.empty())
		return false;
	if(m_userDefinedPairsInNode.empty())
		m_userDefinedPairsInNode.resize(m_curveNet.nodes.size());
	if(m_userDefinedPairsInArc.empty())
		m_userDefinedPairsInArc.resize(m_curveNet.arcs.size());

	for(int i=0;i<m_selectArcList.back().size()-1;i++){
		for(int j=i+1;j<m_selectArcList.back().size();j++){
			if(m_selectArcList.back()[i]==m_selectArcList.back()[j]){
				m_selectArcList.back().erase(m_selectArcList.back().begin()+j);
				j--;
			}
		}
	}
	std::vector<std::vector<int> > jointArcs(m_curveNet.nodes.size());
	std::vector<std::vector<std::vector<int> > > adjArcs(m_curveNet.arcs.size(),std::vector<std::vector<int> >(2));
	std::vector<bool> usedArcs(adjArcs.size(),false);
	for(int i=0;i<m_selectArcList.back().size();i++){
		int a1 = m_selectArcList.back()[i];
		int n[] = {m_curveNet.arcs[a1].endNodesID.first,m_curveNet.arcs[a1].endNodesID.second};
		jointArcs[n[0]].push_back(a1);
		jointArcs[n[1]].push_back(a1);

		usedArcs[a1]=true;
		std::vector<int> leftAdj = m_curveNet.nodes[n[0]].arcID;
		std::vector<int> rightAdj = m_curveNet.nodes[n[1]].arcID;
		std::vector<int> leftAdjDir = m_curveNet.nodes[n[0]].arcDirection;
		std::vector<int> rightAdjDir = m_curveNet.nodes[n[1]].arcDirection;
		for(int j=0;j<leftAdj.size();j++){
			if(leftAdj[j]!=a1){
				adjArcs[leftAdj[j]][leftAdjDir[j]-1].push_back(a1);
			}
		}
		for(int j=0;j<rightAdj.size();j++){
			if(rightAdj[j]!=a1)
				adjArcs[rightAdj[j]][rightAdjDir[j]-1].push_back(a1);
		}
	}

	bool isRightCycle = true;
	// data check;
	for(int i=0;i<jointArcs.size();i++){
		if(jointArcs[i].size()>2){
			isRightCycle = false; break;
		}
	}
	if(!isRightCycle)
		return isRightCycle;

	isRightCycle = false;
	for(int i=0;i<jointArcs.size();i++){
		if(jointArcs[i].empty()||jointArcs[i].size()==1)
			continue;
		isRightCycle=true;
		int a1 = jointArcs[i][0];
		int a2 = jointArcs[i][1];
		int order1,order2;
		if(m_curveNet.arcs[a1].endNodesID.first==i)
			order1 = m_curveNet.arcs[a1].posInNode.first;
		else
			order1 = m_curveNet.arcs[a1].posInNode.second;
		if(m_curveNet.arcs[a2].endNodesID.first==i)
			order2 = m_curveNet.arcs[a2].posInNode.first;
		else
			order2 = m_curveNet.arcs[a2].posInNode.second;
		if(order1<order2){
			std::pair<int,int> temp; temp.first=order1; temp.second=order2;
			m_userDefinedPairsInNode[i].push_back(temp);
		}
		else{
			std::pair<int,int> temp; temp.first=order2; temp.second=order1;
			m_userDefinedPairsInNode[i].push_back(temp);
		}
		m_latestUpdateNodes.push_back(i);
	}
	for(int i=0;i<adjArcs.size();i++){
		if(adjArcs[i][0].size()!=1 || adjArcs[i][1].size()!=1)
			continue;
		if(!usedArcs[i])
			continue;
		int a1 = adjArcs[i][0][0];
		int a2 = adjArcs[i][1][0];
		int n[] = {m_curveNet.arcs[i].endNodesID.first,m_curveNet.arcs[i].endNodesID.second};
		int order1,order2;
		if(m_curveNet.arcs[a1].endNodesID.first==n[0])
			order1 = m_curveNet.arcs[a1].posInNode.first;
		else
			order1 = m_curveNet.arcs[a1].posInNode.second;
		if(m_curveNet.arcs[a2].endNodesID.first==n[1])
			order2 = m_curveNet.arcs[a2].posInNode.first;
		else
			order2 = m_curveNet.arcs[a2].posInNode.second;

		std::pair<int,int> temp; temp.first=order1; temp.second=order2;
		bool isExist=false;
		for(int j=0;j<m_userDefinedPairsInArc[i].size();j++){
			std::pair<int,int> pairArc = m_userDefinedPairsInArc[i][j];
			if(pairArc.first==temp.first && pairArc.second==temp.second)
				isExist=true;
		}
		if(!isExist){
			m_userDefinedPairsInArc[i].push_back(temp);
			m_latestUpdateArcs.push_back(i);
		}
	}
	return isRightCycle;
}

bool mySort2(std::pair<double,int> i,std::pair<double,int>  j) 
{ 
	return (i.first<j.first); 
}
void cycleUtils::chopCurves()
{
	double pointClosenessThreshold=m_pointCloseness;
	std::vector<std::vector<Point> > Curves=m_curveNetworkOriginal;

	std::vector<std::vector<std::pair<std::pair<int,int>,std::pair<int,int> > > > intersectionPoint(Curves.size());
	for(int i=0;i<Curves.size();i++){
		for(int j=0;j<Curves.size();j++){
			if(i==j)
				continue;
			for(int c1=0;c1<Curves[i].size();c1++){
				for(int c2 =0;c2<Curves[j].size();c2++){
					Point p1 = Curves[i][c1];
					Point p2 = Curves[j][c2];
					Point p = p1-p2;
					if(p.length()<=pointClosenessThreshold)
						intersectionPoint[i].push_back(std::pair<std::pair<int,int>,
							std::pair<int,int> >(std::pair<int,int>(i,c1),std::pair<int,int>(j,c2)));
				}
			}
			if(Curves[i].front()==Curves[i].back()){
				intersectionPoint[i].push_back(std::pair<std::pair<int,int>,
					std::pair<int,int> >(std::pair<int,int>(i,Curves[i].size()-1),std::pair<int,int>(0,0)));
				intersectionPoint[i].insert(intersectionPoint[i].begin(),std::pair<std::pair<int,int>,
					std::pair<int,int> >(std::pair<int,int>(i,0),std::pair<int,int>(0,0)));
			}
		}
	}
	std::vector<std::vector<int> > segInCurve(Curves.size());
	for(int i=0;i<intersectionPoint.size();i++){
		for(int j=0;j<intersectionPoint[i].size();j++){
			segInCurve[i].push_back(intersectionPoint[i][j].first.second);
		}
	}
	for(int i=0;i<segInCurve.size();i++){
		sort(segInCurve[i].begin(),segInCurve[i].end());
		std::vector<int>::iterator ip = segInCurve[i].begin();
		while(ip!=segInCurve[i].end()){
			int seg = *ip;
			ip++;
			if(ip==segInCurve[i].end())
				break;
			if(seg==*ip){
				ip--;
				segInCurve[i].erase(ip);
			}
		}
		ip = segInCurve[i].begin();
		while(ip!=segInCurve[i].end()){
			int seg = *ip;
			ip++;
			if(ip==segInCurve[i].end())
				break;
			Point p = Curves[i][seg] - Curves[i][*ip];
			double len=p.length();
			if(len<=(pointClosenessThreshold*2)){
				ip--;
				segInCurve[i].erase(ip);
			}
		}
		if(segInCurve[i][0]!=0)
			segInCurve[i][0]=0;
		if(segInCurve[i].back()!=(Curves[i].size()-1))
			segInCurve[i].back()=(Curves[i].size()-1);

	}

	std::vector< std::vector<Point> > myCurves;
	for(int i=0;i<segInCurve.size();i++){
		for(int j=0;j<segInCurve[i].size()-1;j++){
			std::vector<Point> temp(Curves[i].begin()+segInCurve[i][j],Curves[i].begin()+segInCurve[i][j+1]+1);
			if(temp.size()<=1)
				continue;
			myCurves.push_back(temp);
		}
	}
	m_curveNetworkOriginal = myCurves;	
}
void cycleUtils::fairCurves()
{
	bool isSmoothing=m_isSmothing;
	bool isDeleteDuplicateArc=m_isDeleteBranch;
	bool isDeleteBranch=m_isDeleteDuplicateArc;
	double pointClosenessThreshold=m_pointCloseness;
	double pointSnapThreshold=m_pointSnapThreshold;
	double curveClosenessThreshold=m_curveCloseness;
	std::vector<std::vector<Point> > Curves=m_curveNetworkOriginal;
	/*
		we have four steps to clean up the data, they are:
		1)fairing
		2)deleting duplicate curves
		3)deleting branch
		4)deleting intersection points only with two degrees
	*/

if(isSmoothing){
	//1.fairing the curve network; after this, the curves are smooth and every points in curves are distinguish.
	std::vector<std::vector<Point> > fairCurve = Curves;
	for(int times=0;times<3;times++){
		for(int i=0;i<Curves.size();i++){
			if(Curves[i].size()<3)continue;
			for(int j=1;j<Curves[i].size()-1;j++){
				Point point = Curves[i][j];
				Point leftPoint = Curves[i][j-1];
				Point rightPoint= Curves[i][j+1];
				Point centrePoint = (leftPoint+rightPoint)/2.0;
				point = (point+centrePoint)/2.0;
				fairCurve[i][j]=point;
			}
		}
		Curves=fairCurve;
	}
	fairCurve.clear();

	std::vector<std::vector<Point> > myCurve;
	for(int i=0;i<Curves.size();i++){
		Point p1 = Curves[i].front();
		Point p2 = Curves[i].back();
		if(p1==p2){
			Point leftPoint = p1-Curves[i][1];
			Point rightPoint= p2-Curves[i][Curves[i].size()-2];
			if(leftPoint!=rightPoint){
				continue;
			}
		}
		myCurve.push_back(Curves[i]);
	}
	Curves=myCurve;
}

#if 1
	//if the end nodes of two arcs are close w.r.t a predefined threshold, then set them equally.
	//2.if there are duplicate curves, then delete one;
	std::vector<std::pair<int ,int> > loop;
	for(int i=0;i<Curves.size()-1;i++){
		for(int j=i+1;j<Curves.size();j++){
			int isloop=0;//"=2" is true;
			Point pt1 =Curves[i].front(),pt2 =Curves[j].front();
			if((pt1-pt2).length()<=pointSnapThreshold){
				Curves[j].front()=Curves[i].front();
				isloop++;
			}
			pt1 = Curves[i].front();pt2=Curves[j].back();
			if((pt1-pt2).length()<=pointSnapThreshold){
				Curves[j].back()=Curves[i].front();
				isloop++;
			}
			pt1 = Curves[i].back();pt2=Curves[j].front();
			if((pt1-pt2).length()<=pointSnapThreshold){
				Curves[j].front()=Curves[i].back();
				isloop++;
			}
			pt1 = Curves[i].back();pt2=Curves[j].back();
			if((pt1-pt2).length()<=pointSnapThreshold){
				Curves[j].back()=Curves[i].back();
				isloop++;
			}
			if(isloop==2)
				loop.push_back(std::pair<int,int>(i,j));
		}
	}
#endif

if(isDeleteDuplicateArc){
	std::vector<std::vector<Point> > nonduplicateCurve = Curves;
	for(int i=0;i<loop.size();i++){
		int c1 = loop[i].first, c2 = loop[i].second;
		if(Curves[c1].front()==Curves[c1].back() || Curves[c2].front()==Curves[c2].back())
			continue; //means at least one of the curve is self-loop;
		//compute the largest distance of the shortest distance between two points within two curves;
		double maxmindistance = 0;
		for(int j=0;j<Curves[c1].size();j++){
			Point p1 = Curves[c1][j];
			double mindistance = FLT_MAX;
			for(int k=0;k<Curves[c2].size();k++){
				Point p2 = Curves[c2][k];
				mindistance = std::min(mindistance,(p1-p2).length());
			}
			maxmindistance = std::max(maxmindistance,mindistance);
		}
		if(maxmindistance<curveClosenessThreshold){
			if(nonduplicateCurve[c1].empty())
				nonduplicateCurve[c2].clear();
			else
				nonduplicateCurve[c1].clear();
		}
	}
	Curves.clear();
	for(int i=0;i<nonduplicateCurve.size();i++){
		if(!nonduplicateCurve[i].empty())
			Curves.push_back(nonduplicateCurve[i]);
	}
	nonduplicateCurve.clear();
}

if(isDeleteBranch){
	//3.if there is a branch, then delete it;
	int sizeCurve;
	do{
		sizeCurve =Curves.size();
		std::vector<std::pair<int ,int> > capacity;
		capacity.assign(Curves.size(),std::pair<int,int>(1,1));
		for(int i=0;i<Curves.size()-1;i++){
			for(int j=i+1;j<Curves.size();j++){
				if(Curves[i].front()==Curves[j].front()){
					capacity[i].first++;capacity[j].first++;
				}
				if(Curves[i].front()==Curves[j].back()){
					capacity[i].first++;capacity[j].second++;
				}
				if(Curves[i].back()==Curves[j].front()){
					capacity[i].second++;capacity[j].first++;
				}
				if(Curves[i].back()==Curves[j].back()){
					capacity[i].second++;capacity[j].second++;
				}
			}
		}
		std::vector<std::vector<Point> > nonBranchCurve;
		for(int i=0;i<capacity.size();i++){
			if(capacity[i].first<=1||capacity[i].second<=1)
				continue;
			nonBranchCurve.push_back(Curves[i]);
		}
		Curves = nonBranchCurve;
	}while(sizeCurve!=Curves.size());
}

if(isSmoothing){	
	std::vector<std::vector<Point> > fairCurve = Curves;
	for(int times=0;times<8;times++){
		for(int i=0;i<Curves.size();i++){
			if(Curves[i].size()<3)continue;
			for(int j=1;j<Curves[i].size()-1;j++){
				Point point = Curves[i][j];
				Point leftPoint = Curves[i][j-1];
				Point rightPoint= Curves[i][j+1];
				Point centrePoint = (leftPoint+rightPoint)/2.0;
				point = (point+centrePoint)/2.0;
				fairCurve[i][j]=point;
			}
		}
		Curves=fairCurve;
	}
	fairCurve.clear();
}

	std::vector<std::vector<Point> > myCurves;
	for(int i=0;i<Curves.size();i++){
		if(Curves[i].size()<=2){
			Point p= Curves[i].front()-Curves[i].back();
			if(p.length()<pointClosenessThreshold)
				continue;
		}
		if(Curves[i].front()==Curves[i].back()){
			double maxmindistance = 0;
			for(int j=0;j<Curves[i].size()-1;j++){
				Point p1 = Curves[i][j];
				double mindistance = FLT_MAX;
				for(int k=j+1;k<Curves[i].size();k++){
					Point p2 = Curves[i][k];
					mindistance = std::min(mindistance,(p1-p2).length());
				}
				maxmindistance = std::max(maxmindistance,mindistance);
			}
			if(maxmindistance<curveClosenessThreshold){
				continue;
			}
		}
		myCurves.push_back(Curves[i]);
	}
	m_curveNetworkOriginal = myCurves;
}
void cycleUtils::dataCleanUp()
{
	if(m_curveNetworkOriginal.empty())
		return;

	LinearCurveNet Curves = m_curveNetworkOriginal;

	if(m_isComputeIntersection)
		chopCurves();
	if(m_isSmothing||m_isDeleteBranch||m_isDeleteDuplicateArc)
		fairCurves();
/*
	constructNetwork();
	deleteNodeWithTwoDegree();
	constructNetwork();
*/
}
    
};