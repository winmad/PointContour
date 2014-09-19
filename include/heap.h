#ifndef HEAP_H
#define HEAP_H

#include "nvVector.h"
#include <vector>

class Heap
{
public:
	struct HeapNode
	{
		int index;
		double key;

		HeapNode() {}
		HeapNode(const int& _index , const double& _key) :
			index(_index) , key(_key) {}

		bool operator <(const HeapNode& rhs)
		{
			return key < rhs.key;
		}
	};

	Heap(const int& maxNodeNum) 
	{
		indices.resize(maxNodeNum + 1);
		tree.resize(maxNodeNum + 1);

		for (int i = 0; i < indices.size(); i++)
			indices[i] = -1;

		nodeNum = 0;
	}

	void push(int index , double key)
	{
		//if (indices[index] <= 0 || indices[index] > nodeNum)
        if (indices[index] == -1)
		{
			nodeNum++;
			tree[nodeNum].index = index;
			indices[index] = nodeNum;
			tree[nodeNum].key = key;

			adjustUp(nodeNum);
		}
		else
		{
			int t = indices[index];
			tree[t].key = key;

			adjustUp(t);
		}
	}

	void pop()
	{
		swapNode(1 , nodeNum);
		indices[tree[nodeNum].index] = -1;
		nodeNum--;

		adjustDown(1);
	}

	HeapNode top()
	{
		return tree[1];
	}

	bool empty()
	{
		return nodeNum <= 0;
	}

public:
	int nodeNum;
	std::vector<int> indices;
	std::vector<HeapNode> tree;

	void swapNode(int x , int y)
	{
		indices[tree[x].index] = y;
		indices[tree[y].index] = x;
		std::swap(tree[x] , tree[y]);
	}

	void adjustDown(int root)
	{
		int t = (root << 1);
		while (t <= nodeNum)
		{
			if (t < nodeNum && tree[t + 1] < tree[t])
				t++;
			if (tree[t] < tree[root])
			{
				swapNode(root , t);
				root = t;
				t <<= 1;
			}
			else
			{
				t = nodeNum + 1;
			}
		}
	}

	void adjustUp(int root)
	{
		while (root > 1)
		{
			int pa = (root >> 1);
			if (tree[root] < tree[pa])
			{
				swapNode(pa , root);
				root = pa;
			}
			else
			{
				root = 1;
			}
		}
	}
};

#endif