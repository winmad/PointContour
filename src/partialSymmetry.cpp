#define _WCHAR_H_CPLUSPLUS_98_CONFORMANCE_
#include "partialSymmetry.h"
#include "pointCloudUtils.h"

PartialSymmetry::PartialSymmetry()
{
    pcUtils = NULL;
}

void PartialSymmetry::init(PointCloudUtils *_pcUtils)
{
    pcUtils = _pcUtils;
    std::vector<Data>& data = pcUtils->pcData;
    std::vector<Data> signData;
    for (int i = 0; i < data.size(); i++)
    {

    }
    // use Data, CloesPoint, and KnnQuery in pointCloudUtils.h
    // pos: position in signature space, n: position in original space
    PointKDTree<Data> signSpaceTree(signData);
}