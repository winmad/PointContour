#define _WCHAR_H_CPLUSPLUS_98_CONFORMANCE_
#include "partialSymmetry.h"
#include "pointCloudUtils.h"
#include "RandGenerator.h"
#include <string>

PartialSymmetry::PartialSymmetry()
{
    pcUtils = NULL;
    signSpaceTree = NULL;
}

void PartialSymmetry::init(PointCloudUtils *_pcUtils)
{
    pcUtils = _pcUtils;
    isSampled.resize(pcUtils->pcData.size());
}

void PartialSymmetry::samplePointsUniform(int numSamples)
{
    for (int i = 0; i < isSampled.size(); i++) isSampled[i] = false;
    numSamples = std::min(numSamples , (int)pcUtils->pcData.size());
    for (int i = 0; i < numSamples; i++)
    {
        for (;;)
        {
            int j = (int)(RandGenerator::genFloat() * isSampled.size());
            if (!isSampled[j])
            {
                isSampled[j] = true;
                break;
            }
        }
    }

    signData.clear();
    for (int i = 0; i < pcUtils->pcData.size(); i++)
    {
        if (!isSampled[i]) continue;
        Tensor ts;
        ts.hessian = pcUtils->lerpHessian(pcUtils->pcData[i].pos);
        pcUtils->calcTensorDecomposition(ts);
        Data sign;
        for (int j = 0; j < 3; j++)
            sign.pos[j] = ts.eigenVal[j];
        sign.n = pcUtils->pcData[i].pos;
        sign.index = i;
        signData.push_back(sign);
    }
}

void PartialSymmetry::calcVotes()
{
    signSpaceTree = new PointKDTree<Data>(signData);

    int totVotes = 500000;
    // int knn = totVotes / signData.size();
    int knn = 15;
    votes.clear();

    // for (int i = 0; i < isSampled.size(); i++) isSampled[i] = false;
    for (int i = 0; i < signData.size(); i++) for (int j = i + 1; j < signData.size(); j++)
    {
        if ((signData[i].pos - signData[j].pos).length() > 1e-3) continue;
        /*
        if (i == 0)
        {
            isSampled[signData[i].index] = true;
            isSampled[signData[j].index] = true;
            printf("i: (%.6f,%.6f,%.6f), (%.6f,%.6f,%.6f)\n" ,
                signData[i].pos.x , signData[i].pos.y , signData[i].pos.z ,
                signData[i].n.x , signData[i].n.y , signData[i].n.z);
            printf("j: (%.6f,%.6f,%.6f), (%.6f,%.6f,%.6f)\n\n" ,
                signData[j].pos.x , signData[j].pos.y , signData[j].pos.z ,
                signData[j].n.x , signData[j].n.y , signData[j].n.z);
        }
        */
        vec3d normal = signData[i].n - signData[j].n;
        normal.normalize();
        if (normal.y < 0) normal = -normal;
        Plane plane((signData[i].n + signData[j].n) * 0.5 , normal);
        votes.push_back(plane);
        /*
        //int i = (int)(RandGenerator::genFloat() * signData.size());
        KnnQuery query(knn);
        signSpaceTree->searchKnn(0 , signData[i].pos , query);
        // writeLog("===== pos: (%.6f,%.6f,%.6f), sign: (%.6f,%.6f,%.6f) =====\n" ,
            // signData[i].n.x , signData[i].n.y , signData[i].n.z ,
            // signData[i].pos.x , signData[i].pos.y , signData[i].pos.z);
        for (int j = 0; j < query.knnPoints.size(); j++)
        {
            const Data* nb = query.knnPoints[j].point;
            // writeLog("pos: (%.6f,%.6f,%.6f), sign: (%.6f,%.6f,%.6f)\n" ,
                // nb->n.x , nb->n.y , nb->n.z , nb->pos.x , nb->pos.y , nb->pos.z);
            if (isEqual(signData[i].n , nb->n)) continue;
            vec3d normal = signData[i].n - nb->n;
            normal.normalize();
            if (normal.y < 0) normal = -normal;
            Plane plane((signData[i].n + nb->n) * 0.5 , normal);
            votes.push_back(plane);
        }
        */
    }
    delete signSpaceTree;
    
    std::string fileName = pcUtils->dataVotesPath + pcUtils->name + ".votes";
    FILE *fp = fopen(fileName.c_str() , "w");
    fprintf(fp , "%lu\n" , votes.size());
    for (int i = 0; i < votes.size(); i++)
    {
        double theta , phi;
        votes[i].calcAngle(theta , phi);
        fprintf(fp , "%.6f %.6f %.6f\n" , theta , phi , votes[i].d);
    }
    fclose(fp);

    /*
    fp = fopen("distances.txt" , "w");
    fprintf(fp , "%lu\n" , votes.size() * (votes.size() - 1));
    for (int i = 0; i < votes.size(); i++)
    {
        vec3d u;
        votes[i].calcAngle(u.x , u.y);
        u.z = votes[i].d;
        for (int j = i + 1; j < votes.size(); j++)
        {
            vec3d v;
            votes[j].calcAngle(v.x , v.y);
            v.z = votes[j].d;
            fprintf(fp , "%d %d %.6f\n" , i , j , (u - v).length());
        }
    }
    fclose(fp);
    */
}
