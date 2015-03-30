#include "axisWidget.h"

void AxisWidget::init()
{
    origin = vec3d(0.0);
    length = 0.2;
    radius = 0.2;

    axes.clear();
    axes.push_back(vec3d(1.0 , 0.0 , 0.0));
    axes.push_back(vec3d(0.0 , 1.0 , 0.0));
    axes.push_back(vec3d(0.0 , 0.0 , 1.0));

    tableSize = 90;
    cosTable.resize(tableSize);
    sinTable.resize(tableSize);
    for (int i = 0; i < tableSize; i++)
    {
        cosTable[i] = cos(2.0 * double(i) * PI / (double)tableSize);
        sinTable[i] = sin(2.0 * double(i) * PI / (double)tableSize);
    }

    resamplePoints();
}

void AxisWidget::resamplePoints()
{
    axesPoints.resize(axes.size());
    for (int i = 0; i < axes.size(); i++)
    {
        resampleLine(origin - axes[i] * 0.5 , origin + axes[i] * 0.5 , 200 , axesPoints[i]);
    }

    assert(axes.size() == 3);
    globePoints.resize(axes.size());
    for (int i = 0; i < axes.size(); i++)
    {
        int a = (i + 1) % 3;
        int b = (i + 2) % 3;
        globePoints[i].clear();
        for (int j = 0; j < tableSize; j++)
        {
            vec3d dir = (axes[a] * cosTable[j] + axes[b] * sinTable[j]);
            globePoints[i].push_back(origin + dir * radius);
        }
    }
    /*
    for (int i = 0; i < axesPoints.size(); i++)
    {
        printf("=======\n");
        for (int j = 0; j < axesPoints[i].size(); j++)
        {
            printf("%.6f %.6f %.6f\n" , axesPoints[i][j].x , axesPoints[i][j].y ,
                axesPoints[i][j].z);
        }
    }
    for (int i = 0; i < globePoints.size(); i++)
    {
        printf("=======\n");
        for (int j = 0; j < globePoints[i].size(); j++)
        {
            printf("%.6f %.6f %.6f\n" , globePoints[i][j].x , globePoints[i][j].y ,
                globePoints[i][j].z);
        }
    }
    */
}