#include "axisWidget.h"

void AxisWidget::init()
{
    origin = vec3d(0.0);
    length = 0.2;

    axes.push_back(vec3d(1.0 , 0.0 , 0.0));
    axes.push_back(vec3d(0.0 , 1.0 , 0.0));
    axes.push_back(vec3d(0.0 , 0.0 , 1.0));

    axesPoints.resize(axes.size());
    for (int i = 0; i < axes.size(); i++)
    {
        resampleLine(origin , origin + axes[i] * 0.2 , 50 , axesPoints[i]);
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
    */
}