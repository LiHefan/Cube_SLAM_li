#pragma once

#include<vector>
#include"g2o_Object.h"

//物体路标类
//包含立方体的信息

class object_landmark{
public:
    g2o::cuboid cube_meas; //立方体路标的9自由度信息
    g2o::VertexCuboid* cube_vertex; //g2o的顶点－物体测量
    double meas_quality; //评估提案测量的质量[0,1] the higher, the better
};