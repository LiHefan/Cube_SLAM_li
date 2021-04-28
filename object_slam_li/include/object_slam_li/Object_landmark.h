#pragma once

#include<vector>
#include"object_slam_li/g2o_Object.h"

//object_landmark类
//包含cuboid的信息

class object_landmark{
public:
    g2o::cuboid cube_meas; //cuboid9自由度信息
    g2o::VertexCuboid* cube_vertex; //g2o的顶点－物体测量
    double meas_quality; //评估提案测量的质量[0,1] the higher, the better
};