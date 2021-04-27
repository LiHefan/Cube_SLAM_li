#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include "g2o_Object.h"

class object_landmark;

//跟踪图像帧的类
class tracking_frame{
public:
    int frame_seq_id; //image topic sequence id, fixed;
    cv::Mat frame_img;  //原图
    cv::Mat cuboids_2d_img; //带有立方体提案的2D图像
    
    //g2o优化的顶点
    g2o::VertexSE3Expmap* pose_vertex;

    //从这一帧中生成的立方体，可能不是SLAM的路标
    std::vector<shared_ptr<object_landmark>> observed_cuboids; //generated cuboid from this frame, maynot be actual SLAM landmark

    //优化后的位姿
    g2o::SE3Quat cam_pose_Tcw;  //optimized pose world to cam
    g2o::SE3Quat cam_pose_Twc;  //optimized pose cam to world

}；