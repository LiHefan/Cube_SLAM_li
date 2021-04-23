#include<iostream>
#include<fstream>
#include<string>
#include<sstream>
#include<ctime>
#include<algorithm>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl_ros/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include "Object_landmark.h"
#include "Frame.h"
#include "g2o_Object.h"
#include "matrix_utils.h"

using namespace std;
using namespace Eigen;

typedef pcl::PointCloud<pcl::PointXYZRGB> CloudXYZRGB;

//global variable
string base_folder="/home/li/Cube_SLAM_li/object_slam_li/data/";
cv::Mat_<float> matx_to3d_, maty_to3d_;




int main(int argc,char* argv[]){
    //初始化信息
    cout<<"base_folder "<<base_folder<<endl;

//STEP【１】: 读取参数（离线模式,使用.txt）
    std::string pred_objs_txt=base_folder+"detect_cuboids_saved.txt";       //ground frame中的立方体位姿 //saved cuboids in local ground frame
    std::string init_camera_pose=base_folder+"pop_camera_poses_saved.txt";  //offline camera pose for cuboids detection(x y yaw=0, truth roll/pitch/height)
    std::string truth_camera_pose=base_folder+"truth_cam_poses.txt";        //相机真实的位姿

    Eigen::MatrixXd pred_frame_objects(100,10);
    Eigen::MatrixXd init_frame_poses(100,8);
    Eigen::MatrixXd truth_frame_poses(100,8);

    if(!read_all_number_txt(pred_objs_txt,pred_frame_objects))
        return -1;

    if(!read_all_number_txt(init_camera_pose,init_frame_poses))
        return -1;
    
    if(!read_all_number_txt(truth_camera_pose,truth_frame_poses))
        return -1;


    std::cout<<"read data size: "<<pred_frame_objects.rows()<<" "<<init_frame_poses.rows()<<" "<<truth_frame_poses.rows()<<std::endl;
  //STEP【２】: 传入参数开始增量式图优化
    incremental_build_graph(pred_frame_objects,init_frame_poses,truth_frame_poses);

}

void incremental_build_graph(Eigen::MatrixXd& offline_pred_frame_objects, Eigen::MatrixXd& init_frame_poses, Eigen::MatrixXd& truth_frame_poses){
    //STEP【１】变量定义
    //设置相机内参calib
    Eigen::Matrix3d calib;
    calib<<535.4,0,320.1,
            0,539.2,247.6,
            0,0,1;
    //帧数，即truth_frame_poses中的列数
    int total_frame_number=truth_frame_poses.rows();

    //STEP 【２】:graph optimization
    //in this example, there is only one object
    g2o::SparseOptimizer graph;
    g2o::BlockSolverX::LinearSolverType* linearSolver;
    linearSolver=new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX* solver_ptr=new g2o::BlockSolverX(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver=new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    graph.setAlgorithm(solver);
    graph.setVerbose(false);

    // only first truth pose is used, to diretly visually compare with truth pose, also provide good roll/pitch
    //构造一个四元数形式的fix_init_cam_pose_Twc读取第一帧位姿
    g2o::SE3Quat fix_init_cam_pose_Twc(truth_frame_poses.row(0).tail<7>());

    //保存每帧的优化结果
    std::vector<object_landmark*> cube_pose_opti_history(total_frame_number,nullptr); //lankmark pose after each frame's optimization
    std::vector<object_landmark*> cube_pose_raw_detected_history(total_frame_number,nullptr); //raw deteceted cuboid each frame, before optimization



}









