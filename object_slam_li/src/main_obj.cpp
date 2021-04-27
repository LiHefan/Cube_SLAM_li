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
    std::vector<shared_ptr<object_landmark>> cube_pose_opti_history(total_frame_number,nullptr); //lankmark pose after each frame's optimization
    std::vector<shared_ptr<object_landmark>> cube_pose_raw_detected_history(total_frame_number,nullptr); //raw deteceted cuboid each frame, before optimization

    int offline_cube_obs_row_id=0;

    //all_frames 存储每一帧的信息
    std::vector<shared_ptr<tracking_frame>> all_frames(total_frame_number);
    g2o::VertexCuboid* vCube;

    //处理每一帧图像
    for(int frame_index=0;frame_index<total_frame_number;++frame_index){
        g2o::SE3Quat curr_cam_pose_Twc;     //当前帧的位姿
        g2o::SE3Quat odom_val;              //从上一帧到当前帧的旋转

        //STEP１：计算每一帧位姿
        if(frame_index==0)
            curr_cam_pose_Twc=fix_init_cam_pose_Twc;
        else{
            g2o::SE3Quat prev_pose_Tcw=all_frames[frame_index-1]->cam_pose_Tcw;
            if(frame_index>1){      // frome third frame, use the constant motion model to initialize camera　//second frame=first???
                g2o::SE3Quat prev_prev_pose_Tcw=all_frames[frame_index-2]->cam_pose_Tcw;
                //从上一帧到本帧的变换
                odom_val=prev_pose_Tcw*prev_prev_pose_Tcw.inverse();  //why not use prev_prev_pose_Twc???
            }
            //计算当前帧位姿
            curr_cam_pose_Twc=(odom_val*prev_pose_Tcw).inverse();

        }

        //STEP２：信息存储
        shared_ptr<tracking_frame> currframe=make_shared<tracking_frame>(tracking_frame());
        currframe->frame_seq_id=frame_index;
        all_frames[frame_index]=currframe;

        bool has_detected_cuboid=false;
        g2o::cuboid cube_local_meas;
        double proposal_error;
        //将frame_index格式化为4位,用于读图
        char frame_index_c[256];
        sprintf(frame_index_c,"%04d",frame_index);

        //STEP３：读取离线数据
        //离线模式已知立方体位姿矩阵　offline_pred_frame_objects(detect_cuboids_saved.txt)
        int cube_obs_frame_id=offline_pred_frame_objects(offline_cube_obs_row_id,0);//读取第一列编号
        has_detected_cuboid=(cube_obs_frame_id==frame_index);
        if(has_detected_cuboid){    //prepare object measurement    not all frame has observation
            //读取立方体9自由度位姿
            Eigen::VectorXd measure_data=offline_pred_frame_objects.row(offline_cube_obs_row_id);
            g2o::cuboid cube_ground_value;
            Vector9d cube_pose;
            cube_pose<<measure_data(1),measure_data(2),measure_data(3),
                        0,0,measure_data(4),
                        measure_data(5),measure_data(6),measure_data(7);    //xyz rpy scale
            cube_ground_value.fromMinimalVector(cube_pose);

            //读取相机位姿
            Eigen::VectorXd cam_pose_vec=init_frame_poses.row(frame_index);
            g2o::SE3Quat cam_val_Twc(cam_pose_vec.segment<7>(1));   //time x y z qx qy qz qw
            cube_local_meas=cube_ground_value.transform_to(cam_val_Twc); //measurement in local camera frame

            //立方体提案的误差
            proposal_error=measure_data(8);

            //读取离线保存的带有提案的2D图像，并保存到currframe->cuboids_2d_img中
            std::string detected_cube_2d_img_name=base_folder+"pred_3d_obj_overview/"+std::string(frame_index_c)+"_best_objects.jpg";
            currframe->cuboids_2d_img=cv::imread(detected_cube_2d_img_name,1);
            ++offline_cube_obs_row_id;

        }

        //STEP４:立方体路标
        if(has_detected_cuboid){
            shared_ptr<object_landmark> localcuboid=make_shared<object_landmark>(object_landmark());
            localcuboid->cube_meas=cube_local_meas;
            localcuboid->meas_quality=(1-proposal_error+0.5)/2; //higher,better
            currframe->observed_cuboids.push_back(localcuboid);
        }
        //STEP5: G2O优化
        //set up g2o cube vertex, only one in this dataset
        if(frame_index==0){
            g2o::cuboid init_cuboid_global_pose=cube_local_meas.transform_from(curr_cam_pose_Twc);
            vCube=new g2o::VertexCuboid();
            vCube->setEstimate(init_cuboid_global_pose);
            vCube->setId(0);
            vCube->setFixed(false);
            graph.addVertex(vCube);
        }

        //set up g2o camera vertex
        g2o::VertexSE3Expmap* vSE3=new g2o::VertexSE3Expmap();
        currframe->pose_vertex=vSE3;
        vSE3->setId(frame_index+1);
        graph.addVertex(vSE3);
        vSE3->setEstimate(curr_cam_pose_Twc.inverse());
        vSE3->setFixed(frame_index==0);

        //add g2o camera-object measurement edges
        if(currframe->observed_cuboids.size()>0){
            shared_ptr<object_landmark> cube_landmark_meas=all_frames[frame_index]->observed_cuboids[0];

            g2o::EdgeSE3Cuboid* e=new g2o::EdgeSE3Cuboid();
            e->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(vSE3));
            e->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(vCube));
            e->setMeasurement(cube_landmark_meas->cube_meas);
            e->setId(frame_index);

            //set information matrix
            Vector9d inv_sigma;
            inv_sigma<<1,1,1,1,1,1,1,1,1;
            inv_sigma=inv_sigma*2.0*cube_landmark_meas->meas_quality;
            Matrix9d info=inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
            e->setInformation(info);
            graph.addEdge(e);

        }

        //camera vertex, add cam-cam odomerty edges
        if(frame_index>0){
            g2o::EdgeSE3Expmap* e=new g2o::EdgeSE3Expmap();
            e->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(all_frames[frame_index-1]->pose_vertex));
            e->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(all_frames[frame_index]->pose_vertex));
            e->setMeasurement(odom_val);

            e->setId(total_frame_number+frame_index);
            Vector6d inv_sigma;
            inv_sigma<<1,1,1,1,1,1;
            inv_sigma=inv_sigma*1.0;
            Matrix6d info=inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
            e->setInformation(info);
            graph.addEdge(e);
        }

    }

    



}









