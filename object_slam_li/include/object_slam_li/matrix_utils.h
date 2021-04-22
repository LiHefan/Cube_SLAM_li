#pragma once

//Eigen
#include<Eigen/Core>
#include<Eigen/Dense>

template <class T>
Eigen::Quaternion<T> zyx_euler_to_quat(const T&roll, const T&pitch,const T&yaw); //欧拉角转化为四元数

template <class T>
void quat_to_euler_zyx(const Eigen::Quaternion<T>& q, T& roll, T& pitch, T& yaw); //四元数转成欧拉角

template <class T>
Eigen::Matrix<T, Eigen::Dynamic,Eigen::Dynamic> real_to_homo_coord(const Eigen::Matrix<T, Eigen::Dynamic,Eigen::Dynamic>& pts_in);
template <class T>
void real_to_homo_coord(const Eigen::Matrix<T, Eigen::Dynamic,Eigen::Dynamic>& pts_in, Eigen::Matrix<T, Eigen::Dynamic,Eigen::Dynamic>& pts_homo_out);

template <class T>
Eigen::Matrix<T, Eigen::Dynamic,Eigen::Dynamic> homo_to_real_coord(const Eigen::Matrix<T, Eigen::Dynamic,Eigen::Dynamic>& pts_homo_in);
template <class T>
void homo_to_real_coord(const Eigen::Matrix<T, Eigen::Dynamic,Eigen::Dynamic>& pts_in, Eigen::Matrix<T, Eigen::Dynamic,Eigen::Dynamic>& pts_homo_out);