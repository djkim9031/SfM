#pragma once

#include "data.h"
#include "view.h"
#include "match.h"
#include <iostream>

#include <opencv4/opencv2/opencv.hpp>


inline std::pair<cv::Mat, cv::Mat> get_keypoints_from_indices(const std::vector<cv::KeyPoint> &keypoints1, const std::vector<int> &index_list1,
                                                              const std::vector<cv::KeyPoint> &keypoints2, const std::vector<int> &index_list2){
    //filters a list of keypoints based on the given indices
    
    cv::Mat points1, points2;
    for(auto idx : index_list1){
        double currPoint[] = {keypoints1[idx].pt.x, keypoints1[idx].pt.y};
        cv::Mat currMat = cv::Mat(1, 2, CV_64F, currPoint);
        points1.push_back(currMat);

    }
    for(auto idx : index_list2){
        double currPoint[] = {keypoints2[idx].pt.x, keypoints2[idx].pt.y};
        cv::Mat currMat = cv::Mat(1, 2, CV_64F, currPoint);
        points2.push_back(currMat);
    }

    return {points1,points2};
}

inline cv::Mat remove_outliers_using_F(View &view1, View &view2, Match &match){
    //remove outlier keypoints using the fundamental matrix

    std::pair<cv::Mat, cv::Mat> pts = get_keypoints_from_indices(view1.m_Keypoints, match.m_Indices1, view2.m_Keypoints, match.m_Indices2);
    cv::Mat mask;

    cv::Mat F = cv::findFundamentalMat(pts.first, pts.second, mask, cv::FM_RANSAC, 0.9, 0.99);

    for(int i=0;i<match.m_Indices1.size();++i){
        if((bool)mask.row(i).at<uchar>(0)){
            match.m_Inliers1.push_back(match.m_Indices1[i]);
            match.m_Inliers2.push_back(match.m_Indices2[i]);
        }
    }

    return F;

}

inline void get_camera_from_E(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t1, cv::Mat &t2){
    //calculates rotation and translation components from the essential matrix

    double W_vals[] = {0, -1, 0,
                      1, 0, 0,
                      0, 0, 1};
    cv::Mat W = cv::Mat(3,3,CV_64F, W_vals);
    cv::Mat W_T;
    cv::transpose(W, W_T);
    cv::SVD svd(E);

    R1 = (svd.u) * W * (svd.vt);
    R2 = (svd.u) * W_T * (svd.vt);
    t1 = (svd.u).col(svd.u.cols-1);
    t2 = -t1;

    return;
}

inline bool check_determinant(const cv::Mat &R){
    //validates using the determinant of the rotation matrix

    if (cv::determinant(R)+1<1e-9){
        return false;
    }

    return true;
}

inline cv::Mat get_3D_point(const cv::Mat &u1, const cv::Mat &P1, const cv::Mat &u2, const cv::Mat &P2){
    //solves for 3D point using homogenous 2D points and the respective camera matrices

    double A_data[] = {
        u1.at<double>(0,0)*P1.at<double>(2,0) - P1.at<double>(0,0), u1.at<double>(0,0)*P1.at<double>(2,1) - P1.at<double>(0,1), u1.at<double>(0,0)*P1.at<double>(2,2) - P1.at<double>(0,2),
        u1.at<double>(1,0)*P1.at<double>(2,0) - P1.at<double>(1,0), u1.at<double>(1,0)*P1.at<double>(2,1) - P1.at<double>(1,1), u1.at<double>(1,0)*P1.at<double>(2,2) - P1.at<double>(1,2),
        u2.at<double>(0,0)*P2.at<double>(2,0) - P2.at<double>(0,0), u2.at<double>(0,0)*P2.at<double>(2,1) - P2.at<double>(0,1), u2.at<double>(0,0)*P2.at<double>(2,2) - P2.at<double>(0,2),
        u2.at<double>(1,0)*P2.at<double>(2,0) - P2.at<double>(1,0), u2.at<double>(1,0)*P2.at<double>(2,1) - P2.at<double>(1,1), u2.at<double>(1,0)*P2.at<double>(2,2) - P2.at<double>(1,2)
    };

    double B_data[] ={
        -(u1.at<double>(0,0)*P1.at<double>(2,3) - P1.at<double>(0,3)),
        -(u1.at<double>(1,0)*P1.at<double>(2,3) - P1.at<double>(1,3)),
        -(u2.at<double>(0,0)*P2.at<double>(2,3) - P2.at<double>(0,3)),
        -(u2.at<double>(1,0)*P2.at<double>(2,3) - P2.at<double>(1,3))
    };

    cv::Mat A = cv::Mat(4, 3, CV_64F, A_data);
    cv::Mat B = cv::Mat(4, 1, CV_64F, B_data);
    
    cv::Mat X;
    cv::solve(A, B, X, cv::DECOMP_SVD);

    return X;
}

inline double calculate_reprojection_error(const cv::Mat &point_3D, const cv::Mat &point_2D, const cv::Mat &K, const cv::Mat &R, const cv::Mat &t){
    //calculates the reprojection error for a 3D point by projecting it back into the image plane

    cv::Mat reprojected_error = K*(R*point_3D + t);
    cv::Mat reprojected_error_T; 
    cv::transpose(reprojected_error, reprojected_error_T);
    cv::convertPointsFromHomogeneous(reprojected_error_T, reprojected_error);

    double pts_2D[] = { point_2D.at<double>(0,0), point_2D.at<double>(1,0)};
    double reproj_data[] = {reprojected_error.at<double>(0,0), reprojected_error.at<double>(0,1)};
    reprojected_error_T = cv::Mat(2, 1, CV_64F, reproj_data);
    cv::Mat temp_2D = cv::Mat(2, 1, CV_64F, pts_2D);

    return cv::norm((temp_2D - reprojected_error_T));
}

inline bool check_triangulation(const cv::Mat &points, cv::Mat &P){
    //check whether reconstructed points lie in front of the camera

    double vals[] ={0.0, 0.0, 0.0, 1.0};
    cv::Mat temp = cv::Mat(1, 4, CV_64F, vals);
    cv::vconcat(P, temp, P);

    //A bit of a c++ opencv trick
    //input points data is of the size (row, col) = (numData, 3)
    //However, this data has to be modified as 3-channel 3 dim matrix to operate with prospectiveTransform
    //so, convert it to (row, col, channel) = (1, numData, 3)
    cv::Mat mat_3d = cv::Mat(1, points.rows, CV_64FC3, (double*)points.data);

    cv::Mat reprojected_points;
    cv::perspectiveTransform(mat_3d, reprojected_points, P);
    
    std::vector<cv::Mat> holders(3); //we are only interested in the 3rd value of each data point
    cv::split(reprojected_points, holders);
    double sum = 0.0;
    for(unsigned int i=0; i<holders[2].cols; ++i){
        if(holders[2].at<double>(0,i)>0){
            sum+=1;
        }
    }

    if(sum/holders[2].cols < 0.75){
        return false;
    }

    return true;
}