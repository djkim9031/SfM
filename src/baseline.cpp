#include "baseline.h"

Baseline::Baseline(View &view1, View &view2, Match &match){

    m_View1 = view1;
    m_View1.m_R = cv::Mat::eye(cv::Size(3,3), CV_64F);
    m_View2 = view2;
    m_Match = match;

}

std::pair<cv::Mat, cv::Mat> Baseline::get_pose(const cv::Mat &K){
    //computes and returns the rotation and translation components for the second view

    cv::Mat F = remove_outliers_using_F(m_View1, m_View2, m_Match);
    cv::Mat K_T;
    cv::transpose(K, K_T);
    cv::Mat E = K_T*F*K;

    printf("=================================================================================\n");
    printf("[Logging]\t\t Computed the Essential matrix. \n");
    printf("[Logging]\t\t Choosing the correct pose out of 4 solutions.\n");
    printf("=================================================================================\n");

    return check_pose(E, K);
}

std::pair<cv::Mat, cv::Mat> Baseline::check_pose(const cv::Mat &E,const cv::Mat &K){
    //retrieves the rotation and translation components from the essential matrix by decomposing it and verifying the validity of the 4 possible solutions
    
    cv::Mat R1, R2, t1, t2;
    get_camera_from_E(E, R1, R2, t1, t2);

    if(!check_determinant(R1)){
        get_camera_from_E(-E, R1, R2, t1, t2);
    }

    //solution 1
    std::pair<double, cv::Mat> results = triangulate(K, R1, t1);

    cv::Mat temp_mat; 
    cv::hconcat(R1, t1, temp_mat);
    if(results.first>100.0 || !check_triangulation(results.second, temp_mat)){

        //solution 2
        results = triangulate(K, R1, t2);
        cv::hconcat(R1, t2, temp_mat);
        if(results.first>100.0 || !check_triangulation(results.second, temp_mat)){

            //solution 3
            results = triangulate(K, R2, t1);
            cv::hconcat(R2, t1, temp_mat);
            if(results.first>100.0 || !check_triangulation(results.second, temp_mat)){

                //solution 4
                return {R2, t2};
            }

            return {R2, t1};
        }

        return {R1, t2};
    }

    return {R1, t1};
}

std::pair<double, cv::Mat> Baseline::triangulate(const cv::Mat &K, const cv::Mat &R, const cv::Mat &t){
    //triangulates points between the baseline views and calculates the mean reprojection error of the triangulation

    cv::Mat K_inv = K.inv();
    cv::Mat P1, P2;
    cv::hconcat(m_View1.m_R, m_View1.m_T, P1);
    cv::hconcat(R, t, P2);

    //reconstruct the points with filtered inliers
    std::pair<cv::Mat, cv::Mat> pts = get_keypoints_from_indices(m_View1.m_Keypoints, m_Match.m_Inliers1, m_View2.m_Keypoints, m_Match.m_Inliers2);

    //reprojection error calculation
    std::vector<double> reprojection_errors;

    cv::Mat points_3D;
    for(unsigned int i=0; i<pts.first.rows; ++i){
        double hom_pt1[] = {pts.first.row(i).at<double>(0), pts.first.row(i).at<double>(1), 1.0};
        double hom_pt2[] = {pts.second.row(i).at<double>(0), pts.second.row(i).at<double>(1), 1.0};  

        cv::Mat u1 = cv::Mat(3, 1, CV_64F, hom_pt1);
        cv::Mat u2 = cv::Mat(3, 1, CV_64F, hom_pt2);

        //convert homogenous 2D points to normalized device coordinates
        cv::Mat u1_normalized = K_inv*u1;
        cv::Mat u2_normalized = K_inv*u2;

        //calculate 3D points
        cv::Mat point_3D = get_3D_point(u1_normalized, P1, u2_normalized, P2);
        cv::Mat point_3D_T;
        cv::transpose(point_3D, point_3D_T);

        //calculate reprojection error
        double error = calculate_reprojection_error(point_3D, u2, K, R, t);
        reprojection_errors.push_back(error);

        points_3D.push_back(point_3D_T);  
    }

    double sum = 0.0;
    for(auto val : reprojection_errors){
        sum+=val;
    }

    return {sum/reprojection_errors.size(), points_3D};
}

Baseline::~Baseline(){

}