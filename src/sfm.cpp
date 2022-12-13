#include "sfm.h"

SFM::SFM(std::vector<View> &views, std::vector<Match> &matches, const cv::Mat &K){

    m_Views = views;
    m_Matches = matches;

    m_K = K;
    m_PointCounter = 0;

    for(auto view : m_Views){
        m_Names.push_back(view.m_Name);
    }

    m_Done.clear();

}

void SFM::remove_mapped_points(Match &match, int image_idx){
    //removes points that have already been reconstructed in the completed views

    std::vector<int> m_Inliers1, m_Inliers2;
    for(unsigned int i=0; i<match.m_Inliers1.size(); ++i){
        std::pair<int, int> key = {image_idx, match.m_Inliers1[i]};
        auto it = m_PointMap.find(key);
        if(it==m_PointMap.end()){
            m_Inliers1.push_back(match.m_Inliers1[i]);
            m_Inliers2.push_back(match.m_Inliers2[i]);
        }
    }

    match.m_Inliers1 = m_Inliers1;
    match.m_Inliers2 = m_Inliers2;
}

void SFM::compute_pose(View &view1, View &view2, bool isBaseline){
    //computes the pose of the new view

    if(isBaseline){
        //procedure for baseline pose estimation

        Match match;
        std::string currMatchName1 = view1.m_Name + "_" + view2.m_Name;
        std::string currMatchName2 = view2.m_Name + "_" + view1.m_Name;
        bool found = false;
        for(auto m : m_Matches){
            if(m.m_MatchName == currMatchName1 || m.m_MatchName == currMatchName2){
                match = m;
                found = true;
                break;
            }
        }
        if(!found){
            fprintf(stderr, "[ERROR]\t\t Couldn't find the match object between %s and %s\n", view1.m_Name.c_str(), view2.m_Name.c_str());
            exit(0);
        }

        Baseline baseline_pose(view1, view2, match);
        std::pair<cv::Mat, cv::Mat> pose = baseline_pose.get_pose(m_K);
        view2.m_R = pose.first; view2.m_T = pose.second;

        std::vector<double> rpe1, rpe2;
        triangulate(view1, view2, baseline_pose.m_Match, rpe1, rpe2);
        assert(rpe1.size() == rpe2.size());

        double rpe1_sum = 0;
        double rpe2_sum = 0;
        for(unsigned int i=0;i<rpe1.size();++i){
            rpe1_sum += rpe1[i];
            rpe2_sum += rpe2[i];
        }

        m_Errors.push_back(rpe1_sum/rpe1.size());
        m_Errors.push_back(rpe2_sum/rpe2.size());

        m_Done.push_back(view1);
        m_Done.push_back(view2);
    } else{
        //procedure for estimating the pose of all other views
        compute_pose_PNP(view1, view1.m_R, view1.m_T);
        std::vector<double> errors;

        //reconstruct the unreconstructed points from all of the previous views
        for(unsigned int i=0;i<m_Done.size();++i){
            Match match;
            std::string currMatchName1 = view1.m_Name + "_" + m_Done[i].m_Name;
            std::string currMatchName2 = m_Done[i].m_Name + "_" + view1.m_Name;
            bool found = false;
            for(auto m : m_Matches){
                if(m.m_MatchName == currMatchName1 || m.m_MatchName == currMatchName2){
                    match = m;
                    found = true;
                    break;
                }
            }
            if(!found){
                fprintf(stderr, "[ERROR]\t\t Couldn't find the match object between %s and old view %s\n", view1.m_Name.c_str(), m_Done[i].m_Name.c_str());
                exit(0);
            }

            remove_outliers_using_F(m_Done[i], view1, match);
            remove_mapped_points(match, i);
            std::vector<double> rpe1, rpe_;
            triangulate(m_Done[i], view1, match, rpe_, rpe1);
            errors.insert(errors.end(), rpe1.begin(), rpe1.end());
        }

        double errors_sum = 0.0;
        for(unsigned int i=0; i<errors.size(); ++i){
            errors_sum += errors[i];
        }

        m_Errors.push_back(errors_sum/errors.size());
        m_Done.push_back(view1);
    }
}

void SFM::triangulate(const View &view1, const View &view2, const Match &match, std::vector<double> &rpe1, std::vector<double> &rpe2){
    //triangulates 3D points from two views whose poses have been recovered.
    //Also updates the point map

    cv::Mat K_inv = m_K.inv();
    cv::Mat P1, P2;
    cv::hconcat(view1.m_R, view1.m_T, P1);
    cv::hconcat(view2.m_R, view2.m_T, P2);

    std::pair<cv::Mat, cv::Mat> pts = get_keypoints_from_indices(view1.m_Keypoints, match.m_Inliers1, view2.m_Keypoints, match.m_Inliers2);
    if(pts.first.rows<1){
        rpe1.push_back(0.0);
        rpe2.push_back(0.0);
        return;
    }

    int k1_idx = get_index_of_view(view1);
    int k2_idx = get_index_of_view(view2);

    for(unsigned int i=0; i<pts.first.rows; ++i){
        double hom_pt1[] = {pts.first.row(i).at<double>(0), pts.first.row(i).at<double>(1), 1.0};
        double hom_pt2[] = {pts.second.row(i).at<double>(0), pts.second.row(i).at<double>(1), 1.0};  

        cv::Mat u1 = cv::Mat(3, 1, CV_64F, hom_pt1);
        cv::Mat u2 = cv::Mat(3, 1, CV_64F, hom_pt2);

        cv::Mat u1_normalized = K_inv*u1;
        cv::Mat u2_normalized = K_inv*u2;

        cv::Mat point_3D = get_3D_point(u1_normalized, P1, u2_normalized, P2);
        cv::Mat point_3D_T;
        cv::transpose(point_3D, point_3D_T);
        m_Points3D.push_back(point_3D_T);

        double error1 = calculate_reprojection_error(point_3D, u1, m_K, view1.m_R, view1.m_T);
        rpe1.push_back(error1);
        double error2 = calculate_reprojection_error(point_3D, u2, m_K, view2.m_R, view2.m_T);
        rpe2.push_back(error2);

        //update point map with the key (idx of view, idx of point in the view) and value point_counter
        //multiple keys can have the same value because a 3D point is reconstructed using 2 points
        std::pair<int, int> key1 = {k1_idx, match.m_Inliers1[i]};
        std::pair<int, int> key2 = {k2_idx, match.m_Inliers2[i]};
        m_PointMap.emplace(key1, m_PointCounter);
        m_PointMap.emplace(key2, m_PointCounter);
        m_PointCounter++;       
    }

}

void SFM::compute_pose_PNP(const View &view, cv::Mat &R, cv::Mat &t){
    //computes the pose of new view using perspective n-points

    cv::BFMatcher matcher;
    if(view.m_FeatureType == "sift" || view.m_FeatureType == "surf"){
        matcher.create(cv::NORM_L2, false);
    } else{
        //orb
        matcher.create(cv::NORM_HAMMING, false);
    }

    //collects all the descriptors of the reconstructed views
    std::vector<cv::Mat> old_descriptors;
    for(auto d : m_Done){
        old_descriptors.push_back(d.m_Descriptor);
    }

    //match old descriptors against the descriptors in the new view
    matcher.add(old_descriptors);
    matcher.train();
    std::vector<cv::DMatch> matches;
    matcher.match(view.m_Descriptor, matches);

    cv::Mat points_3D, points_2D;
    unsigned int new_cal_count = 0;
    //build corresponding array of 2D points and 3D points
    for(auto match : matches){
        int old_image_idx = match.imgIdx;
        int new_image_kp_idx = match.queryIdx;
        int old_image_kp_idx = match.trainIdx;

        std::pair<int, int> key = {old_image_idx, old_image_kp_idx};
        auto it = m_PointMap.find(key);
        if(it!=m_PointMap.end()){
            //obtain the 2D point from match
            double pt_data_2d[] = {view.m_Keypoints[new_image_kp_idx].pt.x, view.m_Keypoints[new_image_kp_idx].pt.y};
            cv::Mat point_2D = cv::Mat(1, 2, CV_64F, pt_data_2d);
            points_2D.push_back(point_2D);

            //obtain the 3D point from the point map
            double pt_data_3d[] = {m_Points3D.at<double>(it->second, 0), m_Points3D.at<double>(it->second, 1), m_Points3D.at<double>(it->second, 2)};
            cv::Mat point_3D = cv::Mat(1, 3, CV_64F, pt_data_3d);
            points_3D.push_back(point_3D);

            new_cal_count++;
        }
    }

    //compute a new pose using solvePnPRansac
    if(new_cal_count>=4){ //cv::solvePnPRansac expects to take 4 or more points
        cv::Mat dontCare;
        cv::solvePnPRansac(points_3D, points_2D, m_K, cv::Mat(), R, t, false, 100, 8.0, 0.99, dontCare, cv::SOLVEPNP_DLS);
        cv::Rodrigues(R, R);
    }

    return;
}

void SFM::reconstruct(){
    //starts the main reconstruction loop for a given set of views and matches

    //compute baseline pose
    View baseline_view1 = m_Views[0];
    View baseline_view2 = m_Views[1];
    printf("[Logging]\t\t Computing the baseline pose and reconstructing points... \n");
    compute_pose(baseline_view1, baseline_view2, true);
    printf("[Logging]\t\t Mean reprojection error for the 1st baseline image is %lf\n", m_Errors[0]);
    printf("[Logging]\t\t Mean reprojection error for the 2nd baseline image is %lf\n", m_Errors[1]);

    for(unsigned int i=2;i<m_Views.size();++i){
        printf("[Logging]\t\t Computing the pose and reconstructing points for the view %s\n", m_Views[i].m_Name.c_str());
        compute_pose(m_Views[i], baseline_view1, false);
        printf("[Logging]\t\t Mean reprojection error for the view %s is %lf\n", m_Views[i].m_Name.c_str(), m_Errors[i]);
    }
    
    plot_points();     

}

void SFM::plot_points(){

    pcl::PointCloud<pcl::PointXYZ>::Ptr ptr_cld = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    ptr_cld->points.clear(); 

    for(unsigned int i=0; i<m_Points3D.rows; ++i){
        pcl::PointXYZ point;

        point.x = m_Points3D.at<double>(i, 0);
        point.y = m_Points3D.at<double>(i, 1);
        point.z = m_Points3D.at<double>(i, 2);

        ptr_cld->points.push_back(point);
    }

    pcl::visualization::PCLVisualizer vis3("XYZ point cloud visualizer");
    vis3.addPointCloud<pcl::PointXYZ>(ptr_cld);
    vis3.spin();

}


SFM::~SFM(){

}