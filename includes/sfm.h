#pragma once

#include <map>
#include "utils.h"
#include "baseline.h"

//Import PCL for point cloud registration
#include <pcl-1.12/pcl/common/point_tests.h>
#include <pcl-1.12/pcl/kdtree/kdtree.h>
#include <pcl-1.12/pcl/point_types.h>
#include <pcl-1.12/pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>


class SFM {

public:

    SFM(std::vector<View> &views, std::vector<Match> &matches, const cv::Mat &K);

    int get_index_of_view(const View &view) const { 
        for(int i=0;i<m_Names.size();++i){
            if(m_Names[i]==view.m_Name){
                return i;
            }
        }
        return -1;
    }

    void remove_mapped_points(Match &match, int image_idx);
    void compute_pose(View &view1, View &view2, bool isBaseline);
    void triangulate(const View &view1, const View &view2, const Match &match, std::vector<double> &rpe1, std::vector<double> &rpe2);
    void compute_pose_PNP(const View &view, cv::Mat &R, cv::Mat &t);
    void reconstruct();
    void plot_points();

    ~SFM();

private:

    std::vector<View> m_Views, m_Done;
    std::vector<Match> m_Matches;
    std::vector<std::string> m_Names;
    cv::Mat m_K, m_Points3D;
    unsigned int m_PointCounter;
    std::map<std::pair<int, int>, int> m_PointMap;
    std::vector<double> m_Errors;
    std::string m_Results_Path;

};