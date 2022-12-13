#pragma once

#include "utils.h"
#include <stdio.h>
#include <math.h>

class Baseline{

public:
    Baseline(View &view1, View &view2, Match &match);
    std::pair<cv::Mat, cv::Mat> get_pose(const cv::Mat &K);
    std::pair<cv::Mat, cv::Mat> check_pose(const cv::Mat &E, const cv::Mat &K);
    std::pair<double, cv::Mat> triangulate(const cv::Mat &K, const cv::Mat &R, const cv::Mat &t);
    ~Baseline();

    Match m_Match;
    
private:
    View m_View1, m_View2;

};