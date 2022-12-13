#pragma once

#include <stdio.h>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv4/opencv2/xfeatures2d/nonfree.hpp>
#include <opencv4/opencv2/features2d/features2d.hpp>

#include <sys/stat.h>
#include <dirent.h>
#include <sys/types.h>

#include <tuple>

#include "data.h"

class View{

public:
    View(){};
    View(std::string img_path, std::string root_path, bool hasFeatureFile, std::string feature_type);

    void extract();
    void write();
    void read();

    ~View();

    std::string m_Name;
    std::string m_RootPath;
    std::string m_FeatureType;
    cv::Mat m_Descriptor;
    cv::Mat m_R;
    cv::Mat m_T;
    std::vector<cv::KeyPoint> m_Keypoints;

private:
    std::string m_Extension;
    cv::Mat m_Image;
    
};

std::vector<View> create_views(std::string root_path, std::string image_format="jpg");