#pragma once

#include "view.h"


class Match{

public:
    Match(){};
    Match(View &view1, View &view2, bool hasMatchFile);
    void get();
    void write();
    void read();

    ~Match();

    std::vector<int> m_Indices1, m_Indices2; //indices of the matched keypoints in view1 and view2, respectively
    std::vector<float> m_Distances; //distance between the matched keypoints in the first view
    std::vector<int> m_Inliers1, m_Inliers2; // list to store the indices of the keypoints from view not removed using the fundamental matrix
    std::string m_MatchName;

private:
    std::string m_ImageName1, m_ImageName2;
    std::string m_RootPath;
    View m_View1, m_View2;
    cv::BFMatcher m_Matcher;

};

std::vector<Match> create_matches(std::vector<View> &views);