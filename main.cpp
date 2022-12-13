#include <iostream>
#include "view.h"
#include "match.h"
#include "utils.h"
#include "baseline.h"
#include "sfm.h"


int main(int argc, char** argv){


    cv::Mat K = cv::Mat::zeros(3,3,CV_64F);
    K.at<double>(0,0)=2759.48;
    K.at<double>(1,1)=2764.16;
    K.at<double>(2,2)=1.0;
    K.at<double>(0,2)=1520.69;
    K.at<double>(1,2)=1006.81;

    std::vector<View> views = create_views("..");
    std::vector<Match> matches = create_matches(views);

    SFM sfm(views, matches, K);
    sfm.reconstruct();


    return 0;
}