#include "match.h"

Match::Match(View &view1, View &view2, bool hasMatchFile){

    m_View1 = view1;
    m_View2 = view2;
    m_ImageName1 = view1.m_Name;
    m_ImageName2 = view2.m_Name;
    m_RootPath = view1.m_RootPath;
    m_MatchName = m_View1.m_Name + "_" + m_View2.m_Name;
    m_Matcher = cv::BFMatcher();

    if(m_View1.m_FeatureType == "sift" || m_View1.m_FeatureType == "surf" ){
        m_Matcher.create(cv::NORM_L2, true);
    } else {
        //orb
        m_Matcher.create(cv::NORM_HAMMING, true);
    }

    if(!hasMatchFile){
        get();
    } else {
        read();
    }

}

void Match::get(){

    std::vector<cv::DMatch> matches;
    m_Matcher.match(m_View1.m_Descriptor, m_View2.m_Descriptor, matches);
    /*
    std::sort(matches.begin(), matches.end(), 
              [](const cv::DMatch &elem1,const cv::DMatch &elem2){
                    if(elem1.distance==elem2.distance){
                        return elem1.queryIdx<elem2.queryIdx;
                    }
                    return elem1.distance<elem2.distance;
                }
    );*/
    

    for(auto match : matches){
        m_Indices1.push_back(match.queryIdx);
        m_Indices2.push_back(match.trainIdx);
        m_Distances.push_back(match.distance);
    }

    printf("[Logging]\t\t Computed matches between view %s and view %s \n", m_ImageName1.c_str(), m_ImageName2.c_str());

    write();
}

void Match::write(){

    std::string folder = m_RootPath + "/matches";
    std::string file = folder+"/"+m_ImageName1 + "_" + m_ImageName2 +".matches";
    FILE *fp = fopen(file.c_str(), "wb");
    if(!fp){
        fprintf(stderr, "[ERROR]\t\t Couldn't open the file: %s\n", file.c_str());
        exit(0);
    }

    int major = MAJOR_VERSION;
    int minor = MINOR_VERSION;
    int revision = PATCH_VERSION;
    fwrite(&major, sizeof(int), 1, fp);
    fwrite(&minor, sizeof(int), 1, fp);
    fwrite(&revision, sizeof(int), 1, fp);
    int numIndices = m_Indices1.size();
    fwrite(&numIndices, sizeof(int), 1, fp);

    for(unsigned int i=0; i<m_Indices1.size(); ++i){
        fwrite(&m_Distances[i], sizeof(float), 1, fp);
        fwrite(&m_Indices1[i], sizeof(int), 1, fp);
        fwrite(&m_Indices2[i], sizeof(int), 1, fp);
    }

    fflush(fp);
    fclose(fp);

}

void Match::read(){

    std::string folder = m_RootPath + "/matches";
    std::string file = folder+"/"+m_ImageName1 + "_" + m_ImageName2 +".matches";
    printf("[Logging]\t\t Loading match file %s \n", file.c_str());
    fflush(stdout);
    FILE *fp = fopen(file.c_str(), "rb");
    if(!fp){
        printf("[Logging]\t\t The match file %s is not found. Trying to match again... \n", file.c_str());
        get();
        return;
    }
    
    int major, minor, revision, numIndices;
    fread(&major, sizeof(int), 1, fp);
    fread(&minor, sizeof(int), 1, fp);
    fread(&revision, sizeof(int), 1, fp);
    fread(&numIndices, sizeof(int), 1, fp);

    float* temp_dist = (float*)calloc(1, sizeof(float));
    int* temp_idx1 = (int*)calloc(1, sizeof(int));
    int* temp_idx2 = (int*)calloc(1, sizeof(int));

    m_Distances.clear(); //To make sure it's empty
    m_Indices1.clear();
    m_Indices2.clear();

    for(unsigned int i=0; i<numIndices; ++i){
        
        fread(temp_dist, sizeof(float), 1, fp);
        fread(temp_idx1, sizeof(int), 1, fp);
        fread(temp_idx2, sizeof(int), 1, fp);

        m_Distances.push_back(*temp_dist);
        m_Indices1.push_back(*temp_idx1);
        m_Indices2.push_back(*temp_idx2);

    }

    printf("[Logging]\t\t Done reading the match file %s \n", file.c_str());
    fclose(fp);

    free(temp_dist);
    free(temp_idx1);
    free(temp_idx2);

    
}

Match::~Match(){

}

std::vector<Match> create_matches(std::vector<View> &views){
    
    struct stat pathValidityChecker;
    std::string folder = views[0].m_RootPath + "/matches";
    bool hasMatches = true;

    if (stat(folder.c_str(), &pathValidityChecker) != 0){
        mkdir(folder.c_str(), 0777);
        hasMatches = false;
    } 

    std::vector<Match> matches;
    for(unsigned int i=0;i<views.size()-1;++i){
        for(unsigned int j=i+1;j<views.size();++j){
            Match match(views[i], views[j], hasMatches);
            matches.push_back(match);
        }
    }

    return matches;
}