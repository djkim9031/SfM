#include "view.h"

View::View(std::string img_path, std::string root_path, bool hasFeatureFile, std::string feature_type){

    std::string filename = img_path.substr(img_path.rfind('/')+1, img_path.size());
    m_Extension = filename.substr(filename.rfind('.')+1, filename.size());
    m_Name = filename.substr(0, filename.size() - m_Extension.size()-1);

    m_Image = cv::imread(root_path+"/"+img_path);
    m_FeatureType = feature_type;
    m_RootPath = root_path;
    m_R = cv::Mat::zeros(3,3, CV_64F);
    m_T = cv::Mat::zeros(3,1, CV_64F);

    if(!hasFeatureFile){
        extract();
    }else{
        read();
    }
}

void View::extract(){
    //Extract features from the image

    if(m_FeatureType == "sift"){
      cv::Ptr<cv::SIFT> sift = cv::SIFT::create(50000);
      sift->detectAndCompute(m_Image, cv::Mat(), m_Keypoints, m_Descriptor);

    }else if(m_FeatureType == "surf"){
        //patented feature. Requires non free opencv option
        cv::Ptr<cv::xfeatures2d::SURF> surf = cv::xfeatures2d::SURF::create(10000);
        surf->detectAndCompute(m_Image, cv::Mat(), m_Keypoints, m_Descriptor);

    }else if(m_FeatureType == "orb"){
        cv::Ptr<cv::ORB> orb = cv::ORB::create(10000); //set the max number of features to 1000
        orb->detectAndCompute(m_Image, cv::Mat(), m_Keypoints, m_Descriptor);

    }else{
        fprintf(stderr, "[ERROR]\t\t Feature extration type not recognized. \n");
        exit(1);
    }

    printf("[Logging]\t\t Computed features for image %s \n", m_Name.c_str());
    write();
}

void View::write(){
    //It assumes that the executable file is located under the build directory
    std::string folder = m_RootPath + "/features";
    std::string file = folder+"/"+m_Name+".features";
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

    int num_Keypoints = m_Keypoints.size();
    int num_DescriptorCols = m_Descriptor.cols;
    fwrite(&num_Keypoints, sizeof(int), 1, fp);
    fwrite(&num_DescriptorCols, sizeof(int), 1, fp);

    for(unsigned int i=0; i<m_Keypoints.size(); ++i){

        fwrite(&m_Keypoints[i].pt.x, sizeof(float), 1, fp);
        fwrite(&m_Keypoints[i].pt.y, sizeof(float), 1, fp);
        fwrite(&m_Keypoints[i].size, sizeof(float), 1, fp);
        fwrite(&m_Keypoints[i].angle, sizeof(float), 1, fp);
        fwrite(&m_Keypoints[i].response, sizeof(float), 1, fp);
        fwrite(&m_Keypoints[i].octave, sizeof(int), 1, fp);
        fwrite(&m_Keypoints[i].class_id, sizeof(int), 1, fp);
        for(unsigned int j=0; j<m_Descriptor.cols;++j){
            fwrite(&m_Descriptor.row(i).at<float>(j), sizeof(float), 1, fp);
        }
    }

    fflush(fp);
    fclose(fp);

}

void View::read(){

    std::string folder = m_RootPath + "/features";
    std::string file = folder+"/"+m_Name+".features";
    printf("[Logging]\t\t Loading features of the image %s \n", m_Name.c_str());
    fflush(stdout);
    FILE *fp = fopen(file.c_str(), "rb");
    if(!fp){
        printf("[Logging]\t\t The features file for the specified image %s is not found. Trying to extract again... \n", m_Name.c_str());
        extract();
        return;
    }
    
    int major, minor, revision, num_Keypoints, num_DescriptorCols;
    fread(&major, sizeof(int), 1, fp);
    fread(&minor, sizeof(int), 1, fp);
    fread(&revision, sizeof(int), 1, fp);
    fread(&num_Keypoints, sizeof(int), 1, fp);
    fread(&num_DescriptorCols, sizeof(int), 1, fp);

    float* temp_pt = (float*)calloc(2, sizeof(float));
    float* temp_size = (float*)calloc(1, sizeof(float));
    float* temp_angle = (float*)calloc(1, sizeof(float));
    float* temp_response = (float*)calloc(1, sizeof(float));
    int* temp_octave = (int*)calloc(1, sizeof(int));
    int* temp_class_id = (int*)calloc(1, sizeof(int));
    float* temp_descriptor_vals = (float*)calloc(num_DescriptorCols, sizeof(float));

    m_Keypoints.clear(); //To make sure it's empty
    m_Descriptor.release();

    for(unsigned int i=0; i<num_Keypoints; ++i){
        
        fread(temp_pt, sizeof(float), 2, fp);
        fread(temp_size, sizeof(float), 1, fp);
        fread(temp_angle, sizeof(float), 1, fp);
        fread(temp_response, sizeof(float), 1, fp);
        fread(temp_octave, sizeof(int), 1, fp);
        fread(temp_class_id, sizeof(int), 1, fp);
        fread(temp_descriptor_vals, sizeof(float), num_DescriptorCols, fp);

        cv::KeyPoint kp = cv::KeyPoint(temp_pt[0], temp_pt[1], *temp_size, *temp_angle,
                                       *temp_response, *temp_octave, *temp_class_id);

        cv::Mat descriptor = cv::Mat(1, num_DescriptorCols, CV_32F, temp_descriptor_vals);
        
        m_Keypoints.push_back(kp);
        m_Descriptor.push_back(descriptor);
    }

    printf("[Logging]\t\t Done reading features of the image %s \n", m_Name.c_str());
    fclose(fp);

    free(temp_pt);
    free(temp_size);
    free(temp_angle);
    free(temp_response);
    free(temp_octave);
    free(temp_descriptor_vals);

}

View::~View(){

}

std::vector<View> create_views(std::string root_path, std::string image_format){
    //Looping through the images and creates an array of views
    struct stat pathValidityChecker;
    std::string folder = root_path + "/features";
    bool hasViews = true;

    if (stat(folder.c_str(), &pathValidityChecker) != 0){
        mkdir(folder.c_str(), 0777);
        hasViews = false;
    } 

    DIR *image_dir;
    struct dirent *en;
    std::string img_folder = root_path + "/images"; 
    image_dir = opendir(img_folder.c_str());
    std::vector<std::string> pictures;
    if (image_dir) {
      while ((en = readdir(image_dir)) != NULL) {
         std::string name = en->d_name;
         size_t found = name.find(image_format);
         if(found!=std::string::npos){
            pictures.push_back(name);
         }
      }
      closedir(image_dir); 
    }

    std::sort(pictures.begin(), pictures.end());

    std::vector<View> views;
    for(auto pic: pictures){
        views.push_back(View("images/"+pic, root_path, hasViews, "sift"));
    }
   
    return views;
}