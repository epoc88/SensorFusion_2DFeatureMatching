#include <numeric>
#include "matching2D.hpp"

using namespace std;

void showResults(cv::Mat img,std::vector<cv::KeyPoint> &keypoints, std::string windowName)
{
    cv::Mat visImage = img.clone();
    cv::drawKeypoints(img, keypoints, visImage,
    cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::namedWindow(windowName, 6);
    imshow(windowName, visImage);
    cv::waitKey(0);
}

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(vector<cv::KeyPoint> &kPtsSource, vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      vector<cv::DMatch> &matches, std::string descriptorType, string matcherType, string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {
        int normType = descriptorType.compare("DES_BINARY") == 0 ? cv::NORM_HAMMING : cv::NORM_L2;
        matcher = cv::BFMatcher::create(normType, crossCheck);
      	cout << "BF matching cross-check=" << crossCheck;
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        if (descSource.type() != CV_32F)
        { 
            descSource.convertTo(descSource, CV_32F);
            descRef.convertTo(descRef, CV_32F);
        }
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED); 
        cout << "FLANN matching";
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)
       
      	double t = (double)cv::getTickCount();
        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << " (NN) with n=" << matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;
        
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)
        vector<vector<cv::DMatch>> knn_matches;
		double t = (double)cv::getTickCount();
        matcher->knnMatch(descSource, descRef, knn_matches, 2); // finds two best matches
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << " (KNN) with n=" << knn_matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;

        // filter matches using descriptor distance ratio test
        double minDescDistRation = 0.8;

        for (auto& match : knn_matches) {
            if (match[0].distance < minDescDistRation * match[1].distance) {
                matches.push_back(match[0]);
            }
        }

        cout << "# keypoints removed = " << knn_matches.size() - matches.size() << endl;
    }
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;

    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.
        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }else if (descriptorType == "BRIEF"){
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
    }else if (descriptorType == "ORB"){
        extractor = cv::ORB::create();
    }else if (descriptorType == "FREAK"){
        extractor = cv::xfeatures2d::FREAK::create();
    }else if (descriptorType == "AKAZE"){
        extractor = cv::AKAZE::create();
    }else if (descriptorType == "SIFT"){
       // extractor = cv::xfeatures2d::SIFT::create();
		        extractor = cv::xfeatures2d::FREAK::create();
    }

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {
        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}



void detKeypointsHarris(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    string detectorName = "Harris";
    unsigned int blockSize = 2;   // for every pixel, a blockSize × blockSize neighborhood is considered 
    double k = 0.04;              // Harris parameter
    unsigned int thresh = 100;    // minimum value for a corner in the 8 bit scaled response matrix
    unsigned int apertureSize = 3;  //aperture parameter for Sobel operator

    double maxOverlap = 0.0; 	  // max. permissible overlap between two features in %, in non-maxima suppression


    // Harris corners detection and output normalization
    double t = (double)cv::getTickCount();

    cv::Mat distance = cv::Mat::zeros( img.size(), CV_32FC1 );
    cv::Mat distance_norm, distance_norm_scaled;
    cv::cornerHarris(img, distance, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    normalize( distance, distance_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
    convertScaleAbs( distance_norm, distance_norm_scaled );
    for( int i = 0; i < distance_norm.rows ; i++ )
    {
        for( int j = 0; j < distance_norm.cols; j++ )
        {
            int response = (int) distance_norm.at<float>(i,j);
            if( response > thresh ){
                cv::KeyPoint newKeyPoint(j,i,apertureSize*2);
                newKeyPoint.response = response;
                bool isOverlapped = false;
                for(auto point : keypoints)
                {
                    double overlap = cv::KeyPoint::overlap(newKeyPoint,point);
                    if (overlap > maxOverlap){
                        isOverlapped = true;
                        if(newKeyPoint.response > point.response){
                            point = newKeyPoint;
                            break;
                        }
                    }
                }
                if(!isOverlapped){
                    keypoints.push_back(newKeyPoint); // store new keypoint in dynamic list                   
                }
            }
        }
    }

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
	
    cout << "Harris detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl; 
    
  	if(bVis)
    {
        string windowName = detectorName + " Detector Results";
        showResults(img,keypoints,windowName);
    }
}


void detKeypointsModern(vector<cv::KeyPoint> & keypoints, cv::Mat &img, string detectorType, bool bVis)
{
    cv::Ptr<cv::FeatureDetector> detector;

    if (detectorType == "FAST"){
        detector = cv::FastFeatureDetector::create();
    }else if (detectorType == "BRISK"){
        detector = cv::BRISK::create();
    }else if (detectorType == "ORB"){
        detector = cv::ORB::create();
    }else if (detectorType == "AKAZE"){
        detector = cv::AKAZE::create();
    }else if (detectorType == "SIFT"){
        //detector = cv::xfeatures2d::SIFT::create();
		detector = cv::AKAZE::create();
    }else{
        cout << " Invalid detectorType !" << endl;
    }

    // Apply detection 
    double t = (double)cv::getTickCount();
    detector->detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << detectorType << " detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        string windowName = detectorType + " Detector Results";
        showResults(img,keypoints,windowName);
    }
}
