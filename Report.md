# Sensor Fusion : Camera Based 2D Feature Tracking Project Report

- ####  MP.1 Data Buffer Optimization
	- Task:  Implement a vector for dataBuffer objects whose size does not exceed a limit (e.g. 2 elements). This can be achieved by pushing in new elements on one end and removing elements on the other end.
	- Implementation : When the frames exceed the limit, simply delete the old frame when the new frame arrives.
		```
        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
        if(dataBuffer.size() == dataBufferSize){
 
            dataBuffer.erase(dataBuffer.begin());
        }
        dataBuffer.push_back(frame);
		```



- ####  MP.2 Keypoint Detection
 	- Task: Add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType. Implement detectors HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT and make them selectable by setting a string accordingly.
	- Implementation: The HARRIS detector is implemented with the cv::cornerHarris function, followed with nms step to remove the close detection. Other detectors are implemented in detKeypointsModern function.
		```
        string detectorType = "HARRIS"; // Detectors: SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT
        

        if (detectorType.compare("SHITOMASI") == 0)
        {
            detKeypointsShiTomasi(keypoints, imgGray, false);
        } else if (detectorType.compare("HARRIS") == 0){
            detKeypointsHarris(keypoints, imgGray, false);
        } else {
            detKeypointsModern(keypoints, imgGray,detectorType, false);
        }

       ```
 - ####  MP.3 Keypoint Removal
 	- Task: Remove all keypoints outside of a pre-defined rectangle and only use the keypoints within the rectangle for further processing.
 	- Implementation: Iterate through all the detected keypoints and check if it falls withing the preciding vehicle ROI. Remove all the keypoints which are outside the ROI. 
Use cv::Rect::contains function to judge if the point is inside our ROI.
      ```
        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
        vector<cv::KeyPoint>::iterator keypoint;
        vector<cv::KeyPoint> keypoints_roi;
        if (bFocusOnVehicle)
        {
             for(keypoint = keypoints.begin(); keypoint != keypoints.end(); keypoint++)
             {
                 if (vehicleRect.contains(keypoint->pt))
                 {  
                    cv::KeyPoint newKeyPoint;
                    newKeyPoint.pt = cv::Point2f(keypoint->pt);
                    newKeyPoint.size = 1;
                    keypoints_roi.push_back(newKeyPoint);
                 }
             }
            keypoints =  keypoints_roi;
            cout << "The number of keypoint on focus area of the front vehicle is: " << keypoints.size()<<" keypoints"<<endl;
        }
      ```
  - ####  MP.4 Keypoint Descriptors
 	- Task : Implement descriptors BRIEF, ORB, FREAK, AKAZE and SIFT and make them selectable by setting a string accordingly.
	 - Implementation : The descriptors are implemented in the `matching2D_Student.cpp` and `MidTermProject_Camera_Student.cpp`.
 In the `MidTermProject_Camera_Student.cpp` the descriptor is initialised and the `descriptorType` is set. The `descKeypoints(....)` in  `matching2D_Student.cpp` contains the actual keypoint description implementation with selector for `descriptorType`. And cv::DescriptorExtractor is used to extract the features.
 
 	- In `MidTermProject_Camera_Student.cpp` : 

 ```
        cv::Mat descriptors;
        string descriptorType = "ORB"; // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT  
        descKeypoints(dataBuffer.rbegin()->keypoints, dataBuffer.rbegin()->cameraImg, descriptors, descriptorType);
 ```    
 	- In  `matching2D_Student.cpp` : 
 ```
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
```

  
 - ####  MP.5 Descriptor Matching  and MP.6 Descriptor Distance Ratio 
 	- Task : Implement FLANN matching as well as k-nearest neighbor selection. Both methods must be selectable using the respective strings in the main function. Use the K-Nearest-Neighbor matching to implement the descriptor distance ratio test, which looks at the ratio of best vs. second-best match to decide whether to keep an associated pair of keypoints.
  	- Implementation : The Brute Force matcher `MAT_BF` and FLANN matcher `MAT_FLANN` are implemented in the `matchDescriptors(....)` function (in `matching2D_Student.cpp`) and matcher type is selectable based on the string set in main file `MidTermProject_Camera_Student.cpp`. The matching task for the selected matcher type is implemented based on the selector type. Nearest neighbor (best match) `SEL_NN` and K-Nearest-Neighbor(KNN) `SEL_KNN` selection is implemented and is selectable based on the string set in main file `MidTermProject_Camera_Student.cpp`. For KNN , k = 2 and the matches are filtered using descriptor distance ratio test where the descriptor distance ratio threshold is 0.8 for matching. Select the distance type based on the feature type. Use L2 distance for SIFT, and Hamming distance for binary descriptors. 
```
      // Find best matches for keypoints in two camera images based on several matching methods
	void matchDescriptors(vector<cv::KeyPoint> &kPtsSource, vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef, vector<cv::DMatch> &matches, std::string descriptorType, string matcherType, string selectorType)
	{
	    // configure matcher
	    bool crossCheck = false;
	    cv::Ptr<cv::DescriptorMatcher> matcher;

	    if (matcherType.compare("MAT_BF") == 0)
	    {
		int normType = descriptorType.comparecv::DescriptorExtractor("DES_BINARY") == 0 ? cv::NORM_HAMMING : cv::NORM_L2;
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
```
  
 - ####  MP.7 Performance Evaluation 1	
 	- Task : Count the number of keypoints on the preceding vehicle for all 10 images and take note of the distribution of their neighborhood size. Do this for all the detectors you have implemented.
  	- Evaluation : The number of keypoints are listed for each image. Only the keypoints on the preceeding vehicle are considered.  

	
	  Detector |Img-1|	Img-2|Img-3	|Img-4	|Img-5	|Img-6	|Img-7	|Img-8	|Img-9	|Img-10
	----|-----|-----|-------|-------|-------|-----|-------|-------|-------|------
	**SHI-Tomasi**	|125|	118|	123|	120|	120|	113|	114|	123|	111|	112
	**Harris**		|17	|	14|		18|		21|		26|		43|		18|		31|		26|		34
	**FAST**		|149|	152|	150|	155|	149|	149|	156|	150|	138|	143
	**BRISK**		|264|	282|	282|	277|	297|	279|	289|	272|	266|	254
	**ORB**			|92	|	102|	106|	113|	109|	125|	130|	129|	127|	128
	**AKAZE**		|166|	157|	161|	155|	163|	164|	173|	175|	177|	179
	**SIFT**		|138|	132|	124|	137|	134|	140|	137|	148|	159|	137

  
 - #### MP.8 Performance Evaluation 2	
 	- Task : Count the number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors. In the matching step, the BF approach is used with the descriptor distance ratio set to 0.8.
  	- Evaluation : Below is the table with the averages over all 10 images. 

	| Detector\Descriptor | BRISK | BRIEF | ORB | FREAK | AKAZE | SIFT |
	| --- | --- | --- |--- |--- |--- |--- |
	| **SHITOMASI** | 767 |944|908|768|N/A|927|
	| **HARRIS** | 393|460 |451|396|N/A|459|
	| **FAST** | 899 |1099|1071|878|N/A|1046|
	| **BRISK** | 1570 |1704|1514|1524|N/A|1646|
	| **ORB** | 751 |545|763|420|N/A|763|
	| **AKAZE** | 1215 |1266|1182|1187|1259|1270|
	| **SIFT** | 592 |702|Out of Memory|593|N/A|800|

 
 -  #### MP.9 Performance Evaluation 3	
 	- Task : Log the time it takes for keypoint detection and descriptor extraction. The results must be entered into a spreadsheet and based on this data, the TOP3 detector / descriptor combinations must be recommended as the best choice for our purpose of detecting keypoints on vehicles.
 	- Evaluation : The above table lists the average time for keypoint detection and descriptor extraction over all 10 images.

	| Detector\Descriptor | BRISK | BRIEF | ORB | FREAK | AKAZE | SIFT |
	| --- | --- | --- |--- |--- |--- |--- |
	| **SHITOMASI** | 17.98 |21.38|18.8|52.4079|N/A| 31.82|
	| **HARRIS** | 16.98|17.11 |16.5383|51.01| N/A| 32.73|
	| **FAST** | 3.36 |1.8786 |2.03823|41.549|N/A|35.71|
	| **BRISK** | 43.736 |44.159|47.966|89.2|N/A|92.17|
	| **ORB** | 8.54 |8.035|11.81|47.578|N/A|51.6748|
	| **AKAZE** | 103.108 |81.3247|84.51|149.97|173.611|128.77|
	| **SIFT** | 124.09 |146.49|Out of Memory|188.17|N/A|181.0381|


- TOP3 detector / descriptor combinations are chosen based on the achieve minimal processing time with significant matches. 
	
	DETECTOR/DESCRIPTOR  | NUMBER OF KEYPOINTS | TIME
	-------------------- | --------------------| --------
	FAST+BRIEF           | 1099 keypoints    | 1,87 ms 
	FAST+ORB             | 1071 keypoints    | 2.03 ms 
	FAST+BRISK           | 899 keypoints     | 3.36 ms 
	-------------
