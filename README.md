# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.


## Report



| Detector | Descriptor | Keypoints | Detection Time (ms) | Descriptor Extraction Time (ms) | Matches | Match / ms |
| ---   | ---  | --- | --- | --- | --- | --- |
| SHITOMASI | BRISK| 117| 16| 385| 76| 0.189526|
| SHITOMASI | BRIEF| 117| 16| 1| 90| 5.29412|
| SHITOMASI | ORB| 117| 17| 0| 85| 5|
| SHITOMASI | FREAK| 117| 12| 46| 63| 1.08621|
| SHITOMASI | SIFT| 117| 11| 16| 103| 3.81481|
| HARRIS | BRISK| 24| 16| 384| 11| 0.0275|
| HARRIS | BRIEF| 24| 16| 0| 15| 0.9375|
| HARRIS | ORB| 24| 16| 0| 15| 0.9375|
| HARRIS | FREAK| 24| 16| 44| 10| 0.166667|
| HARRIS | SIFT| 24| 18| 14| 18| 0.5625|
| FAST | BRISK| 409| 1| 390| 203| 0.519182|
| FAST | BRIEF| 409| 1| 1| 242| 121|
| FAST | ORB| 409| 1| 1| 229| 114.5|
| FAST | FREAK| 409| 1| 48| 174| 3.55102|
| FAST | SIFT| 409| 1| 47| 309| 6.4375|
| BRISK | BRISK| 276| 433| 389| 144| 0.175182|
| BRISK | BRIEF| 276| 433| 1| 149| 0.343318|
| BRISK | ORB| 276| 431| 4| 103| 0.236782|
| BRISK | FREAK| 276| 430| 47| 121| 0.253669|
| BRISK | SIFT| 276| 432| 61| 182| 0.369168|
| ORB | BRISK| 116| 7| 386| 72| 0.183206|
| ORB | BRIEF| 116| 7| 0| 50| 7.14286|
| ORB | ORB| 116| 7| 4| 58| 5.27273|
| ORB | FREAK| 116| 7| 45| 38| 0.730769|
| ORB | SIFT| 116| 7| 70| 84| 1.09091|
| AKAZE | AKAZE| 167| 101| 87| 130| 0.691489|
| AKAZE | BRISK| 167| 109| 386| 123| 0.248485|
| AKAZE | BRIEF| 167| 110| 0| 120| 1.09091|
| AKAZE | ORB| 167| 109| 3| 102| 0.910714|
| AKAZE | FREAK| 167| 106| 47| 108| 0.705882|
| AKAZE | SIFT| 167| 99| 30| 141| 1.09302|
| SIFT | BRISK| 138| 160| 375| 59| 0.11028|
| SIFT | BRIEF| 138| 163| 0| 66| 0.404908|
| SIFT | FREAK| 138| 160| 47| 56| 0.270531|
| SIFT | SIFT| 138| 141| 91| 88| 0.37931|
