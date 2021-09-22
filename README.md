# SFND 3D Object Tracking

Welcome to the final project of the camera course. By completing all the lessons, you now have a solid understanding of keypoint detectors, descriptors, and methods to match them between successive images. Also, you know how to detect objects in an image using the YOLO deep-learning framework. And finally, you know how to associate regions in a camera image with Lidar points in 3D space. Let's take a look at our program schematic to see what we already have accomplished and what's still missing.

<img src="images/course_code_structure.png" width="779" height="414" />

In this final project, you will implement the missing parts in the schematic. To do this, you will complete four major tasks: 
1. First, you will develop a way to match 3D objects over time by using keypoint correspondences. 
2. Second, you will compute the TTC based on Lidar measurements. 
3. You will then proceed to do the same using the camera, which requires to first associate keypoint matches to regions of interest and then to compute the TTC based on those matches. 
4. And lastly, you will conduct various tests with the framework. Your goal is to identify the most suitable detector/descriptor combination for TTC estimation and also to search for problems that can lead to faulty measurements by the camera or Lidar sensor. In the last course of this Nanodegree, you will learn about the Kalman filter, which is a great way to combine the two independent TTC measurements into an improved version which is much more reliable than a single sensor alone can be. But before we think about such things, let us focus on your final project in the camera course. 

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
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.

## Rubric Points
* FP.1 Match 3D Objects
  * In order to match bounding boxes in the previous and the current frame, each of the bounding boxes in both frames are iterated over to check for all the matched pairs found by descriptor matching. The bounding boxes pair with the highest number of keypoints matched pairs are considered the best matches. The best matches are used for following tasks.
* FP.2 Compute Lidar-based TTC
  * The equation from the lecture was used to compute Lidar based TTC. We need to find out the distance from the ego vehicle to the preceding vehicle according to the lidar point cloud. First, all lidar points which didn't fall within the ego lane is dismissed. Then, in order to be more robust against outliers, the median of the whole point cloud x measurements were used. It'll be less robust if the mean is used as mean is subject to more influence of the outliers.
* FP.3 Associate Keypoint Correspondences with Bounding Boxes
  * Outliers are removed based on eculidean distance. The median of the eculidean distance for all the keypoints are calculated. Then a tolerance is set. For any keypoint that's beyond the tolerance, it's dismissed. 
* FP.4 Compute Camera-based TTC
  * The scale change of the preceding vehicle between the previous and current frame is used to compute camera-based TTC. After all the outliers are removed, the eculidean distance of all the keypoints in both frames are calculated. The median distance is used for a more robust result.
* FP.5 Performance Evaluation 1
  * The TTC for both camera and lidar has been recorded and plotted in an excel named "TTC Camera and Lidar.xlsx". Based on the plotting for xmin throughout the time, it can be seen that there isn't sudden change of relative speed between the ego and preceding vehicle. The measured Lidar TTC decreases in the overall trend. However the measurements at frame 5 and 6 seems off. They are represented by the sudden jump up on the plot. 
  * This might have happened because of inaccurate time interval. We used frame rate in this case, which is constant for all the frames. However in reality the frame rate of any sensor is not perfectly constant. 
* FP.6 Performance Evaluation 2
  * Different combination of detectors and descriptors are looked into. The results are recorded and plotted in an excel named "Different Detector and Descriptor.xlsx".
  * In general, when ORB is used as detector, TTC values calculated for camera have some extreme values, both negative and positve. This could come frome the outliers that's not removed successfully. On the other hand, SHITOMASI seems to work well with all other descriptors. SHITOMAS/ORB gives reasonable output for TTC and this combination also has the shortest process time when SHITOMAS is chosen as the detector.
  * There are several unreasonable TTC measurements for camera. Negative values indicates that the distance between the ego and preceding vehicle is increasing. This might have happened due to the reason that using median alone is not robust enough. There are also negative infinite values. It could be the results of the feature eculidean distance being very similar for two frames. 
