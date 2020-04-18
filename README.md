# SFND 3D Object Tracking

<img src="images/course_code_structure.png" width="779" height="414" />

In this final project, I have completed four major tasks: 
1. First, I have developed a way to match 3D objects over time by using keypoint correspondences. 
2. Second, I have computed the TTC based on Lidar measurements. 
3. Then I have proceeded to do the same using the camera, which requires to first associate keypoint matches to regions of interest and then to compute the TTC based on those matches. 
4. And lastly, I have conducted various tests with the framework. The goal was to identify the most suitable detector/descriptor combination for TTC estimation and also to search for problems that can lead to faulty measurements by the camera or Lidar sensor. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* Git LFS
  * Weight files are handled using [LFS](https://git-lfs.github.com/)
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

## Project Rubric and Explanation

**FP. 1. Implement the method "matchBoundingBoxes", which takes as input both the previous and the current data frames and provides as output the ids of the matched regions of interest (i.e. the boxID property). Matches must be the ones with the highest number of keypoint correspondences.**

The code is implemented in the method "matchBoundingBoxes" in the camFusion_Student.cpp file. It involved looping through all of the keypoints, extracting their corresponding keypoints from the matches and checking if they lie within the roi defined within the bounding boxes. If they do, the boundingBox IDs are pushed into a multimap. The highest number of correspondences for each bounding box is then counted in the multimap.  

**FP. 2. Compute the time-to-collision in seconds for all matched 3D objects using only Lidar measurements from the matched bounding boxes between current and previous frame.**

The code for the same has been implemented in the method " computeTTCLidar". It involves calculating the mean of the cropped lidar points from the previous frame and the current frame and using their difference to calculate the TTC, since a mean is more robust than using the closest lidar to the host.

**FP. 3. Prepare the TTC computation based on camera measurements by associating keypoint correspondences to the bounding boxes which enclose them. All matches which satisfy this condition must be added to a vector in the respective bounding box.**

The method "clusterKptMatchesWithROI" was used to implement the code for the same. It involved looping through the keypoints in the current frame and checking if the keypoint lies within the ROI of the bounding box. If the keypoints lie within the boundingBox, the keypoint and its corresponding match are pushed to the boundingbox's keypoint matches.

**FP. 4. Compute the time-to-collision in second for all matched 3D objects using only keypoint correspondences from the matched bounding boxes between current and previous frame.**

The TTC for Camera is calculated in the method "ComputeTTCCamera" using the distance ratios for the keypoints correspondences. The same is calculated using different combinations of detector-descriptors.

**FP. 5. Find examples where the TTC estimate of the Lidar sensor does not seem plausible. Describe your observations and provide a sound argumentation why you think this happened.**

The TTC estimate for the Lidar was found to be noisy and too high and too low for several of the images. The TTC Lidar is expected to behave in a more robust way and not jump around to such an extent between frames. Some reasons of this behavior could be: 
1. The TTC estimate equation does not take into account the acceleration with respect to the preceding car since the equation is constant- velocity based. 
2. The number of frames taken into account for each TTC calculation is only 2. This is unrealitic since it could lead to noisy estimates as small variations in lidar measurements between frames could lead to undue variations in the TTC.

**FP.6. Run several detector / descriptor combinations and look at the differences in TTC estimation. Find out which methods perform best and also include several examples where camera-based TTC estimation is way off. As with Lidar, describe your observations again and also look into potential reasons.**

Several detector-descriptor combinations were performed and the Camera and Lidar based TTCs were calculated.

SIFT - FREAK : The Camera Based TTC measurements were way off.(The results were negative)

SIFT - BRIEF : The highest difference was found to be 12 sec.

SIFT - BRISK : The Camera Based TTC measurements were way off.(The results were negative)

AKAZE - ORB : The highest difference was found to be 6 sec.

AKAZE - FREAK : The highest difference was found to be 6 sec.

AKAZE - BRIEF :The highest difference was found to be 4 sec.

AKAZE - BRISK : Performed well. The highest difference was found to be 6 sec

ORB - ORB : The highest difference was found to be 38 sec

ORB - FREAK : The Camera Based TTC measurements were way off.(The results were negative)

ORB - BRIEF : The highest difference was found to be 14 sec.

ORB - BRISK : The highest difference was found to be 130 sec.

BRISK - ORB : The highest difference was found to be 30 sec.

BRISK - FREAK : The highest difference was found to be 20 sec.

BRISK - BRIEF : The highest difference was found to be 11 sec.

BRISK - BRISK : The Camera Based TTC measurements were way off.(Upto 50 sec)

FAST - ORB : The Camera based TTC measurements was off for 1 frame but was close for others.

FAST - FREAK : The Camera Based TTC measurements were way off.(The results were negative)

FAST - BRIEF : The Camera Based TTC measurements was off for a couple of frames.

FAST - BRISK : The Camera Based TTC measurements were way off.(The results were negative)

HARRIS - ORB :The Camera based TTC measurements were close to Lidar but were off for some frames. The highest difference was found to be 15 sec.

HARRIS - FREAK : The Camera Based TTC measurements were way off.(The results were negative)

HARRIS - BRIEF : The Camera based TTC measurements were close to Lidar but were off for some frames. The highest difference was found to be 10 sec.

HARRIS - BRISK : The Camera Based TTC measurements were way off.(The results were negative)

SHITOMASI - ORB : The Camera Based TTC measurements were found to perform very well as they followed the Lidar measurements were closely. The highest difference was found to be 3 sec.

SHITOMASI - FREAK : The Camera Based TTC measurements were way off.(The results were negative)

SHITOMASI - BRIEF : The Camera Based TTC measurements were found to perform very well as they followed the Lidar measurements closely. However, there are still variations as the TTC camera was found to decrease when the lidar TTC increased. The highest difference was found to be 6 sec.

SHITOMASI - BRISK : The Camera Based TTC measurements were way off.(The results were negative)

AKAZE - AKAZE : The highest difference was found to be 12 sec.

Based on the above data, we can see that there were several instances where the TTC estimate for Camera was way off compared to Lidar. The best combination was found to be SHITOMASI-ORB and AKAZE-BRIEF. 



