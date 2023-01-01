# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Dependencies for Running Locally
1. cmake >= 2.8
 * All OSes: [click here for installation instructions](https://cmake.org/install/)

2. make >= 4.1 (Linux, Mac), 3.81 (Windows)
 * Linux: make is installed by default on most Linux distros
 * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
 * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)

3. OpenCV >= 4.1
 * All OSes: refer to the [official instructions](https://docs.opencv.org/master/df/d65/tutorial_table_of_content_introduction.html)
 * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors. If using [homebrew](https://brew.sh/): `$> brew install --build-from-source opencv` will install required dependencies and compile opencv with the `opencv_contrib` module by default (no need to set `-DOPENCV_ENABLE_NONFREE=ON` manually). 
 * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)

4. gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using either [MinGW-w64](http://mingw-w64.org/doku.php/start) or [Microsoft's VCPKG, a C++ package manager](https://docs.microsoft.com/en-us/cpp/build/install-vcpkg?view=msvc-160&tabs=windows). VCPKG maintains its own binary distributions of OpenCV and many other packages. To see what packages are available, type `vcpkg search` at the command prompt. For example, once you've _VCPKG_ installed, you can install _OpenCV 4.1_ with the command:
```bash
c:\vcpkg> vcpkg install opencv4[nonfree,contrib]:x64-windows
```
Then, add *C:\vcpkg\installed\x64-windows\bin* and *C:\vcpkg\installed\x64-windows\debug\bin* to your user's _PATH_ variable. Also, set the _CMake Toolchain File_ to *c:\vcpkg\scripts\buildsystems\vcpkg.cmake*.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.

## Technical Report

### Task 1
The circular buffer was implemented using a conditional and the erase method of vector.

```c++
...
if(dataBuffer.size() > dataBufferSize)
{
  dataBuffer.erase(dataBuffer.begin());
}
...
```

### Task 2
The different detectors where added to the detKeypointsModern function using their default parameters. The detectors are selected by string comparison. E.g.
```c++
...
else if(detectorType.compare("BRISK")==0)
{
    detector = cv::BRISK::create();
}
else if(detectorType.compare("ORB")==0)
{
    detector = cv::ORB::create();
}
...
```

### Task 3
The keypoints corresponding to the preceding vehicle were filtered out using a cv::Rect object type and its "contains" method.

```c++
...
cv::Rect vehicleRect(535, 180, 180, 150);
if (bFocusOnVehicle)
{
    for(auto it = keypoints.begin(); it!= keypoints.end(); ++it)
    {
        if(!vehicleRect.contains(it->pt))
        {
            keypoints.erase(it--);
        }
    }
}
...
```

### Task 4
The different descriptors where added to the descKeypoints function in the same fashion as in Task 2. E.g.

```c++
...
else if (descriptorType.compare("BRIEF") == 0)
{
    extractor = cv::BRISK::create();
}
else if (descriptorType.compare("ORB") == 0)
{
    extractor = cv::ORB::create();
}
...
```

### Task 5
FLANN-based matching was added to the matchDescriptors function. The selection of the matching method happens again by string comparison. An important issue is the conversion to a 32-bit floating point due to a bug in OpenCV. The matcher is then created using its corresponding OpenCV implementation.

```c++
...
else if (matcherType.compare("MAT_FLANN") == 0)
{
  if (descSource.type() != CV_32F)
  {
      descSource.convertTo(descSource, CV_32F);
      descRef.convertTo(descRef, CV_32F);
  }
  matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
...
}
```

### Task 6
Filtering by descriptor distance ratio was implemented in the matchDescriptors function. The selection happens again by string comparison. kNN matching is performed by using the knnMatch method with k = 2. The threshold to filter out ambiguous matches is set to 0.8.

```c++
else if (selectorType.compare("SEL_KNN") == 0)
{
    vector<vector<cv::DMatch>> knnMatches;
    matcher->knnMatch(descSource, descRef, knnMatches, 2);

    const float ratioThresh = 0.8f;
    for (size_t i = 0; i < knnMatches.size(); i++)
    {
        if (knnMatches[i][0].distance < ratioThresh * knnMatches[i][1].distance)
        {
            matches.push_back(knnMatches[i][0]);
        }
    }
}
```

## Results
### Task 7
### Number of detected keypoints

| Frame | HARRIS | SHITOMASI | FAST | BRISK | ORB | SIFT | AKAZE |
|-------|--------|-----------|------|-------|-----|------|-------|
| 1     | 14     | 118       | 152  | 282   | 102 | 132  | 157   |
| 2     | 18     | 123       | 150  | 282   | 106 | 124  | 161   |
| 3     | 21     | 120       | 155  | 277   | 113 | 137  | 155   |
| 4     | 26     | 120       | 149  | 297   | 109 | 134  | 163   |
| 5     | 43     | 113       | 149  | 279   | 125 | 140  | 164   |
| 6     | 18     | 114       | 156  | 289   | 130 | 137  | 173   |
| 7     | 31     | 123       | 150  | 272   | 129 | 148  | 175   |
| 8     | 26     | 111       | 138  | 266   | 127 | 159  | 177   |
| 9     | 34     | 112       | 143  | 254   | 128 | 137  | 179   |
<p align = "center">
Table 1.- Number of keypoints per frame using different detectors.
</p>

### Task 8
### Average number of matched keypoints

|   Descriptor\Detector    | HARRIS | SHITOMASI | FAST   | BRISK  | ORB    | SIFT   | AKAZE  |
|--------------------------|--------|-----------|--------|--------|--------|--------|--------|
|          BRIEF           | 23.78  | 118.56    | 149.78 | 278.67 | 105.56 | 138.67 |   -    |
|          ORB             | 23.78  | 118.56    | 149.78 | 278.67 | 114.78 | N/A    |   -    |
|          FREAK           | 23.78  | 118.56    | 149.78 | 258.44 | 61     | 137.67 |   -    |
|          AKAZE           |   -    |   -       |   -    |   -    |   -    |   -    | 165.67 |
|          SIFT            | 23.78  | 118.56    | 149.78 | 278.67 | 114.78 | 138.78 |   -    |
<p align = "center">
Table 2.- Average number of matched keypoints per detector-descriptor combination.
</p>

### Task 9
### Average time for detection and descriptor extraction

| Descriptor\Detector | HARRIS | SHITOMASI | FAST | BRISK | ORB | SIFT | AKAZE |
|---------------------|--------|-----------|------|-------|-----|------|-------|
| BRIEF               | 19     | 452       | 447  | 943   | 407 | 562  | -     |
| ORB                 | 7      | 58        | 49   | 551   | 17  | -    | -     |
| FREAK               | 17     | 95        | 99   | 597   | 61  | 245  | -     |
| AKAZE               | -      | -         | -    | -     | -   | -    | 235   |
| SIFT                | 12     | 447       | 83   | 615   | 93  | 254  | -     |

<p align = "center">
Table 3.- Average time for detection and descriptor extraction per detector-descriptor combination.
</p>

### Recomendations
Since we would like to maximize the probabilities of detecting the car in front of us we would chose the average number of matched keypoints as the most important decision criterion when selecting a descriptor-detector combination. Since the computation time has to happen quickly enough to even be able to brake when presented to a critical situation. The second most important decision criterion would be the average time for detection and description extraction. Therefore we would use a compromise between number of matched keypoints and average computation time to help us make our decision and calculate the matching/time score based on the following ratio:

$$score = {{average\ number\ of\ matched\ keypoints}\over{average\ time}}$$

Resulting in:

| Descriptor\Detector | HARRIS | SHITOMASI | FAST | BRISK | ORB  | SIFT | AKAZE |
|---------------------|--------|-----------|------|-------|------|------|-------|
| BRIEF               | 1.25   | 0.26      | 0.34 | 0.30  | 0.26 | 0.25 | -     |
| ORB                 | 3.40   | 2.04      | 3.06 | 0.51  | 6.75 | -    | -     |
| FREAK               | 1.40   | 1.25      | 1.51 | 0.43  | 1.00 | 0.56 | -     |
| AKAZE               | -      | -         | -    | -     | -    | -    | 0.70  |
| SIFT                | 1.98   | 0.27      | 1.80 | 0.45  | 1.23 | 0.55 | -     |
<p align = "center">
Table 4.- "Efficiency score" per detector-descriptor combination.
</p>


Table 4 presents a summarized version of an "efficiency score". This information is however not enough to make a solid decision. We must combine information from the previous steps as well.
We can discard the values provided by the HARRIS corner detection because 14-40 detections might not suffice to produce a reliable matching.
From Table 1 we see that BRISK is consistently able to compute the highest number of keypoints and therefore also gets the higher number of matched keypoints regardless of the descriptor used. We see that BRISK would be the decision to go when requiring a higher confidence for our detection. However the detector-descriptor combination we use makes a considerable difference when it comes to the computation time. More than 500 ms for the BRISK-based detection would probably mean a collision if we drive at e.g. 60 km/h = 16.67 m/s. We would have traveled about 8 m after this period.


Therefore the best combinations considering this detections-time relation would be:
1) ORB-BRIEF
2) FAST-ORB
3) SHITOMASI-ORB