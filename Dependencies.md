# LoS Monocular Exploration - Known Dependencies

In this document, we list all codes included by LoS-Exploration and linked libraries which are not our property.

### Library dependencies

- *AirSim*. [MIT License](https://en.wikipedia.org/wiki/MIT_License).

- *OpenCV*. [BSD License](https://en.wikipedia.org/wiki/BSD_licenses).

- *Pangolin*. [MIT License](https://en.wikipedia.org/wiki/MIT_License).

### Code in **src/src_orbslam**

- *ORB-SLAM*. We used the ORB-SLAM as the SLAM of our exploration. All code inside **src/src_orbslam** came from [ORB-SLAM](https://github.com/raulmur/ORB_SLAM2) repository. We made some minor changes to make it compatible with our exploration approach. The original code is [GPLv3](http://www.gnu.org/licenses/gpl-3.0.html) licensed.



## ORB-SLAM Inherit dependencies

Together with this project, we released all the [ORB-SLAM](https://github.com/raulmur/ORB_SLAM2) code with some minor changes to make it compatible with our exploration approach. So, we inherited these dependencies from it:

### Code in **src/src_orbslam**

- *ORBextractor.cc*. This is a modified version of orb.cpp of OpenCV library. The original code is BSD licensed.

- *PnPsolver.h*, *PnPsolver.cc*. This is a modified version of the epnp.h and epnp.cc of Vincent Lepetit. This code can be found in popular [BSD](https://en.wikipedia.org/wiki/BSD_licenses) licensed computer vision libraries as [OpenCV](https://github.com/opencv/opencv/blob/master/modules/calib3d/src/epnp.cpp) and [OpenGV](https://github.com/laurentkneip/opengv/blob/master/src/absolute_pose/modules/Epnp.cpp). The original code is [FreeBSD](https://en.wikipedia.org/wiki/FreeBSD_Documentation_License).

- Function ORBmatcher::DescriptorDistance in *ORBmatcher.cc*. The code is in the public domain and can be found [here](http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel).

### Code in third party folder

- All code in **DBoW2** folder. This is a modified version of [DBoW2](https://github.com/dorian3d/DBoW2) and [DLib](https://github.com/dorian3d/DLib) library. All files included are [BSD](https://en.wikipedia.org/wiki/BSD_licenses) licensed.

- All code in **g2o** folder. This is a modified version of [g2o](https://github.com/RainerKuemmerle/g2o). All files included are [BSD](https://en.wikipedia.org/wiki/BSD_licenses) licensed.

### Library dependencies

- *Eigen3*. For versions greater than 3.1.1 is [MPL2](https://www.mozilla.org/en-US/MPL/2.0/) licensed, earlier versions are [LGPLv3](http://www.gnu.org/licenses/lgpl-3.0) licensed.