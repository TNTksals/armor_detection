# Armor Detection

## Overview

The code about armor-detector that written during the final assignment 

Dependencies:

- ROS noetic
- OpenCV 4.2.0

## Getting started 

### Download and build code

1. Get the source 

```
git clone git@github.com:TNTksals/vision-armor-detection.git
```

2. Build in your workspace

```
catkin build
source ./devel/setup.bash
```

### Test

1. Using [rosmon](http://wiki.ros.org/rosmon), run

```
mon launch armor_detector armor_detector.launch
```

check the image on rqt_image_view and the tf on rviz.

![image_view](https://user-images.githubusercontent.com/89313083/162264700-22995e37-ce1b-4374-a5b4-8a4e665b12c5.png)



![rviz](https://user-images.githubusercontent.com/89313083/162264738-f22495a9-f79b-40c4-ae91-30a4269a9b81.png)

2. Adjust the params by rqt_reconfigure:

```
rosrun rqt_reconfigure rqt_reconfigure
```

![dynamic_reconfigure](https://user-images.githubusercontent.com/89313083/162264803-d63b06b3-6c92-4739-be31-585504e339ca.png)

## Contact Information

\[ ROS \]: http://www.ros.org

\[ rviz \]: http://wiki.ros.org/rviz

\[ OpenCV \]: https://opencv.org/

