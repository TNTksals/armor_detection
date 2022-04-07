# Armor Detection

## Overview

The code about armor-detector that written during the final assignment 

Dependencies:

- ROS noetic
- OpenCV 4.2.0

## Getting started 

### Download and build code

1. Get the source 

```bash
git clone git@github.com:TNTksals/vision-armor-detection.git
```

2. Build in your workspace

```bash
catkin build
source ./devel/setup.bash
```

### Test

1. Using [rosmon](http://wiki.ros.org/rosmon), run

```bash
mon launch armor_detector armor_detector.launch
```

check the image on rqt_image_view and the tf on rviz.

![image-20220408013500901](/home/kslas/snap/typora/57/.config/Typora/typora-user-images/image-20220408013500901.png)



![image-20220408013620908](/home/kslas/snap/typora/57/.config/Typora/typora-user-images/image-20220408013620908.png)

2. Adjust the params by rqt_reconfigure:

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

![image-20220408013702105](/home/kslas/snap/typora/57/.config/Typora/typora-user-images/image-20220408013702105.png)

## Contact Information

[ROS]: http://www.ros.org

[rviz]: http://wiki.ros.org/rviz

[OpenCV]: https://opencv.org/

