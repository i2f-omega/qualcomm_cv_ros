# qualcomm_cv_ros
ROS package for computer vision demos, working towards Terrain Relative Navigation, using the ModalAI Qualcomm Flight Pro.


## Dependencies / Setup
1. ModalAI Qualcomm must be set up following the quick start guides in the [ModalAI Docs](https://docs.modalai.com/)
   1. Make sure highres camera is connected to J2 port in proper direction
      1. Wrong direction will cause device to not show with `$ adb devices`
   2. Select camera configuration 4 detailed [here](https://docs.modalai.com/configure-cameras/) and [here](https://docs.modalai.com/camera-connections/), even though we don't have a tracking camera
   3. This repo uses ROS, so make sure those setup steps are followed
2. We use the 4k High-resolution Sensor for VOXL (IMX377 M12-style Lens) from ModalAI found [here](https://www.modalai.com/products/4k-high-resolution-sensor-for-voxl-imx377-m12)
   1. We also have an interchangeable lens pack to change focal length and Field Of View (FOV), hence the focal length launch parameter in our scripts/launch files
3. Ensure camera and ROS are working, as stated [here](https://docs.modalai.com/voxl-cam-ros/)
   1. Launch roscore
      1. `$ adb shell`
      2. `# bash`
      3. `yocto:/# roscore`
   2. Launch High resolution 
   3. `yocto:/# roslaunch voxl_cam_ros  hires.launch`
      1. Launches the greyscale highres camera stream
      2. For some reason, color highres won't display on RVIZ...
   4. Diplay stream on host PC with RVIZ
      1. `$ rviz`
      2. Select `Add` button -> `/hires/image_raw/Image/raw`
4. Install OpenCV for Qualcomm found on the VOXl GitLab [here](https://gitlab.com/voxl-public/voxl-opencv-3-4-6)
5. _____ is a ROS package built following these steps [here](https://docs.modalai.com/build-ros-nodes-for-voxl/)
   1. It is designed to be run on the Qualcomm itself. This involves developing on the host PC and using adb to push updates onto the Qualcomm as [here](https://docs.modalai.com/setup-adb/)
      1. `adb push <host/pc/package/path>/_______ ____/____/____`
      2. TODO: Fill in blanks for package name and paths