# qualcomm_cv_ros
ROS package for computer vision demos, working towards Terrain Relative Navigation, using the ModalAI Qualcomm Flight Pro.


## Dependencies / Setup
1. ModalAI Qualcomm must be set up following the quick start guides in the [ModalAI Docs](https://docs.modalai.com/)
   1. Make sure the highres camera is connected to J2 port in the proper direction
      1. Wrong direction will cause device to not show with `$ adb devices`
   2. Select camera configuration 4 detailed [here](https://docs.modalai.com/configure-cameras/) and [here](https://docs.modalai.com/camera-connections/), even though we don't have a tracking camera
   3. This repo uses ROS, so make sure those setup steps are followed
2. We use the 4k High-resolution Sensor for VOXL (IMX377 M12-style Lens) from ModalAI found [here](https://www.modalai.com/products/4k-high-resolution-sensor-for-voxl-imx377-m12)
   1. We also have an interchangeable lens pack to change focal length and Field Of View (FOV), hence the focal length launch parameter in our scripts/launch files
3. Ensure camera and ROS are working, as stated [here](https://docs.modalai.com/voxl-cam-ros/)
   1. Launch High resolution camera
      1. `$ adb shell`
      2. `# bash`
      3. `yocto:/# roslaunch voxl_cam_ros  hires.launch`
         1. Launches the greyscale highres camera stream
         2. Alternatively: `yocto:/# roslaunch voxl_cam_ros  hires_color.launch`
   2. Diplay stream on host PC with RVIZ
      1. `$ rviz`
      2. Select `Add` button and select `/hires/image_raw/Image/raw`
4. Install OpenCV for Qualcomm found on the VOXL GitLab [here](https://gitlab.com/voxl-public/voxl-opencv-3-4-6)
5. `qualcomm_cv_ros` is a ROS package built following the steps [here](https://docs.modalai.com/build-ros-nodes-for-voxl/)
   1. Instead of cloning their package, clone ours with `git clone https://github.com/i2f-omega/qualcomm_cv_ros.git`
      1. Make sure to pull using HTTPS or you'll get permission issues
   2. It is designed to be run on the Qualcomm itself. This involves developing on the host PC and using adb to push updates onto the Qualcomm as detailed [here](https://docs.modalai.com/setup-adb/) or pulling the updated repo directly from GitHub
      1. `$ adb push <host/pc/path>/catkin_ws/src/qualcomm_cv_ros/src/<file_name>.py /home/root/catkin_ws/src/qualcomm_cv_ros/src/`
         1. Same procedure for `.launch` files
      2. Alternatively, you can git pull directly onto the Qualcomm if repo is updated
      3. NOTE: If you add files with adb and then pull updated repo you may have to delete file on the board with `yocto:/# rm <path/to/filename>` to resolve conflicts

**Don't forget to `yocto:/# source ~/catkin_ws/devel/setup.bash` if it isn't finding the launch files or executables you're trying to run**

## Current usage
Currently we use the `voxl_cam_ros` package from ModalAI for accessing/publishing the video stream.
We will likely incorporate this launch file and/or source code into our own package to streamline the process.
But for now, use these steps:

1. Launch high resolution camera stream
   1. `$ adb shell`
   2. `# bash`
   3. `yocto:/# roslaunch voxl_cam_ros hires.launch`
2. Run the image analysis script of choice
   1. `$ adb shell`
   2. `# bash`
   3. `yocto:/# rosrun qualcomm_cv_ros <script_name>.py`
3. Display stream on host PC with RVIZ
   1. `$ rviz`
   2. Select `Add` button and select `<repubbed/image/topic>`

### Feature Matching Notes
1. There are two feature detection algorithms implemented. ORB is worse, faster, and can run on the Qualcomm, SIFT is better, slower, and can only (as of now) run on the host PC.
   1. ORB - Fast (~40x SIFT), Open Source, Not scale or rotationally invariant
   2. SIFT - Slow, Patented, scale and rotationally invariant
2. SIFT requires `opencv_contrib` which is not installed on the ModalAI Qualcomm Flight Pro boards
   1. You must use ORB if you are performing the feature matching on the Qualcomm
   2. You may use SIFT if you subscribe to the image from the Qualcomm and perform matching on the host PC.
   3. Install on host PC with: `pip install opencv-contrib-python==<OpenCV_version_number>`
3. SURF would be a decent middle ground to implement (between ORB & SIFT), but would have the same issues as SIFT of only running on the host PC.

## File camera
To stream video from a file, we use [video_stream_opencv](https://github.com/ros-drivers/video_stream_opencv).

1. Build `video_stream_opencv`:
```bash
# From your catkin workspace:
cd src
git clone https://github.com/ros-drivers/video_stream_opencv.git
cd ..
catkin_make
```
2. Download flyby video (e.g. https://www.jpl.nasa.gov/video/details.php?id=1167).
3. Launch file_cam node:
```bash
# From qualcomm_cv_ros:
roslaunch launch/file_cam.launch file:=$YOUR_VIDEO_FILE viz:=true start:=5300 stop:=7000

# This is equivalent to:
roslaunch ../video_stream_opencv/launch/camera.launch \
    video_stream_provider:=$YOUR_VIDEO_FILE \
    start_frame:=5300 stop_frame:=7000
```
Most arguments are optional. Additional arguments can be found in [video_stream_opencv/launch/camera.launch](https://github.com/ros-drivers/video_stream_opencv/blob/master/launch/camera.launch)

`file_cam.launch` changes some of the default arguments and shortens their names:

- `camera_name` (camera) -> `name` (hires)
- `loop_videofile` (false) -> `loop` (false)
- `visualize` (true) -> `viz` (**false**)
- `start_frame` (0) -> `start` (0)
- `stop_frame` (-1) -> `stop` (-1)
- `video_stream_provider` (0) -> `file` (`/media/sf_vm_ros/grail_20121205b_GRAILflyover20121205-1280.mp4`)


### If you have install issues:
   For some reason I couldn't build with the instructions above, so install/usage as follows is a possible workaround:
   * Delete `build/` and `devel/` directories in your catkin_ws
   * Delete the `video_stream_opencv` package in `catkin_ws/src/`
   * Install with: `sudo apt-get install ros-YOURVERSION-video-stream-opencv`
   * After a `catkin_make` and a `source ~/catkin_ws/devel/setup.bash` (if not included in `.bashrc`), a video file can be streamed with: `roslaunch video_stream_opencv camera.launch video_stream_provider:="<path/to/video.filetype>" camera_name:=hires visualize:=true`
