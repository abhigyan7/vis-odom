# Visual Odometry

This is a simple visual odometry pipeline that estimates the relative 
pose between two monocular frames using a fundamental matrix. The goal
is for this pipeline to develop full-fledged monocular SLAM capabilities
with support for sensors such as inertial measurement units and global 
positioning. 

### Build
I build the code against the latest versions of OpenCV, Ceres solver and 
glog that's available in archlinux. I use pangolin v0.6 due to a bug in 
the later versions thatI haven't gotten around to fixing, but I will look 
into it soon.

- Clone the repo.
- `mkdir build && cd build && cmake .. && make main`
- Hope you are lucky.

### Running

The build emits the binary `main`, to which you pass in the following
arguments.

- video file

  This is any valid string that can be passed to OpenCV's VideoCapture. To
  use a directory of images, use OpenCV's globbing style, like so:
  ```/mnt/datasets/kitti-odometry/seq0/dataset/sequences/00/image_0/%6d.png```
  
- frame_id

  How many frames to process before stopping
  
- f

  Focal length in pixel units
  
- cx cy

  Image center coordinates in the image
  
- n_features

  Unused, can be used to set the number of features to detect before matching
  
- poses_file 

  A file containing ground truth poses formatted like the KITTI ground truths. 
  It is optional. If provided, estimated translations are scaled to match the
  data and ground truth trajectory is shown in the Pangolin display.

