# need eigen
git clone https://gitlab.com/libeigen/eigen.git
# build separately
catkin build --no-deps wr_ekf_ros
# run
./devel/.private/wr_ekf_ros/lib/wr_ekf_ros/wr_ekf
