#!/bin/sh

# make ROS directory, link to shared ROS install
cd
mkdir ros
cd ros
ln -s /lusr/opt/ros-28450 shared_ros
ln -s shared_ros/installed/ros .
ln -s shared_ros/installed/pkgs .

# Copy ROS environment initialization commands to stdout, these are
# normally appended to .bashrc (or to some file sourced there).  The
# EOF is quoted, so none of the lines that follow will be expanded.
cat <<'EOF'
export ROS_ROOT=$HOME/ros/ros
export PATH=$ROS_ROOT/bin:${PATH}
export PYTHONPATH=$ROS_ROOT/core/roslib/src:${PYTHONPATH} ;     
export OCTAVE_PATH=$ROS_ROOT/core/experimental/rosoct/octave:${OCTAVE_PATH}
if [ ! "$ROS_MASTER_URI" ]
then   export ROS_MASTER_URI=http://localhost:11311
fi
export ROS_PACKAGE_PATH=$HOME/ros/pkgs:$HOME/svn/utexas-art-ros-pkg

source $ROS_ROOT/tools/rosbash/rosbash        
EOF
