# Additions to ~/.bashrc: executed by bash(1) for non-login shells.
#
# Add to the end of your ~/.bashrc, or source from there.

alias dr='env | fgrep ROS'

# For Android Development Kit
#export PATH=$PATH:~/Desktop/android-sdk-linux_86/tools

## set ROS overlay name (Workspace is a symlink)
#OVR=cturtle_art
#OVR=unstable_art
#OVR=unstable_camera
OVR=Workspace
if [ -r ~/ros/$OVR/setup.bash ]
then
        export ROS_MASTER_URI=http://$HOSTNAME.local:11311

        source ~/ros/$OVR/setup.bash

        export ROS_HOME=~/.ros
        export ROSCONSOLE_CONFIG_FILE=$ROS_HOME/config/rosconsole.config

        # add art_run/bin to $PATH
        if [ $(which rospack) ]
        then ART_RUN=$(rospack find art_run 2>/dev/null)
	else ART_RUN=""
        fi
        if [ "$ART_RUN" != "" ] && [ -d $ART_RUN/bin ]
        then export PATH=$ART_RUN/bin:$PATH
        fi
fi

