rosnode kill -a && killall -9 roscore rosmaster gzserver gazebo || killall -9 roscore rosmaster gzserver gazebo || echo "Nothing to kill, moving on"
kill -9 $(pgrep -f ros) 