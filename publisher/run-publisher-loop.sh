until ((roslaunch ndn_utils ros2ndn_converter.launch)); do 
	echo "ndn-opt publisher exited with code $?, respawning">&2 
	sleep 1 
done
echo "ndn-opt publisher exited with code $?"
