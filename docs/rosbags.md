create a bagfiles dir
use rosbag record /topic_name
rosbag record -a -o myfilename.bag
then CTRL+C


rosbag play bag_name.bag
-l for looping
-r 0.5 speed multiplicator (0.5 is slow)
