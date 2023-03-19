create a bagfiles dir
use rosbag record /topic_name
rosbag record -a -o sample
then CTRL+C


rosbag play bag_name.bag
-l for looping
-r 0.5 speed multiplicator (0.5 is slow)

for example
```
rosbag play -l -r 0.5 --clock sample_2023-03-11-21-16-59.bag 
```

to filter a rosbag to get rid of one topic (/map for example)
```
rosbag filter file.bag file-no-tf.bag "topic != '/tf' and topic != 'topic2"
```

rosbag record -a --exclude "/map" --exclude "/map_metadata"