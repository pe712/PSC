# Introduction

This is a quick introduction to rosbags and how to use them. Rosbags are used to record every message sent to certain topics during a ros execution. Thanks to that record, it is possible to run the exact same situation to understand the issue.

You can have more detailled information about rosbags commands [here](http://wiki.ros.org/rosbag/Commandline) and some advanced examples [here](http://wiki.ros.org/rosbag/Cookbook)

# Recording rosbags

* create a bagfiles dir and change directory to this one
* use `rosbag record /topic_name` and then press `CTRL+C` to stop. 
  option -a is for recording every topic
  option -o sample to save as sample.bag

```
rosbag record -a -o sample
```

# Playing rosbags

* rosbag play bag_name.bag
  option -l for looping
  option -r 0.5 to change speed multiplicator (0.5 is slow)

```
rosbag play -l -r 0.5 --clock sample_2023-03-11-21-16-59.bag 
```

# Filtering rosbags
This is handy to get rid of one topic (/map for example when trying to use a cartographer package). We have tried two different approches. However, it didn't worked and we didn't managed to figure why (it has been done from a f1tenth simulation).

* filtering an existing rosbag
```
rosbag filter file.bag file-no-tf.bag "topic != '/topic1' and topic != 'topic2"
```

* recording everything except some topics
```
rosbag record -a --exclude "/topic1" --exclude "/topic2"
```


