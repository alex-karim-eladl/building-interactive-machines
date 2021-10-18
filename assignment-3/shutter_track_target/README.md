detect_visual_target.py:
The detect_visual_target node creates a mask with just those pixels in an input video between given lower_hue and upper_hue values. It then computes the x,y location of this blob and publishes it to the /Observation topic.

kalman_filter.py:
This node subscribes to the /Observation topic to get the observed x, y values of the blob. It then uses a kalman filter to estimate the real location of the tracked object. 

filtered_colored_target.launch:
The filtered_colored_target launch script plays the bag (input video) and runs the above two nodes with the appropriate paramater which are passed as command line arguments. It also runs rqt_image_view to vizualize the input as well as the output streams of the above two nodes. 


