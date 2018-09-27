# Assignment 2

This is the third assignment for Yale's CPSC-659 Building Interactive Machines course.

## Table of Contents



## Introduction 
This assignment will provide you practical experience with custom messages in ROS, low-level image processing,
 and Kalman filters.


#### System Requirements
As for the prior assignments, you should have access to a computer with `Ubuntu 16.04` and `ROS Kinetic` to complete the homework. 

> NOTE: If you have Ubuntu 18.04, you can also complete this homework 
using ROS Melodic. 

You should also have `git` installed in the machine that you are using to work on your assignment.
You will use git to save your work to your [GitLab](http://www.gitlab.com) repository.


#### Background Knowledge

This assignment assumes that you have already completed the prior assignments and, thus, you
have set up your catkin workspace. You are also expected to have experience with Linux shells 
(e.g., [bash](https://www.gnu.org/software/bash/)), [git](https://git-scm.com/), and
the [Robot Operating System (ROS)](http://www.ros.org/). This includes being familiar with
the `roscore`, `rosrun`, `roslaunch`, `rosbag`, `rostopic`, `rosmsg`, `rosnode`, `rqt_image_view`, 
and `rviz` tools. 


#### Notation
We refer to `vectors` or column matrices with bold lower-case letters (e.g., $`\bold{x}`$).
Other `matrices`, such as linear transformations, and `scalars` are written with regular
font weight. 


#### Deliverables

- **Report:** You are expected to submit a pdf to Canvas with answers to the questions/tasks at 
the end of each part of the assignment. This report should also have any information needed 
to understand and/or run your code, as well as the specific commit SHA of the version of the code
that you would like to be evaluated on. Though not mandatory, it is recommended that you generate this pdf 
with [Overleaf](https://www.overleaf.com/edu/yale#!overview) and this 
[simple assignment template](https://www.overleaf.com/latex/templates/simple-assignment-template/mzkqqqjypzvd) 
in [LaTeX](https://www.latex-project.org/).

- **Code:** Finally, you are expected to push code for this assignment to your 
[GitLab](http://www.gitlab.com) repository as indicated in the [general instructions](../README.md) 
document for CPSC-659 assignments. 


#### Evaluation

You assignment will be evaluated based on the content of your report and your code:


- Report (20 pts + 3 extra pts)
    - Part IV (20 pts + 3 extra pts): IV-1 (4 pts) + IV-2 (4 pts) + IV-3 (4 pts) + IV-4 (2 pts) + IV-11 (3 pts) + IV-12 (3 pts) + IV-14 (3 extra pts)
- Code (75 pts + 2 extra pts)
    * Part I (10 pts) 
    * Part II (5 pts)
    * Part III (30 pts): III-1 (10 pts) + III-2 (10 pts) + III-3 (5 pts) + III-4 (5 pts)
    * Part IV (30 pts + 2 extra pts): IV-5 (6 pts) + IV-6 (6 pts) + IV-7 (4 pts) + IV-8 (2 pts) + IV-9 (2 pts) + IV-10 (5 pts) + IV-12 (2 pts) + IV-13 (3 pts) + IV-14 (2 extra pts)

#### Further Reading

- [Playing Catch and Juggling with a Humanoid Robot](https://www.disneyresearch.com/publication/playing-catch-and-juggling-with-a-humanoid-robot/):
Application of Kalman Filtering to a fun entertainment application.

- [Discriminative Training of Kalman Filters](http://www.roboticsproceedings.org/rss01/p38.pdf):
Describes systematic ways of tuning Kalman Filters given ground truth data.


## Part I. Creating a Custom ROS Message Type
ROS uses [messages](http://wiki.ros.org/msg) of various types to transmit information between nodes. For example, in the 
past you have worked [TransformStamped](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/TransformStamped.html) 
messages and [PoseStamped](http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/PoseStamped.html). Now, you 
will create your own custom message type.

To get started, read [this tutorial on Creating Messages and Services](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv) 
in ROS.
Then, follow the steps below to make your own message type as in the tutorial. This message type will serve in the next parts
 of the assignment to send information about a detected visual target.

1. Create a `msg` directory within the `shutter_track_target` ROS repository of this assignment. This directory
will hold your new message definition.

    ```bash
    $ cd shutter_track_target
    $ mkdir msg
    ```
    
2. Create a file in the shutter_track_target/msg directory named `Observation.msg`. The content of the file
should define the following fields for the message type:

    - **header** (of type [std_msgs/Header](http://docs.ros.org/api/std_msgs/html/msg/Header.html))
    - **x** (of type float64 -- built-in [primitive message type](http://wiki.ros.org/msg) --)
    - **y** (of type float64 -- built-in [primitive message type](http://wiki.ros.org/msg) --)
    
3. Edit the package.xml and CMakeLists.txt files in the shutter_track_target package to define your
new message type as in the [tutorial on Creating Messages and Services](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv).

    *Tip:* You need to add package dependencies to package.xml, as well as define your new message
    type and dependencies in CMakeLists.txt. Don't forget to uncomment the generate_messages() function
    in CMakeLists.txt as in the tutorial.

4. Build your catkin workspace to generate the new message type

    ```bash
    $ cd <catkin_workspace>
    $ catkin_make
    ```
    
    *Tip:* If for some reason catkin_make fails to generate your message, check the CMakeLists.txt and
    package.xml filtes that you edited in step 3.
    
5. Verify that your message type is built and has the 3 fields as mentioned in the step 2 above. You
can use with the `rosmsg` tool to this end:

    ```bash
    $ rosmsg show shutter_track_target/Observation
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    float64 x
    float64 y
    ```
    (the output of rosmsg should be as in the code snipped above)
    

### Questions / Tasks

- **I-1.** Add the Observation.msg, package.xml and CMakeLists.txt files to your repository.
Commit the changes that you made to generate your new message type in the shutter_track_target package.

## Part II. Get Data 
Download the ROS bag [left-seq1.bag](https://drive.google.com/open?id=1k694YrMM54QAK85AZ_a-F7jWmtWQ1TEn) from Google 
Drive. Then, place it inside a `data` directory within the shutter_track_target package:

```bash
$ cd shutter_track_target
$ mkdir data
$ cd data
$ mv <path-to-bag> . # move the bag to the data directory
```

Inspect the bag to make sure that it was downloaded properly:

```bash
$ rosbag info left-seq1.bag
path:        left-seq1.bag
version:     2.0
duration:    11.5s
start:       Sep 26 2018 21:49:02.76 (1538012942.76)
end:         Sep 26 2018 21:49:14.23 (1538012954.23)
size:        630.3 MB
messages:    239
compression: none [239/239 chunks]
types:       sensor_msgs/Image [060021388200f6f0f447d0fcd9c64743]
topics:      /virtual_camera/image_raw   239 msgs    : sensor_msgs/Image
```

And play the bag to see its content: 

```bash
$ rosparam set use_sim_time true # necessary if the use_sim_time parameter is not set in ROS or it is set to false
$ rosbag play --clock left-seq1.bag
$ rosrun rqt_image_view rqt_image_view # visualize the /virtual_camera/image_raw image
```

You should then see an image sequence of Marynel moving two colored cubes as in the figure below:

<img src="docs/left-seq1.png" width="400"/>
<br> <br>

    > ROS nodes use the /clock topic to gather information about time (see [here](http://wiki.ros.org/Clock) for more information).
    When the use_sim_time parameter is set to true, ROS will stop publishing your computer's system clock
    through /clock and, instead, rosbag play will publish a simulated time.
    If you are playing ROS bags and don't set use_sim_time parameter to true, then messages may be handled incorrectly by
    ROS nodes. You can check the value of the use_sim_time parameter in the [ROS parameter server](http://wiki.ros.org/Parameter%20Server)
    with the command:

    ```bash
    $ rosparam get use_sim_time
    ```

### Questions / Tasks
- **II-1.** Make a video of rqt_image_view that shows the content of the /virtual_camera/image_raw
topic as the left-seq1.bag rosbag plays for at least 3 seconds. Turn this video into an animated
gif, e.g., with ffmpeg and imagemagick as indicated in 
[this code snipped](https://gitlab.com/snippets/1743818), and include it in a `docs` directory within the
 shutter_track_target package. Name the gif "input.gif" and commit it to your repository.
 

## Part III. Detecting a Visual Target
You will now use simple image processing techniques to detect the blue cube in the images within the left-seq1.bag.
These detections will be observations for the location of the target, which you will then filter in the next part
of the assignment.

To get you started, this assignment provides the skeleton structure for a node that detects
blobs of a given color: see the shutter_track_target/scripts/detect_visual_target.py script. Read the script
to understand how it works in general. 

For the next parts of this assignment, do NOT edit the init() and image_callback() functions of the DetectTarget class
in the detect_visual_target.py node. Only edit the filter_image(), compute_keypoints_for_blobs(), and publish_observation()
functions as indicated in the tasks below. 

### Questions / Tasks
Complete the detect_visual_target.py node by implementing the filter_image(), compute_keypoints_for_blobs(), and publish_observation()
functions. Afterwards, make an animated video of the visual output of your node, as indicated below.

- **III-1.** Read this [tutorial on Color Spaces in OpenCV](https://www.learnopencv.com/color-spaces-in-opencv-cpp-python/).
Afterwards, implement the `filter_image()` function in the detect_visual_target.py node so that the function:

    1. Converts the input `cv_image` to the HSV color space with OpenCV.
    2. Filters the image in the HSV color space using the [cv2.inRange]() function from OpenCV. 
    3. Finally, returns the image (mask) output by the inRange() function.
    
    *Tip:* This [tutorial](https://pythonprogramming.net/color-filter-python-opencv-tutorial/) provides
    an example on color filtering in OpenCV. 
    
    To check that your filter_image() function is working properly, run your node with the default hue range of 100-140:
    
    ```bash
    # play the left-seq1.bag as in Part II of the assignment, and then run the node:
    $ rosrun shutter_track_target detect_visual_target.py
    ```
    
    You should then be able to visualize the output topic `/observation_image` for debugging purposes with 
    `rqt_image_view`. The output image (or mask) should have high values for the pixels corresponding to the blue 
    cube in the input image, as shown below.
    
    <img src="docs/blue-target.png" width="800"/><br>

- **III-2.** Read this other [tutorial on Blob Detection](https://www.learnopencv.com/blob-detection-using-opencv-python-c/)
and implement the `compute_keypoints_for_blobs()` function in the detect_visual_target.py node so that it:

    1. Creates a `cv2.SimpleBlobDetector_Params()` object with all of the parameters
    for an OpenCV blob detector.
    
    2. Creates a SimpleBlobDetector object with the parameters from the previous step.
    
    3. Uses the SimpleBlobDetector to detect blobs on the `filtered_image` mask that is input to the function.
    
    4. Returns the detected list of keypoints.
    
    *Tip:* You can read more about how the SimpleBlobDetector algorithm works on the [official OpenCV documentation](https://docs.opencv.org/2.4/modules/features2d/doc/common_interfaces_of_feature_detectors.html#SimpleBlobDetector%20:%20public%20FeatureDetector).

    Once you've implemented the compute_keypoints_for_blobs() function, you can run the detect_visual_target.py 
    node to debug your code for this part of the assignment, as in the task III-1. Edit the parameters of the 
    blob detector such that the blue cube is detected well in the left-seq1.bag
    image sequence. The result should look similar to the image below:
    
    <img src="docs/keypoint.png" width="800"/>
    
    The thin red circle in the right image above corresponds to a detected keypoint. The crossmark corresponds to the biggest keypoint
    found by OpenCV blob's detection algorithm. The position of this keypoint is what the node outputs
    through the "/observation" topic (see the next task).

- **III-3.** Finally, implement the `publish_observation()` function in the detect_visual_target.py node.
This function receives a `tuple (x,y)` corresponding to the location of the biggest keypoint 
found by the blob detection algorithm (here, only one). The function should publish this coordinate as an Observation message
through the "/observation" topic (self.obs_pub variable). 

    Before publishing the Observation message, make sure to set its 
    `header field`. The header field should have the same values as the header variable 
    that passed to the publish_observation() function. This will ensure that the time stamp and frame of the Observation message
    matches the time stamp and frame of the image that it was generated from.
    
    Once you think that your detect_visual_target.py node is properly detecting targets of a given color
    and publishing observations, commit the script to your repository.

- **III-4.** As in II-1, make a video of rqt_image_view that shows the content of the /observation_image
topic as the left-seq1.bag rosbag plays for at least 5 seconds. Turn this video into an animated
gif, name it "keypoints.gif", and include it in the `docs` directory within the
 shutter_track_target package. Commit the gif to your repository.
 

## Part IV. Filtering the Target's Position
You will now implement a linear Kalman filter to track a visual target in image space. Please read Sections 3.2.1
 and 3.2.2 from
Chapter 3 of the [Probabilistic Robotics book](http://www.probabilistic-robotics.org/) before starting with 
this assigment. The Chapter is available as a Course Reserve in the CPSC-659 Canvas website.


The `filter state` 
$`\bold{x} \in \mathbb{R}^6`$ should contain $`\bold{x} = [p_x, p_y, v_x, v_y, a_x, a_y]^T`$, 
where $`\mathbf{p} = (p_x, p_y)`$ corresponds to the estimated position of the target in an image, 
$`\mathbf{v} = (v_x, v_y)`$ is its estimated velocity, and $`\mathbf{a} = (a_x, a_y)`$ is its estimated acceleration.

The `transition model` of the filter should include additive gaussian noise and 
follow the [equations of motion](https://en.wikipedia.org/wiki/Equations_of_motion):

- $`\bold{p}_{t+1} = \bold{p}_t + \bold{v}_t \Delta t + \frac{1}{2} \bold{a} (\Delta t)^2`$
- $`\bold{v}_{t+1} = \bold{v}_t + \bold{a}_t \Delta t`$
- $`\bold{a}_{t+1} = \bold{a}_t`$

The `measurement model` of the filter should correct for the predicted state based on
the observed position of the visual target. This observation will be generated by your detect_visual_target.py
node.

You should implement your Kalman filter 
within the `kalman_filter.py` script that is inside of the shutter_track_target/scripts 
directory, as indicated in the tasks below. The kalman_filter.py script already provides you with
the main logic for a filtering node, and will help you debug your filter visually. 
 
### Questions / Tasks

- **IV-1.** Please write down the mathematical equation for the filter's linear transition model, including
Gaussian noise, in your report. Note that because you are not moving the target, but tracking will be
happening based on the observed position of the target in images, your filter will have no control $`\bold{u}`$.

- **IV-2.** Please write down the mathematical equation for the filter's measurement model, including Gaussian
noise, in your report.

- **IV-3.** Please write down the mathematical equation that defines the Kalman gain in your report.

- **IV-4.** What happens with the values in the Kalman gain if the covariance $`Q`$ for the measurement noise 
grows?

- **IV-5.** Complete the `KF_predict_step()` function at the top of the
kalman_filter.py script such that its predict a new belief for the state (encoded by its mean and covariance) based
on the prior belief and the transition model of the filter.

- **IV-6.** Complete the `KF_measurement_update_step()` function in the kalman_filter.py script such that
it corrects the belief of the state of the filter based on the latest observation and the filter's
measurement model.

- **IV-7.** Implement the `assemble_A_matrix()` and `assemble_C_matrix()` methods within the KalmanFilterNode
class of the kalman_filter.py script. The methods should set the A and C
parameters of the transition and measurement model of the filter used by the KalmanFilterNode. Use [numpy
arrays](https://docs.scipy.org/doc/numpy-1.15.1/reference/generated/numpy.array.html) to represent the A and C matrices.

    Note that you do not need to implement the logic that passes the A and C parameters to the filter. This is
    already done for you in the main loop of the KalmanFIlterNode class.
    
- **IV-8.** Implement the `initialize_process_covariance()` and `initialize_measurement_covariance()` methods
within the KalmanFilterNode class of the kalman_filter.py script. These methods should set some fixed value
for the Q and R covariances
for the noise of the transition model and measurement model, respectively. Don't worry too much about the
exact values that you set for the noise now, as you will have to tune these values later in the assignment.

- **IV-9.** Implement the `assemble_observation_vector()` function within the KalmanFilterNode class of the
kalman_filter.py script. This function should return a vector (numpy array) with the observed position for the
target. 
 
    To make this part of the assigment easy, note that the `assemble_observation_vector()` function has
    an Observation message argument. This argument provides the latest observed position of the target as received
    through the "/observation" topic in the KalmanFilterNode.

- **IV-9.** Implement the `initialize_mu_and_sigma()` method within the KalmanFilterNode class of the
kalman_filter.py script. This method should set the initial values for the filter belief based on the latest
observed target position. Again, note that this observation is passed to the `initialize_mu_and_sigma()` method
as an input argument.

- **IV-10.** Once you have finished the prior tasks, complete the filter_colored_target.launch file within the shutter_track_target/launch directory.
The launch file should run your kalman_filter.py script after playing a bag, running rqt_image_view, and 
running your detect_visual_target.py script.

    Note that the begining of the launch file already defines a set of arguments:
    
    - **bagfile:** path to the left-seq1.bag file.
    - **add_observation_noise:** whether to add artificial noise to the observed target position.
    - **lower_hue_value:** Min. hue value for the colored target that is tracked.
    - **higher_hue_value:** Max. hue value for the colored target that is tracked.
    - **playback_speed:** Speed at which to play the bag (1.0 simulates real time play back).
    
    Do NOT modify these arguments in the launch file, nor the way how the bag is launched and
    the detect_visual_target.py node is run.
    
    Once you have completed the launch file, you should be able to run it as:
    
    ```bash
    $ roslaunch shutter_track_target filter_colored_target.launch
    ```
    
    An rqt_image_view window will then open that allows you to visualize the 3 images being streamed
    through the ROS network: 
    
    - /virtual_camera/image_raw topic: the original image sequence
    - /observation_image topic: the detected target position
    - /tracked_image topic: the tracked target
    
    The images sent over the /tracked_image topic display two trajectories:
    
    <img src="docs/filtering.png" width="400"/>
 
    The red line connects the observed locations for the target (as received through the /observations topic).
    The thinner green line connects the estimated location for the target (from the Kalman Filter belief).


- **IV-11.** Tune the parameters of your filter (initial belief, R, and Q) such that you can effectively 
track the blue cube in the image sequence within the left-seq1.bag. Use the filtered_colored_target.launch
file to quickly launch rosbag, rqt_image_view, detect_visual_target.py and kalman_filter.py. As the bag plays, use
rqt_image_view to visualize the images being streamed through your network and debug your code.

    Note you can slow down the speed at which rosbag plays the image sequence with the `playback_speed`
    argument of the filtered_colored_target.launch file. For example,
    
    ```bash
    # play the bag at half real-time speed
    $ roslaunch shutter_track_target filter_colored_target.launch playback_speed:=0.5
    ```

    Once it looks like your filter is tracking the target, include the filter parameters that you are using
     for this part of the assignment in your report. 
     
- **IV-12.** Tune the parameters of your filter such that it can track the blue cube when the argument
add_observation_noise is set to true in filter_colored_target.launch:

    ```bash
    # play the bag at half real-time speed
    $ roslaunch shutter_track_target filter_colored_target.launch add_observation_noise:=true [playback_speed:=0.5]
    ```
    
    In the above command, the brackets [*] indicate optional arguments.
    
    Once it looks like your filter is tracking the target, include the filter parameters that you are using
     for this part of the assignment in your report.  In addition, make a video of rqt_image_view that shows the images sent
    through the /tracked_image topic for at least 5 seconds with add_observation_noise:=true. 
    Turn this video into an animated
    gif, name it "filtered_blue_cube_with_extra_noise.gif", and include it in the `docs` directory within the
     shutter_track_target package. Commit the gif to your repository.
     
- **IV-13.** Write a README.md Markdown file inside the shutter_track_target repository that explains
what the detect_visual_target.py and kalman_filter.py nodes do, and how the filtered_colored_target.launch script
works. To provide visual support to the explanation of how the launch file works, include the gifs that you generated
previously in this assignment in the README.md file.
   
     > More information on including images 
     in GitLab's markdown can be found [here](https://docs.gitlab.com/ee/user/markdown.html#images).
     
- **IV-14. (5 extra points)** Change the hue arguments of the filter_colored_target.launch so that you
can track the red cube in the left-seq1.bag sequence:

    ```bash
    # play the bag at half real-time speed
    $ roslaunch shutter_track_target filter_colored_target.launch lower_hue_value:=<min_value> higher_hue_value:=<max_value> [playback_speed:=0.5]
    ```
    
    You may also want to tune the parameters of your filter if tracking is not working well with the red target.
    
    Once it looks like your filter is tracking the red target, include the filter parameters that you are using
     for this part of the assignment in your report.  In addition, make a video of rqt_image_view that shows the images sent
    through the /tracked_image topic for at least 5 seconds with add_observation_noise:=true. 
    Turn this video into an animated
    gif, name it "filtered_red_cube.gif", and include it in the `docs` directory within the
     shutter_track_target package. Commit the gif to your repository.


Once you've finished the assignment, **add the commit SHA** that you would like to be evaluate on to your report.









