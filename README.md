# Installation

## Make a new workspace
```bash
mkdir -p percep_ws/src
```

## Clone the ROS-Perception-Pipeline repository

Now go ahead and clone this repository inside the "src" folder of the workspace you just created.

```bash
cd percep_ws/src

git clone git@github.com:atom-robotics-lab/ros-perception-pipeline.git
```

## Make the package

We'll need to "make" everything in our catkin workspace so that the ROS environment knows about our new package. (This will also compile any necessary code in the package). Execute the given commands in your terminal.

```bash
colcon build --symlink-install
```

Now you will need to source your workspace
```bash
source install/local_setup.bash
```

## Usage

<br>

### 1. Launch the Playground simulation
We have made a demo playground world to test our pipeline. To launch this world, follow the steps 
given below

```bash
ros2 launch perception_bringup playground.launch.py 
```
The above command will launch the playground world as shown below :

<img src = "assets/gazebo.png" width = 800>
<br>

Don't forget to click on the **play** button on the bottom left corner of the Ignition Gazebo window

<br>

### 2. Launch the Object Detection node
<br>

Use the pip install command as shown below to install the required packages.
```bash
pip install -r src/ros-perception-pipeline/object_detection/requirements.txt
```

Use the command given below to run the ObjectDetection node. Remember to change the path of the object_detection.yaml
file according to your present working directory

```bash 
ros2 run object_detection ObjectDetection --ros-args --params-file src/ros-perception-pipeline/object_detection/config/object_detection.yaml
```

### 3. Changing the Detector

To change the object detector being used, you can change the parameters inside the object_detection.yaml file location inside 
the **config** folder. 

<img src = "assets/config.png" width = 500>
<br>

## Testing

Now to see the inference results, open a new terminal and enter the given command

```bash
ros2 run rqt_image_view rqt_image_view
```

<img src = "assets/rqt_yolov8.png" width = 800>
<br>



