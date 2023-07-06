<!-- Improved compatibility of back to top link: See: https://github.com/othneildrew/Best-README-Template/pull/73 -->
<a name="readme-top"></a>
<!--
*** Thanks for checking out the Best-README-Template. If you have a suggestion
*** that would make this better, please fork the repo and create a pull request
*** or simply open an issue with the tag "enhancement".
*** Don't forget to give the project a star!
*** Thanks again! Now go create something AMAZING! :D
-->


<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]



<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://github.com/atom-robotics-lab/assets/blob/main/logo_1.png?raw=true">
    <img src="https://github.com/atom-robotics-lab/assets/blob/main/logo_1.png?raw=true" alt="Logo" width="120" height="120">
  </a>

  <h3 align="center">Best-README-Template</h3>

  <p align="center">
    This is the repo focus on the development of a perception pipeline for the  <a href="https://github.com/atom-robotics-lab/MR-Robot">MR-Robot: ModulaR Robot</a> Project, Mr robot is autonomous navigation robot made by A.T.O.M Robotics capable of doing multiple day to day operations such as mapping, navigation for transportation, sanitaion etc. Various other operations can also be performed thanks to its modularity.
    If you’re interested in helping to improve our Project</a>, find out how to <a href="https://github.com/atom-robotics-lab/MR-Robot/blob/main/contributing.md">contribute<a>.
    <br />
    <br />
    <br />
    <a href="https://github.com/atom-robotics-lab/ros-perception-pipeline/issues/new?labels=bug&assignees=jasmeet0915,Kartik9250,insaaniManav,namikxgithub">Report Bug</a>
    ·
    <a href="https://github.com/atom-robotics-lab/ros-perception-pipeline/issues/new?labels=enhancement&assignees=jasmeet0915,Kartik9250,insaaniManav,namikxgithub">Request Feature</a>
  </p>
</div>



<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a>
        <ul>
        <li><a href="#testing">Testing</a></li>
      </ul>
    </li>
    <!--<li><a href="#roadmap">Roadmap</a></li>-->
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

The perception pipeline project aims to develop a robust and efficient system for extracting meaningful information from sensor data in order to enable autonomous robots, such as the MR-Robot, to understand and interact with their environment. By incorporating advanced techniques like object detection using deep learning models, the perception pipeline enhances the robot's ability to detect and identify objects, enabling it to perform tasks such as mapping, navigation, and sanitation. This project focuses on creating a modular and adaptable perception pipeline that can be customized and expanded to suit different robotic applications, thereby paving the way for more intelligent and capable autonomous systems.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Built With

* [![ROS](https://img.shields.io/badge/ros-%230A0FF9.svg?style=for-the-badge&logo=ros&logoColor=white)](https://www.sphinx-docs.org)
* [![OpenCV](https://img.shields.io/badge/opencv-%23white.svg?style=for-the-badge&logo=opencv&logoColor=white)](https://opencv.org/)
* [![Blender](https://img.shields.io/badge/blender-%23F5792A.svg?style=for-the-badge&logo=blender&logoColor=white)](https://www.blender.org/)
* [![Raspberry Pi](https://img.shields.io/badge/-RaspberryPi-C51A4A?style=for-the-badge&logo=Raspberry-Pi)](https://www.raspberrypi.org/)
* [![Espressif](https://img.shields.io/badge/espressif-E7352C?style=for-the-badge&logo=espressif&logoColor=white)](https://www.espressif.com/)
* [![Arduino](https://img.shields.io/badge/Arduino-00979D?style=for-the-badge&logo=Arduino&logoColor=white)](https://www.arduino.cc/)
* [![Ubuntu](https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white)](https://ubuntu.com/)
* [![Python](https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://www.python.org/)

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- GETTING STARTED -->
## Getting Started

This is an example of how you may give instructions on setting up your project locally.
To get a local copy up and running follow these simple example steps.

### Prerequisites

This is an example of how to list things you need to use the software and how to install them.
* Ros
  - Refer to our [Ros installation guide](https://atom-robotics-lab.github.io/wiki/markdown/ros/installation.html)
  - Installing Navigation specific dependencies: map-server, move_base and amcl
    ```sh
    sudo apt install ros-noetic-map-server ros-noetic-move-base ros-noetic-amcl
    ```

* Opencv
  ```sh
  sudo apt install libopencv-dev python3-opencv
  ```

### Installation

1. Make a new workspace
```bash
mkdir -p percep_ws/src
```

2. Clone the ROS-Perception-Pipeline repository

Now go ahead and clone this repository inside the "src" folder of the workspace you just created.

```bash
cd percep_ws/src

git clone git@github.com:atom-robotics-lab/ros-perception-pipeline.git
```

3. Make the package

We'll need to "make" everything in our catkin workspace so that the ROS environment knows about our new package. (This will also compile any necessary code in the package). Execute the given commands in your terminal.

```bash
colcon build --symlink-install
```

Now you will need to source your workspace
```bash
source install/local_setup.bash
```



<!-- USAGE EXAMPLES -->
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
<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- ROADMAP 
## Roadmap

- [x] Alpha version
- [x] Version 1
    - [x] Adding camera
    - [x] Adding 3d camera
    - [ ] Hardware prototype

See the [open issues](https://github.com/atom-robotics-lab/MR-Robot/issues) for a full list of proposed features (and known issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p>

ROADMAP ?? -->



<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open-source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- LICENSE -->
## License

Distributed under the MIT License. See `LICENSE` for more information.

<br />

<!-- CONTACT -->
## Contact

Our Socials - [Linktree](https://linktr.ee/atomlabs) - atom@inventati.org

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

* [Our wiki](https://atom-robotics-lab.github.io/wiki)
* [ROS Official Documentation](http://wiki.ros.org/Documentation)
* [Opencv Official Documentation](https://docs.opencv.org/4.x/)
* [Rviz Documentation](http://wiki.ros.org/rviz)
* [Gazebo Tutorials](https://classic.gazebosim.org/tutorials)
* [Ubuntu Installation guide](https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview)
* [Raspberrypi Documentation](https://www.raspberrypi.com/documentation/)
* [Esp32 Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/)
* [Blender Documentaion](https://docs.blender.org/)
* [YOLOv3](https://arxiv.org/abs/1804.02767)
* [YOLOv4](https://arxiv.org/abs/2004.10934)

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/atom-robotics-lab/MR-Robot.svg?style=for-the-badge
[contributors-url]: https://github.com/atom-robotics-lab/MR-Robot/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/atom-robotics-lab/MR-Robot.svg?style=for-the-badge
[forks-url]: https://github.com/atom-robotics-lab/wiki/network/members
[stars-shield]: https://img.shields.io/github/stars/atom-robotics-lab/MR-Robot.svg?style=for-the-badge
[stars-url]: https://github.com/atom-robotics-lab/wiki/stargazers
[issues-shield]: https://img.shields.io/github/issues/atom-robotics-lab/MR-Robot.svg?style=for-the-badge
[issues-url]: https://github.com/atom-robotics-lab/MR-Robot/issues
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://www.linkedin.com/company/a-t-o-m-robotics-lab/