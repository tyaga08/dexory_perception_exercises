# lidar_data_analysis

## Overview

`lidar_data_analysis` is a ROS 2 package that provides a simple node to analyze the LiDAR problem from Dexory.

## Requirements

- ROS 2 Humble
- Python 3.10
- Suitable to run inside the `ade` environment by Dexory
- python3-pip
- matplotlib

## Installation
1. **Change the directory to the ade container setup dir, and start the container**
    
    ```bash
    cd ~/ade_dev_dir
    ade start
    ade enter

    ```

2. **Install the requirements/dependencies**
    
    ```bash
    sudo apt update
    sudo apt install python3-pip
    python3 -m pip install matplotlib
    python3 -m pip install setuptools==58.2.0  ## Required to resolve the 'setup.py install is deprecated' error while building the package
    ```

3. **Clone the repository into your ROS 2 workspace:**

    ```bash
    cd ~/ros2_ws
    git clone git@github.com:tyaga08/dexory_perception_exercises.git
    ```

4. **Build the package:**

    ```bash
    cd ~/ros2_ws/dexory_perception_exercises
    colcon build --symlink-install
    ```

4. **Source the setup script:**

    ```bash
    source install/setup.bash
    ```

## Usage

### Running the Node

In the same terminal run the `sensor_analysis_node`, use the following command:

```bash
ros2 run lidar_data_analysis sensor_analysis_node
```

### Running the ros2 bag

In a separate terminal run the ros2 bag `exercise_0.mcap`

```bash
ros2 bag play -s mcap exercise_0.mcap
```

After the ros2 bag has finished playing, exit the `sensor_analysis_node` to see the plotted values
