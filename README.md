# bmkbot
Mobile Robot Platform for Education Using ROS2 SIM &amp; REAL


### Artduino install - PC

```bash
 sudo apt-get install arduino
```

## install & build - PC & SBC

---

## YD LiDAR install

### 1. install build dependecies

```bash
sudo apt install cmake pkg-config python3 python3-pip swig -y
```

### 2. build SDK from source

```bash
cd ~
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
mkdir YDLidar-SDK/build 
cd YDLidar-SDK/build
cmake ..
make
sudo make install
```

### 3. install python API

```bash
cd ~/YDLidar-SDK
pip install .
```

### driver install

```bash
cd ~/ros2_ws/src
git clone -b humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git
cd ~/ros2_ws
colcon build --symlink-install
source install/local_setup.bash
source install/setup.bash
```

## MCU install

```bash
pip3 install pyserial 
```

## TTS install

```bash
pip3 install gtts playsound
pip3 install pydub
```

## Cartographer SLAM launch

```bash
ros2 launch bmkbot_bringup bmkbot_bringup.launch.py
```

```bash
ros2 launch bmkbot_cartographer cartographer_slam.launch.py
```

## Nav2 launch

```bash
ros2 launch bmkbot_bringup bmkbot_bringup.launch.py 
```

```bash
ros2 launch bmkbot_nav2 nav2.launch.py
```