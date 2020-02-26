# rospkg
---
### 项目介绍
利用AWR1642毫米波雷达对多人进行跟踪,主要有以下两个方案:
1. 方案一
AWR1642接收到回波信号以后将数据通过UART传输到ROS节点,通过乒乓缓存将该数据保存到本地磁盘存储为csv格式,然后MATLAB通过共享文件夹形式读取到数据,对数据进行凝聚/跟踪处理,最终将数据绘制到界面上.(观察者模式)
![](https://github.com/ccpang96/rospkg/blob/master/Dcos/%E6%96%B9%E6%B3%95%E4%B8%80.png)
2. 方案二
ARM通过共享内存方式获取到DSP得到的点云数据,在ARM上进行点迹凝聚和目标跟踪处理,最终通过串口将点云数据传输到本地电脑上,matlab进行界面绘制.
![](https://github.com/ccpang96/rospkg/blob/master/Dcos/%E6%96%B9%E6%B3%95%E4%BA%8C.png)
![](https://github.com/ccpang96/rospkg/blob/master/Dcos/%E7%A1%AC%E4%BB%B6%E5%B7%A6%E8%A7%86%E5%9B%BE.jpg)
### 使用说明(AWR1642BOOST ES2.0 EVM):
1. 搭建 AWR1642BOOST ES2.0 EVM 硬件平台,如下所示, 供电5V/2.5A ,通过USB链接到Ubuntu 18.04 LTS [ROS Melodic](http://wiki.ros.org/melodic).
![](https://github.com/ccpang96/rospkg/blob/master/Dcos/%E7%A1%AC%E4%BB%B6%E4%B8%BB%E8%A7%86%E5%9B%BE1.png)
2. 从[MMWAVE-SDK](http://www.ti.com/tool/MMWAVE-SDK) 下载 SDK 2.0并使用[UNIFLASH](http://www.ti.com/tool/UNIFLASH)烧录 xwr16xx_mmw_demo.bin.
3. 将本仓库和ROS serial库下载到<workspace dir>/src`:
```
git clone https://github.com/radar-lab/ti_mmwave_rospkg.git
git clone https://github.com/wjwwood/serial.git
```
4. 进入`<workspace dir>`:

```
catkin_make && source devel/setup.bash
echo "source <workspace_dir>/devel/setup.bash" >> ~/.bashrc
```

5. 使能串口:
```
sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyACM1
```
6. 配置AWR1642探测模式:
```
roslaunch ti_mmwave_rospkg 1642es2_short_range.launch
```
7. ROS topics can be accessed as follows:
```
rostopic list
rostopic echo /ti_mmwave/radar_scan
```
8. ROS parameters can be accessed as follows:
```
rosparam list
rosparam get /ti_mmwave/max_doppler_vel
```

Note: AWR1843 requires SDK 3.2.0.4, which has different output format. The later release will improve this part.

---
### Message format:
```
header: 
  seq: 6264
  stamp: 
    secs: 1538888235
    nsecs: 712113897
  frame_id: "ti_mmwave"   # Frame ID used for multi-sensor scenarios
point_id: 17              # Point ID of the detecting frame (Every frame starts with 0)
x: 8.650390625            # Point x coordinates in m (front from antenna)
y: 6.92578125             # Point y coordinates in m (left/right from antenna, right positive)
z: 0.0                    # Point z coordinates in m (up/down from antenna, up positive)
range: 11.067276001       # Radar measured range in m
velocity: 0.0             # Radar measured range rate in m/s
doppler_bin: 8            # Doppler bin location of the point (total bins = num of chirps)
bearing: 38.6818885803    # Radar measured angle in degrees (right positive)
intensity: 13.6172780991  # Radar measured intensity in dB
```
---
### Troubleshooting
1.
```
mmWaveCommSrv: Failed to open User serial port with error: IO Exception (13): Permission denied
mmWaveCommSrv: Waiting 20 seconds before trying again...
```
This happens when serial port is called without superuser permission, do the following steps:
```
sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyACM1
```
2.
```
mmWaveQuickConfig: Command failed (mmWave sensor did not respond with 'Done')
mmWaveQuickConfig: Response: 'sensorStop
'?`????`????`???~' is not recognized as a CLI command
mmwDemo:/>'
```
When this happens, re-run the command you send to sensor. If it continues, shut down and restart the sensor.

---
### Multiple devices support (dual AWR1642 ES2.0 EVM):
1. Connect two devices and try `ll /dev/serial/by-id` or `ls /dev`. In this case, `/dev/ttyACM0` to `/dev/ttyACM3` should shown.
2. To avoid serial port confliction, you need to launch devices separately. So for first device (it will open rviz):

```
roslaunch ti_mmwave_rospkg multi_1642_0.launch 
```
3. Change radars' location in first three arguments `<node pkg="tf" type="static_transform_publisher" name="radar_baselink_0" args="0 -1 0 0 0 0 ti_mmwave_pcl ti_mmwave_0 100"/>` (stands for x,y,z for positions) in launch file `multi_1642_1.launch`. And launch second device:

```
roslaunch ti_mmwave_rospkg multi_1642_1.launch 
```

Note: As serial connection and the original code, you need to launch devices separately using different launch files.

---
### Camera overlay support (working with USB camera or CV camera):
1. Download and build USB camera repo [here](https://github.com/radar-lab/usb_webcam`). And set parameters of camera in `<usb_webcam dir>/launch/usb_webcam.launch`.
2. To test the device image working, try:
```
roslaunch usb_webcam usb_webcam.launch
rosrun rqt_image_view rqt_image_view  
```
3. Make sure you have done [ROS camera calibration](http://wiki.ros.org/camera_calibration) and create a `*.yaml` configuration file accordingly.
4. Launch radar-camera system using:
```
roslaunch ti_mmwave_rospkg camera_overlay.launch
```

---
### Changelog:

```
v3.3.0

Add support for XWR18XX devices. SDK version: 3.2.0.4.

v3.2.2
Fix bugs and update README.

v3.2.1
Support camera overlay over 3D 1443s.

v3.2.0
Added camera overlay support.

v3.1.0
Strengthened code.

v3.0.0
Added README.
Improved rviz looking for point cloud data.
Added support for multiple radars working together. 
Improved radar's all around working conditions.

v2.0.0
Added support for ES2.0 EVM devices.

v1.0.0
Added Doppler from TI's mmWave radars.
```
