# rospkg
---
### 项目介绍
利用AWR1642毫米波雷达对多人进行跟踪,主要有以下两个方案:
1. 方案一
AWR1642接收到回波信号以后将数据通过UART传输到ROS节点,通过乒乓缓存将该数据保存到本地磁盘存储为csv格式,然后MATLAB通过共享文件夹形式读取到数据,对数据进行凝聚/跟踪处理,最终将数据绘制到界面上.(观察者模式)
![](https://github.com/ccpang96/rospkg/blob/master/Dcos/%E6%96%B9%E6%B3%95%E4%B8%80.png)

</br>
2. 方案二
ARM通过共享内存方式获取到DSP得到的点云数据,在ARM上进行点迹凝聚和目标跟踪处理,最终通过串口将点云数据传输到本地电脑上,matlab进行界面绘制.

![](https://github.com/ccpang96/rospkg/blob/master/Dcos/%E6%96%B9%E6%B3%95%E4%BA%8C.png)



---


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
7. ROS消息查看
```
rostopic list
rostopic echo /ti_mmwave/radar_scan
```
8. 修改AWR1642参数
```
rosparam list
rosparam get /ti_mmwave/max_doppler_vel
```

---
### 点云数据格式:
```
header: 
  seq: 6264
  stamp: 
    secs: 1538888235
    nsecs: 712113897
  frame_id: "rospkg"      #
point_id: 17              # 每一帧中探测到的点云ID(从0开始)
x: 8.650390625            # x轴 m
y: 6.92578125             # y轴 m
z: 0.0                    # z轴 m
range: 11.067276001       # 目标距离 m
velocity: 0.0             # 目标速度 m/s
doppler_bin: 8            # 多普勒速度
bearing: 38.6818885803    # 目标角度
intensity: 13.6172780991  # 目标功率 dB
```
