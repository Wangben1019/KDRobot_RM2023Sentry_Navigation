# ROS开机自启

## 一.安装robot_upstart

```shell
sudo apt install ros-noetic-robot-upstart # 安装robot_upstart功能包，根据自己版本来

# 如果缺少依赖
rosdep install robot_upstart
```

## 二.准备好需要运行的roslaunch文件

将所有需要启动的ros节点集中到一个launch文件中

```xml
<launch>
    <include file="$(find livox_ros_driver2)/launch/msg_MID360.launch" />

    <include file="$(find fast_lio)/launch/mapping_mid360.launch" />

    <include file="$(find vehicle_simulator)/launch/system_real_robot.launch" />

    <node pkg="user_package" type="control_chassis" name="control_chassis" output="screen" />

</launch>
```

他的位置为：

```shell
/home/wang/demo_sentry_ws/src/user_package/launch/Start_Robot.launch
```

## 三.装载launch文件

由于robot_upstart是ROS节点，启动前需要先开启roscore。

```shell
roscore
```

随后装载launch文件

```shell
rosrun robot_upstart install user_package/launch/Start_Robot.launch --job sentry --interface wlan0
```

* install后接launch文件所在的功能包路径。注意：路径只需要从该功能包开始即可，不要经过绝对路径，也不要经过从工作区顶部文件夹等
* --job指定任务的别名
* --interface指定网络接口，一定要设置，不然无法实现多机ROS通信

## 四.启动、重启、停止任务

### 启动任务

```shell
sudo systemctl daemon-reload && sudo systemctl start sentry
```

* sentry为上文设置的任务别名
* 本句命令会在运行完装载launch文件命令后给出提示

```shell
wang@wang:~$ rosrun robot_upstart install user_package/launch/Start_Robot.launch --job sentry --interface wlan0
b'/usr/lib/systemd/systemd'
Preparing to install files to the following paths:
  /etc/ros/noetic/sentry.d/.installed_files
  /etc/ros/noetic/sentry.d/Start_Robot.launch
  /etc/systemd/system/multi-user.target.wants/sentry.service
  /lib/systemd/system/sentry.service
  /usr/sbin/sentry-start
  /usr/sbin/sentry-stop
Now calling: /usr/bin/sudo /opt/ros/noetic/lib/robot_upstart/mutate_files
[sudo] wang 的密码： 
Filesystem operation succeeded.
** To complete installation please run the following command:
 sudo systemctl daemon-reload && sudo systemctl start sentry

```

### 重启任务

```shell
sudo systemctl restart sentry
```

### 停止任务

```shell
sudo systemctl stop sentry
```

## 卸载自启动任务

由于robot_upstart是ROS节点，启动前需要先开启roscore。

```shell
roscore
```

随后进行下一步操作

### 卸载

```shell
rosrun robot_upstart uninstall sentry
```

### 删除日志

```shell
sudo rm -rf ~/myrobot.log
```

* 在装载launch哪一步是可以添加日志存储位置的 --logdir 后接日志存放目录。



## python实现

在pkg user_package/scripts下