# ros-summer-school-2019
激光雷达环境识别挑战赛--实现基本要求

by Andyoyo@swust

### 1 下载 编译 运行

`git clone https://github.com/AndyoyoZ/ros-summer-school-2019.git`

`cd ros-summer-school-2019`

`catkin_make`

`source devel/setup.bash`

`rosrun shap_analysis shap_analysis_node`

### 2 回放bag包数据

ros-summer-school-2019/src/shap_analysis/rosbag目录下放有三个测试用的bag包，回放bag包数据进行测试，按空格可暂停回放数据

`rosbag play circle.bag`

### 3 效果

在终端输出形状

![circle](https://github.com/AndyoyoZ/ros-summer-school-2019/blob/master/screenshot/1.png) 

![rectangle](https://github.com/AndyoyoZ/ros-summer-school-2019/blob/master/screenshot/2.png) 

![triangle](https://github.com/AndyoyoZ/ros-summer-school-2019/blob/master/screenshot/3.png) 
