# multi_transport
formation control for transportation

```bash
# 编译
catkin build
```

### 终端1（获取动捕）
```bash
# 环境
source devel/setup.bash

# 启动launch文件
roslaunch vrpn_client_ros sample.launch
```

### 终端2（运行代码）
```bash
# 环境
source devel/setup.bash

# 启动launch文件
rosrun global_planning v01_2cars_rrt_planning.py
```
