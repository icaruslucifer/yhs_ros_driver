# yhs_ros_driver
煜禾森ros版底盘驱动代码


### 代码结构
- ysh_msgs 为ros消息协议
- yhs_zlgcan 为通过周立功的USB CAN分析仪进行解析，使用的是USBCAN-II+分析仪，周立功的使用说明可参考https://manual.zlg.cn/web/#/53?page_id=2215
- yhs_socketcan 为通过linux原生的socketcan进行解析


### 安装及使用
- 安装：创建ros功能直接拷贝到工程src目录下即可
- 使用：
  - 使用zlgcan在终端执行：roslaunch yhs_zlgcan yhs_zlgcan.launch
  - 使用socketcan在终端执行: rosrun yhs_socketcan yhs_socketcan yhs_socketcan_node

### 说明
- yhs_zlgcan 编译通过并实车测试
- yhs_socketcan 编译通过，未实车测试
