# yhs_ros_driver
煜禾森ros版底盘驱动代码

- ysh_msgs 为ros消息协议
- yhs_zlgcan 为通过周立功的USB CAN分析仪进行解析，使用的是USBCAN-II+分析仪，周立功的使用说明可参考https://manual.zlg.cn/web/#/53?page_id=2215
- yhs_socketcan 为通过linux原生的socketcan进行解析

### 说明
- yhs_zlgcan 编译通过并实车测试
- yhs_socketcan 编译通过，未实车测试
