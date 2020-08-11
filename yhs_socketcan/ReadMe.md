

#### 安装
- enable kernel module: gs_usb
> sudo modprobe gs_usb

- bring up can interface
> sudo ip link set can0 up type can bitrate 500000

- install can utils
> sudo apt install -y can-utils


#### 设置
- sudo ip link set can0 up type can bitrate 500000


 使用 socketcan 接口进行 IO

 配置激活 can:
 sudo ip link set can0 up type can bitrate 500000 restart-ms 10
 比特率可以根据实际进行测试，在 socketcan 接口中不能修改波特率
 所以需要在系统启动时候，且CAN 连接的时候进行配置

 关闭 can
 sudo ip link set can0 down

 测试发送: cansend can0 123#11223344
 测试接收: candump can0