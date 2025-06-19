# 管道机器人

移植于原版的TCP通信架构的工程，readme中主要记录开发过程中遇到的问题以及开发日程

## 开发日程

### 2025.3.14

- 完成初版工程工作空间的搭建
- 移植了 `VINS-RGBD`以及 `realsense-ros`的工程
- 初步调整完善了TCP通信与手柄APP通信的 `tcp_server_node`部分模块

### 2025.3.28

- 测通与单片机的通讯
- 调整优化了recv函数中出现的字符中断不完整的问题

## 2025.3.29

- 调整规范了节点之间的消息信道
- 删除了之前定义的一些冗余消息类型
- 将后续的python收发数据节点拆分为多个可执行程序

## 2025.3.30

- 规范了所有要用到的节点通讯格式，将底层与32交互的python文件拆分为一个独立的 `single_side`类，使得底层代码简洁
- 整理清晰了消息信道定义，目前已经将所有来自于手柄的信息、32回传的信息以及控制32的接口汇聚在了主类 `MAIN_ROBOT`中
- 目前还差舵轮运动学求解、以及对机器人后续的状态机逻辑编写（需要阅读一下东电的舵轮复位操作部分的逻辑）
- **注**：已经用板子测通了32端消息的发布，打开 `single_side.py`中主函数的注释即可检验功能。

## 2025.04.15

- 初步完成了上位机状态机控制设计
- 完成了现有手柄的控制信息转义
- 待测试可用性

## 2025.05.08

- 初步完成了推杆控制的python文件与cpp文件编写，目前只差与手柄之间的type对齐
- 后续使用时在 `MAIN_ROBOT::motion_cmd_callback`函数中添加新的链接type即可

## 2025.05.27

- 加入了IMU模块的数据读取，并在主程序中添加了相关接收sub
- 卡在了IMU体坐标系轴向机器人坐标系轴转换的坐标变换求解地方，需要补一下机器人学的知识

## 2025.05.29

- 处理好了之前的坐标系问题，采用了四元数来描述夹紧后续的姿态变化过程，从而求解误差角度
- 引入了PID控制器，并做了基本检验，可以进行单侧姿态闭环PID测试了

## 2025.06.06

- 新增了基于舵轮里程的轮式里程计
- 在主程序中完成了结合前后侧夹紧状态的里程计数据自动生成
- *还没有测试是否有效*

## 2025.06.19
- 标定了轮式里程计的缩放因子
- 与现有软件联调，调通了步进与扫查功能
- 测试了在步进模式下轮式里程计的实际运动精度，结果发现在`single_side`的节点中接收时的数据本身存在跳变，也即收到的数据是在厘米级的跳动，感觉32端的数据不是很实时

## 问题解决

### 20250530

重要问题：
原来之前的python端"single side"控制器一直有问题，问题核心概括为：

```test
请阅读我这部分代码，为什么我在终端进行Debug打印时会输出这样的结果：
ID: 0 存储的舵轮速度 0.04021909087896347
ID: 1 存储的舵轮速度 0.04021909087896347
ID: 2 存储的舵轮速度 -0.04001908749341965
IP: 192.168.0.201 发送的舵轮速度 -0.04001908749341965
IP: 192.168.0.202 发送的舵轮速度 -0.04001908749341965
IP: 192.168.0.203 发送的舵轮速度 -0.04001908749341965
从存储的消息来看，ID0,1两个轮子的速度与ID2应该是不同的，但是为什么在后面用来发布数据的代码这里再次打印，ID0,1这两个轮子的速度却又都等于ID2轮子的速度
```

对应出问题的代码为这段：

```python
for i in range(self.board_num):
    self.current_cmd.dir_steer_state[i] = int(msg.dir_steer_state[i])
    self.current_cmd.dir_steer_dir[i] = float(msg.dir_steer_dir[i])
    self.current_cmd.dir_steer_vel[i] = float(msg.dir_steer_vel[i])
    # 装填到等待发送的info中
    (self.ether_info_buf[i]).MainAssistCmdName["state"] = self.current_cmd.dir_steer_state[i]
    (self.ether_info_buf[i]).MainAssistCmdName["tar_p"] = self.current_cmd.dir_steer_dir[i]
    (self.ether_info_buf[i]).MainAssistCmdName["tar_v"] = self.current_cmd.dir_steer_vel[i]
    print('ID: %s 存储的舵轮速度 %s' % (i, self.ether_info_buf[i].MainAssistCmdName["tar_v"]))
if self.board_num > 1:
    # 夹角
    self.current_cmd.dir_arm_angle[0] = float(msg.dir_arm_angle[0])
    self.current_cmd.dir_arm_angle[1] = float(msg.dir_arm_angle[1])
    self.ether_info_buf[0].MainAssistCmdName["tar_angle"] = self.current_cmd.dir_arm_angle[0]
    self.ether_info_buf[1].MainAssistCmdName["tar_angle"] = self.current_cmd.dir_arm_angle[1]
  
    # 弹簧
    self.current_cmd.dir_spring_length = float(msg.dir_spring_length)
    # 改为全部发送
    self.ether_info_buf[0].MainAssistCmdName["tar_spring"] = self.current_cmd.dir_spring_length
    self.ether_info_buf[1].MainAssistCmdName["tar_spring"] = self.current_cmd.dir_spring_length
    self.ether_info_buf[2].MainAssistCmdName["tar_spring"] = self.current_cmd.dir_spring_length
# 发送信息
self.ether_nodes_buf[0].sendTask((self.ether_info_buf[0]))
print('IP: %s 发送的舵轮速度 %s' % (self.ether_nodes_buf[0].ip,self.ether_info_buf[0].MainAssistCmdName["tar_v"]))
self.ether_nodes_buf[1].sendTask((self.ether_info_buf[1]))
print('IP: %s 发送的舵轮速度 %s' % (self.ether_nodes_buf[1].ip,self.ether_info_buf[1].MainAssistCmdName["tar_v"]))
# 如果是多板子，则发送第三个控制板
self.ether_nodes_buf[2].sendTask((self.ether_info_buf[2]))
print('IP: %s 发送的舵轮速度 %s' % (self.ether_nodes_buf[2].ip,self.ether_info_buf[2].MainAssistCmdName["tar_v"]))
```

也即之前所有的辅助驱动轮控制指令都和主动轮是**相同的**！！，这里的原因询问了Copilot，核心原因就是packInfo 类（即 self.ether_info_buf[i]）的 MainAssistCmdName 字典，三个对象其实引用的是同一个字典，即它们不是独立的，而是同一个内存对象！所以你在 for i in range(self.board_num) 里赋值时，虽然看起来是分别赋值，但实际上是给同一个字典赋值，最后的值就是最后一次循环（即ID2）的值。

解决方案其实也很简单，就是将类似这样开头的定义：

```python
class packInfo:
    MainAssistCmdName = {}
```

变更为这样：

```python
class packInfo:
    def __init__(self):
        self.MainAssistCmdName = {}
```

才会使得每个类实例化后都是独立的。

## 常用调试指令

`rostopic pub -1 /push_cmd robot_ctrl/push_board_cmd '{tar_length_f: 25.0, tar_length_b: 25.0, tar_length_m: 20.0}'`
