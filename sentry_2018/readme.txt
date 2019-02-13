#Sentry

-Sentry_Master
MCU:stm32f427(新开发板）
硬件定义：底盘电机编号0x201以及0x202
简介：这是哨兵的控制程序
工作日记：
<4/16>创建正式工程，换用stm32f427。继承了工程的程序
<4/22>完成了云台串级PID的框架，没有启用云台控制。删去了不必要的代码，现在电机的编号为0x201和0x202。FSM只有PREPARE,NORMAL以及STOP三个模式。删去了left_right_ref以及rotate_ref。
<4/24>云台PID算法完成，现在速度环/位置环以及pitch/yaw各有一个函数进行控制，pid参数在该函数中设置。已经完成了pitch轴的初步测试，包括静态，step response以及正弦测试。
<4/26>云台PID双环都需要wind-up。step response和sine response似乎有一定的互斥关系（？）需要取舍。目前是PI控制。
<4/28>解决了遥控器的bug，使用寄存器读取数据。推测是HAL库本身有bug
<4/29>哨兵YAW轴机械摩擦较大，且不规律。曲线不光滑说明物理模型经常变化。另外，yaw轴电机的CAN线容易被扯掉，需要解决。程序pitch,yaw均已完成。

