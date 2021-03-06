#include <stm32f4xx.h>
#include "main.h"
#include "sineWave.h"
RC_Ctl_t RC_CtrlData;   //remote control data
ChassisSpeed_Ref_t ChassisSpeedRef;
Gimbal_Ref_t GimbalRef;
 FrictionWheelState_e friction_wheel_state = FRICTION_WHEEL_OFF;
static RemoteSwitch_t switch1;   //遥控器左侧拨杆
static volatile Shoot_State_e shootState = NOSHOOTING;
static InputMode_e inputmode = REMOTE_INPUT;   //输入模式设定
Shoot_Mode_e shootMode = NORMAL;
Speed_Mode_e speedMode = FAST;
Direction_e dir;
float speedRef;
float swayRef;



RampGen_t frictionRamp = RAMP_GEN_DAFAULT;  //摩擦轮斜坡
RampGen_t LRSpeedRamp = RAMP_GEN_DAFAULT;   //mouse左右移动斜坡
RampGen_t FBSpeedRamp = RAMP_GEN_DAFAULT;   //mouse前后移动斜坡

int8_t emergency = 0;
void GetRemoteSwitchAction(RemoteSwitch_t *sw, uint8_t val)
{
	static uint32_t switch_cnt = 0;

	/* 最新状态值 */
	sw->switch_value_raw = val;
	sw->switch_value_buf[sw->buf_index] = sw->switch_value_raw;

	/* 取最新值和上一次值 */
	sw->switch_value1 = (sw->switch_value_buf[sw->buf_last_index] << 2)|
	(sw->switch_value_buf[sw->buf_index]);


	/* 最老的状态值的索引 */
	sw->buf_end_index = (sw->buf_index + 1)%REMOTE_SWITCH_VALUE_BUF_DEEP;

	/* 合并三个值 */
	sw->switch_value2 = (sw->switch_value_buf[sw->buf_end_index]<<4)|sw->switch_value1;	

	/* 长按判断 */
	if(sw->switch_value_buf[sw->buf_index] == sw->switch_value_buf[sw->buf_last_index])
	{
		switch_cnt++;	
	}
	else
	{
		switch_cnt = 0;
	}

	if(switch_cnt >= 40)
	{
		sw->switch_long_value = sw->switch_value_buf[sw->buf_index]; 	
	}

	//索引循环
	sw->buf_last_index = sw->buf_index;
	sw->buf_index++;		
	if(sw->buf_index == REMOTE_SWITCH_VALUE_BUF_DEEP)
	{
		sw->buf_index = 0;	
	}			
}
//return the state of the remote 0:no action 1:action 
uint8_t IsRemoteBeingAction(void)
{
	return (fabs(ChassisSpeedRef.forward_back_ref)>=10 || fabs(ChassisSpeedRef.left_right_ref)>=10 || fabs(GimbalRef.yaw_speed_ref)>=10 || fabs(GimbalRef.pitch_speed_ref)>=10);
}

//输入模式设置 
void SetInputMode(Remote *rc)
{
	if(rc->s2 == 1)
	{
		inputmode = REMOTE_INPUT;
	}
	else if(rc->s2 == 3)
	{
		inputmode = KEY_MOUSE_INPUT;
	}
	else if(rc->s2 == 2)
	{
		inputmode = STOP;
	}	
}

InputMode_e GetInputMode()
{
	return inputmode;
}

/*
input: RemoteSwitch_t *sw, include the switch info
*/
void RemoteShootControl(RemoteSwitch_t *sw, uint8_t val) 
{
	GetRemoteSwitchAction(sw, val);
	switch(friction_wheel_state)
	{
		case FRICTION_WHEEL_OFF:
		{
			if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)   //从关闭到start turning
			{
				//SetShootState(NOSHOOTING);
				frictionRamp.ResetCounter(&frictionRamp);
				friction_wheel_state = FRICTION_WHEEL_START_TURNNING;	 
				LASER_ON(); 
			}				 		
		}break;
		case FRICTION_WHEEL_START_TURNNING:
		{
			//emergency = -1;
			if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)   //刚启动就被关闭
			{
				LASER_OFF();
				//SetShootState(NOSHOOTING);
				SetFrictionWheelSpeed(1000);
				friction_wheel_state = FRICTION_WHEEL_OFF;
				frictionRamp.ResetCounter(&frictionRamp);
			}
			else
			{
				//摩擦轮加速
				
				SetFrictionWheelSpeed(1000 + (FRICTION_WHEEL_MAX_DUTY-1000)*frictionRamp.Calc(&frictionRamp)); 
				if(frictionRamp.IsOverflow(&frictionRamp))
				{
					friction_wheel_state = FRICTION_WHEEL_ON; 	
				}
				
			}
		}break;
		case FRICTION_WHEEL_ON:
		{
			if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)   //关闭摩擦轮
			{
				LASER_OFF();
				friction_wheel_state = FRICTION_WHEEL_OFF;				  
				SetFrictionWheelSpeed(1000); 
				frictionRamp.ResetCounter(&frictionRamp);
				//SetShootState(NOSHOOTING);
			}
			else if(sw->switch_value_raw == 2)
			{
				//SetShootState(SHOOTING);
			}
			else
			{
				//SetShootState(NOSHOOTING);
			}					 
		} break;				
	}
}
	 
void MouseShootControl(Mouse *mouse)
{
//	int16_t closeDelayCount = 0;   //右键关闭摩擦轮3s延时计数
//	switch(friction_wheel_state)
//	{
//		case FRICTION_WHEEL_OFF:
//		{
//			if(mouse->last_press_r == 0 && mouse->press_r == 1)   //从关闭到start turning
//			{
//				SetShootState(NOSHOOTING);
//				frictionRamp.ResetCounter(&frictionRamp);
//				friction_wheel_state = FRICTION_WHEEL_START_TURNNING;	 
//				LASER_ON(); 
//				closeDelayCount = 0;
//			}				 		
//		}break;
//		case FRICTION_WHEEL_START_TURNNING:
//		{
//			if(mouse->press_r == 1)
//			{
//				closeDelayCount++;
//			}
//			else
//			{
//				closeDelayCount = 0;
//			}
//			if(mouse->last_press_r == 0 && mouse->press_r == 1)   //关闭摩擦轮 count was 50
//			{
//				LASER_OFF();
//				friction_wheel_state = FRICTION_WHEEL_OFF;				  
//				SetFrictionWheelSpeed(1000); 
//				frictionRamp.ResetCounter(&frictionRamp);
//				SetShootState(NOSHOOTING);
//			}
//			else
//			{
//				//摩擦轮加速				
//				SetFrictionWheelSpeed(1000 + (FRICTION_WHEEL_MAX_DUTY-1000)*frictionRamp.Calc(&frictionRamp)); 
//				if(frictionRamp.IsOverflow(&frictionRamp))
//				{
//					friction_wheel_state = FRICTION_WHEEL_ON; 	
//				}
//				
//			}
//		}break;
//		case FRICTION_WHEEL_ON:
//		{
//			if(mouse->press_r == 1)
//			{
//				closeDelayCount++;
//			}
//			else
//			{
//				closeDelayCount = 0;
//			}
//			
//			if(mouse->last_press_r == 0 && mouse->press_r == 1)   //关闭摩擦轮 count was 50
//			{
//				LASER_OFF();
//				friction_wheel_state = FRICTION_WHEEL_OFF;				  
//				SetFrictionWheelSpeed(1000); 
//				frictionRamp.ResetCounter(&frictionRamp);
//				SetShootState(NOSHOOTING);
//			}			
//			else 
      			 
	mouse->last_press_r = mouse->press_r;
}

static uint8_t mouse_r_counter = 0;
void mouse_press_r_filter(Mouse* mouse,uint8_t data)
{
	if(data==1) mouse->press_r = 1; //set press to 1 on every rising edge
	else if(mouse->last_press_r==1) mouse_r_counter++; //if r was pressed and signal is 0, increment counter for anti-shake 
	if(mouse_r_counter==5) // if 5 subsequent values are 0, set press_r to 0 
	{
		mouse->press_r = 0;
		mouse_r_counter = 0; //reset counter
	}
}
void RemoteDataPrcess(uint8_t *pData)
{
    if(pData == NULL)
    {
        return;
    }
    
    RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; 
    RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
    RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |
                         ((int16_t)pData[4] << 10)) & 0x07FF;
    RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
    
    RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
    RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);

    RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
    RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
    RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);    

    RC_CtrlData.mouse.press_l = pData[12];
		mouse_press_r_filter(&(RC_CtrlData.mouse),pData[13]);//signal is bouncing between 0 and 1 when pressed
    RC_CtrlData.key.v = ((int16_t)pData[14]);// | ((int16_t)pData[15] << 8);
	
	SetInputMode(&RC_CtrlData.rc);
	
		//
	      
	//RemoteControlProcess(&(RC_CtrlData.rc));
	Control_Mode_Switch(&(RC_CtrlData));
	Move_Control(&(RC_CtrlData));
	RemoteShootControl(&switch1, RC_CtrlData.rc.s1); // friciton wheel control
	//memorize previous input
	RC_CtrlData.key.l_v = RC_CtrlData.key.v;
	RC_CtrlData.mouse.last_press_l = RC_CtrlData.mouse.press_l;
	RC_CtrlData.mouse.last_press_r = RC_CtrlData.mouse.press_r;
	//if(GetInputMode()==REMOTE_INPUT) RemoteControlProcess(&(RC_CtrlData.rc));
	/*
	switch(GetInputMode())
	{
		case REMOTE_INPUT:
		{
			//遥控器控制模式
			RemoteControlProcess(&(RC_CtrlData.rc));
		}break;
		case KEY_MOUSE_INPUT:
		{
			//遥控器控制模式
			MouseKeyControlProcess(&RC_CtrlData.mouse,&RC_CtrlData.key);
		}break;
		case STOP:
		{
			//紧急停车
		}break;
	}
	*/		
}
//遥控器控制模式处理
void RemoteControlProcess(Remote *rc)
{
    if(GetWorkState()!=PREPARE_STATE)
    {
        ChassisSpeedRef.forward_back_ref = (RC_CtrlData.rc.ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_REF_FACT;
        ChassisSpeedRef.left_right_ref   = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_REF_FACT; 
    }

    if(GetWorkState() == NORMAL_STATE)
    {
        GimbalRef.pitch_angle_dynamic_ref += (rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_PITCH_ANGLE_INC_FACT;
        GimbalRef.yaw_angle_dynamic_ref   += (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_YAW_ANGLE_INC_FACT;      	
	}
	
	/* not used to control, just as a flag */ 
    GimbalRef.pitch_speed_ref = rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET;    //speed_ref仅做输入量判断用
    GimbalRef.yaw_speed_ref   = (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);
	GimbalAngleLimit();
	//遥控器拨杆数据处理	
	RemoteShootControl(&switch1, rc->s1);
		

}
//键盘鼠标控制模式处理
void MouseKeyControlProcess(Mouse *mouse, Key *key)
{
	static uint16_t forward_back_speed = 0;
	static uint16_t left_right_speed = 0;
    if(GetWorkState()!=PREPARE_STATE)
    {
		//speed mode: normal speed/high speed
			if(key->v & 0x80) // key:ctrl
		{
			forward_back_speed =  HIGH_FORWARD_BACK_SPEED;
			left_right_speed = HIGH_LEFT_RIGHT_SPEED;
		}
		else
		{
			forward_back_speed =  NORMAL_FORWARD_BACK_SPEED;
			left_right_speed = NORMAL_LEFT_RIGHT_SPEED;
		}
		//movement process
		if(key->v & 0x01)  // key: w
		{
			ChassisSpeedRef.forward_back_ref = forward_back_speed* FBSpeedRamp.Calc(&FBSpeedRamp);
		}
		else if(key->v & 0x02) //key: s
		{
			ChassisSpeedRef.forward_back_ref = -forward_back_speed* FBSpeedRamp.Calc(&FBSpeedRamp);
		}
		else
		{
			ChassisSpeedRef.forward_back_ref = 0;
			FBSpeedRamp.ResetCounter(&FBSpeedRamp);
		}
		
		
		if(key->v & 0x04)  // key: d
		{
			ChassisSpeedRef.left_right_ref = -left_right_speed* LRSpeedRamp.Calc(&LRSpeedRamp);
		}
		else if(key->v & 0x08) //key: a
		{
			ChassisSpeedRef.left_right_ref = left_right_speed* LRSpeedRamp.Calc(&LRSpeedRamp);
		}

		else
		{
			ChassisSpeedRef.left_right_ref = 0;
			LRSpeedRamp.ResetCounter(&LRSpeedRamp);
		}
	}
	//step2: gimbal ref calc
    if(GetWorkState() == NORMAL_STATE)
    {
		VAL_LIMIT(mouse->x, -150, 150); 
		VAL_LIMIT(mouse->y, -150, 150); 
		
        GimbalRef.pitch_angle_dynamic_ref -= mouse->y* MOUSE_TO_PITCH_ANGLE_INC_FACT;  //(rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_PITCH_ANGLE_INC_FACT;
        GimbalRef.yaw_angle_dynamic_ref   += mouse->x* MOUSE_TO_YAW_ANGLE_INC_FACT;

	}
	
	/* not used to control, just as a flag */ 
    GimbalRef.pitch_speed_ref = mouse->y;    //speed_ref仅做输入量判断用
    GimbalRef.yaw_speed_ref   = mouse->x;
	GimbalAngleLimit();	
	MouseShootControl(mouse);
	
}

//

Shoot_State_e GetShootState()
{
	return shootState;
}

Shoot_Mode_e GetShootMode()
{
	return shootMode;
}

void SetShootState(Shoot_State_e v)
{
	shootState = v;
}

FrictionWheelState_e GetFrictionState()
{
	return friction_wheel_state;
}

void SetFrictionState(FrictionWheelState_e v)
{
	friction_wheel_state = v;
}
//遥控器输入值设置，
void GimbalAngleLimit()
{
	VAL_LIMIT(GimbalRef.pitch_angle_dynamic_ref, -PITCH_MAX-10.0f, PITCH_MAX);
	VAL_LIMIT(GimbalRef.yaw_angle_dynamic_ref, GMYPositionPID.fdb - 60, GMYPositionPID.fdb + 60);
}


//遥控器数据初始化，斜坡函数等的初始化
void RemoteTaskInit()
{
	//斜坡初始化
	frictionRamp.SetScale(&frictionRamp, FRICTION_RAMP_TICK_COUNT);
	LRSpeedRamp.SetScale(&LRSpeedRamp, MOUSE_LR_RAMP_TICK_COUNT);
	FBSpeedRamp.SetScale(&FBSpeedRamp, MOUSR_FB_RAMP_TICK_COUNT);
	frictionRamp.ResetCounter(&frictionRamp);
	LRSpeedRamp.ResetCounter(&LRSpeedRamp);
	FBSpeedRamp.ResetCounter(&FBSpeedRamp);
	//底盘云台给定值初始化
	GimbalRef.pitch_angle_dynamic_ref = 0.0f;
	GimbalRef.yaw_angle_dynamic_ref = 0.0f;
	ChassisSpeedRef.forward_back_ref = 0.0f;
	ChassisSpeedRef.left_right_ref = 0.0f;
	ChassisSpeedRef.rotate_ref = 0.0f;
	//摩擦轮运行状态初始化
	SetFrictionState(FRICTION_WHEEL_OFF);
}

void Control_Mode_Switch(RC_Ctl_t* data)
{
	//SHOOT MODE control
	if(data->key.v&0x10) //shift pressed (SHOOT MODE)
	{
		shootMode = BURST;
	}
	else
	{
		shootMode = NORMAL;
	}
	
	
	if(data->mouse.press_l== 1)  //按下左键，射击
	{
		SetShootState(SHOOTING);				
	}
	else
	{
		SetShootState(NOSHOOTING);				
	}		
	//SPEED MODE control
	if((data->mouse.press_r==1)&&(data->mouse.last_press_r==0))//right mouse clicked (SPEED MODE)
	{
		if(speedMode!=SWAY)
		{
			speedMode = SWAY;
		}
		else 
		{
			speedMode = FAST;
		}
	}
	else if((data->key.v&0x20)&&(!(data->key.l_v&0x20))) //ctrl pressed (SPEED MODE)
	{
		if(speedMode!=SLOW) speedMode = SLOW;
		else speedMode = FAST;
	}
  else if(data->key.v&0x40)//SWITCH ACTION: q
	{
		if(speedMode!=DUCKING)
		  speedMode = DUCKING;
		else speedMode = FAST;
	}
	switch(speedMode)
	{
		case FAST:
		{
			speedRef = MOTOR_MAX_REF;
		}break;
		case SLOW:
		{
			speedRef = MOTOR_MAX_REF/2;
		}break;
		case SWAY:
		{
			speedRef = MOTOR_MAX_REF/2;
			//swayRef = sineWave(MOTOR_MAX_REF/2,SWAY_FREQ,(time_tick_1ms-start_time_1ms)/1000.0);
		}break;
	}
}

void updateDirection(Key* k)
{
	if((k->v&0x01)&&(k->v&0x08)) dir = LEFT_FORWARD;
	else if ((k->v&0x01)&&(k->v&0x04)) dir = RIGHT_FORWARD;
	else if((k->v&0x02)&&(k->v&0x08)) dir = LEFT_BACKWARD;
	else if ((k->v&0x02)&&(k->v&0x04)) dir = RIGHT_BACKWARD;
	else if (k->v&0x01) dir = FULL_FORWARD;
	else if (k->v&0x02) dir = FULL_BACKWARD;
	else if (k->v&0x04) dir = FULL_RIGHTWARD;
	else if (k->v&0x08) dir = FULL_LEFTWARD;
	else dir = NONE;
}

void Move_Control(RC_Ctl_t* data)
{
	//Gimbal Control
	if(GetWorkState() == NORMAL_STATE)
  {
		VAL_LIMIT(data->mouse.x, -150, 150); 
		VAL_LIMIT(data->mouse.y, -150, 150); 
		
    GimbalRef.pitch_angle_dynamic_ref -= data->mouse.y* MOUSE_TO_PITCH_ANGLE_INC_FACT;  //(rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_PITCH_ANGLE_INC_FACT;
    GimbalRef.yaw_angle_dynamic_ref   += data->mouse.x* MOUSE_TO_YAW_ANGLE_INC_FACT;

	}
	
	/* not used to control, just as a flag */ 
  GimbalRef.pitch_speed_ref = data->mouse.y;    //speed_ref仅做输入量判断用
  GimbalRef.yaw_speed_ref   = data->mouse.x;
	GimbalAngleLimit();
	
	//Chassis Contorl
	updateDirection(&(data->key));
	if(speedMode != DUCKING)
	{
		switch(dir)
		{
			case FULL_FORWARD:
				ChassisSpeedRef.forward_back_ref = speedRef;
				ChassisSpeedRef.left_right_ref = 0;
				break;
			case FULL_BACKWARD:
				ChassisSpeedRef.forward_back_ref = -speedRef;
				ChassisSpeedRef.left_right_ref = 0;
				break;
			case FULL_LEFTWARD:
				ChassisSpeedRef.forward_back_ref = 0;
				ChassisSpeedRef.left_right_ref = speedRef*0.8f;
				break;
			case FULL_RIGHTWARD:
				ChassisSpeedRef.forward_back_ref = 0;
				ChassisSpeedRef.left_right_ref = -speedRef*0.8f;
				break;
			case LEFT_FORWARD:
				ChassisSpeedRef.forward_back_ref = speedRef/2;
				ChassisSpeedRef.left_right_ref = speedRef/2;
				break;
			case RIGHT_FORWARD:
				ChassisSpeedRef.forward_back_ref = speedRef/2;
				ChassisSpeedRef.left_right_ref = -speedRef/2;
				break;
			case LEFT_BACKWARD:
				ChassisSpeedRef.forward_back_ref = -speedRef/2;
				ChassisSpeedRef.left_right_ref = speedRef/2;
				break;
			case RIGHT_BACKWARD:
				ChassisSpeedRef.forward_back_ref = -speedRef/2;
				ChassisSpeedRef.left_right_ref = -speedRef/2;
				break;
			case NONE:
				ChassisSpeedRef.forward_back_ref = 0;
				ChassisSpeedRef.left_right_ref = 0;
				break;
		}
  }
	else
	{
		switch(dir)
		{
			case RIGHT_FORWARD://originally full_foward
				ChassisSpeedRef.forward_back_ref = speedRef;
				ChassisSpeedRef.left_right_ref = 0;
				break;
			case LEFT_BACKWARD://originally full_backward
				ChassisSpeedRef.forward_back_ref = -speedRef;
				ChassisSpeedRef.left_right_ref = 0;
				break;
			case LEFT_FORWARD://originally full_leftward
				ChassisSpeedRef.forward_back_ref = 0;
				ChassisSpeedRef.left_right_ref = speedRef*0.8f;
				break;
			case RIGHT_BACKWARD://originally full_rightward
				ChassisSpeedRef.forward_back_ref = 0;
				ChassisSpeedRef.left_right_ref = -speedRef*0.8f;
				break;
			case FULL_FORWARD://originally left_foward
				ChassisSpeedRef.forward_back_ref = speedRef/2;
				ChassisSpeedRef.left_right_ref = speedRef/2;
				break;
			case FULL_RIGHTWARD://originally right foward
				ChassisSpeedRef.forward_back_ref = speedRef/2;
				ChassisSpeedRef.left_right_ref = -speedRef/2;
				break;
			case FULL_LEFTWARD://originaly left backward
				ChassisSpeedRef.forward_back_ref = -speedRef/2;
				ChassisSpeedRef.left_right_ref = speedRef/2;
				break;
			case FULL_BACKWARD://originally right backward
				ChassisSpeedRef.forward_back_ref = -speedRef/2;
				ChassisSpeedRef.left_right_ref = -speedRef/2;
				break;
			case NONE:
				ChassisSpeedRef.forward_back_ref = 0;
				ChassisSpeedRef.left_right_ref = 0;
				break;
		}
	}
}


