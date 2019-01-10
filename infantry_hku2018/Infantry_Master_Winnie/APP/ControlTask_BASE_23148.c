#include "main.h"

PID_Regulator_t GMPPositionPID = GIMBAL_MOTOR_PITCH_POSITION_PID_DEFAULT;     
PID_Regulator_t GMPSpeedPID = GIMBAL_MOTOR_PITCH_SPEED_PID_DEFAULT;
PID_Regulator_t GMYPositionPID = GIMBAL_MOTOR_YAW_POSITION_PID_DEFAULT;			
PID_Regulator_t GMYSpeedPID = GIMBAL_MOTOR_YAW_SPEED_PID_DEFAULT;

PID_Regulator_t CMRotatePID = CHASSIS_MOTOR_ROTATE_PID_DEFAULT; 
PID_Regulator_t CM1SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM2SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM3SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM4SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;

PID_Regulator_t ShootMotorPositionPID = SHOOT_MOTOR_POSITION_PID_DEFAULT;      //shoot motor
PID_Regulator_t ShootMotorSpeedPID = SHOOT_MOTOR_SPEED_PID_DEFAULT;

/*--------------------------------------------CTRL Variables----------------------------------------*/
WorkState_e lastWorkState = PREPARE_STATE;
WorkState_e workState = PREPARE_STATE;
ShootMotorState_e shooterState = SHOOT_PREPARE;
RampGen_t GMPitchRamp = RAMP_GEN_DAFAULT;
RampGen_t GMYawRamp = RAMP_GEN_DAFAULT;
CM_Speed_Ref CM_sref;
extern int8_t emergency;

/*
*********************************************************************************************************
*                                            FUNCTIONS 
*********************************************************************************************************
*/

static void SetWorkState(WorkState_e state)
{
    workState = state;
}

WorkState_e GetWorkState()
{
	return workState;
}

ShootMotorState_e GetShootMotorState()
{
	return shooterState;
}

//���̿�������
static float CM1Speed_l_output=0;
static float CM2Speed_l_output=0;
static float CM3Speed_l_output=0;
static float CM4Speed_l_output=0;
void CMControlLoop(void)
{  
	//������ת������
	if(GetWorkState()==PREPARE_STATE) //�����׶Σ����̲���ת
	{
		ChassisSpeedRef.rotate_ref = 0;	 
	}
	else
	{
		 //���̸����������תPID����
		 CMRotatePID.ref = 0;
		 CMRotatePID.fdb = GMYawEncoder.ecd_angle;
		 CMRotatePID.Calc(&CMRotatePID);   
		 ChassisSpeedRef.rotate_ref = CMRotatePID.output;
	}
	if(Is_Lost_Error_Set(LOST_ERROR_RC))      //���ң������ʧ��ǿ�ƽ��ٶ��趨ֵreset
	{
		ChassisSpeedRef.forward_back_ref = 0;
		ChassisSpeedRef.left_right_ref = 0;
	}
/*
	CM1SpeedPID.ref =  (-ChassisSpeedRef.forward_back_ref*0.075 + ChassisSpeedRef.left_right_ref*0.075 + ChassisSpeedRef.rotate_ref)*25;
	CM2SpeedPID.ref = (ChassisSpeedRef.forward_back_ref*0.075 + ChassisSpeedRef.left_right_ref*0.075 + ChassisSpeedRef.rotate_ref)*25;
	CM3SpeedPID.ref = (ChassisSpeedRef.forward_back_ref*0.075 - ChassisSpeedRef.left_right_ref*0.075 + ChassisSpeedRef.rotate_ref)*25;
	CM4SpeedPID.ref = (-ChassisSpeedRef.forward_back_ref*0.075 - ChassisSpeedRef.left_right_ref*0.075 + ChassisSpeedRef.rotate_ref)*25;
*/
	CM_sref.CM1= (-ChassisSpeedRef.forward_back_ref + ChassisSpeedRef.left_right_ref + ChassisSpeedRef.rotate_ref*10);
	CM_sref.CM2 = (ChassisSpeedRef.forward_back_ref + ChassisSpeedRef.left_right_ref + ChassisSpeedRef.rotate_ref*10);
	CM_sref.CM3 = (ChassisSpeedRef.forward_back_ref - ChassisSpeedRef.left_right_ref + ChassisSpeedRef.rotate_ref*10);
	CM_sref.CM4 = (-ChassisSpeedRef.forward_back_ref - ChassisSpeedRef.left_right_ref + ChassisSpeedRef.rotate_ref*10);
	
	if(fabs(CM_sref.CM1)>600||fabs(CM_sref.CM2)>600||fabs(CM_sref.CM3)>600||fabs(CM_sref.CM4)>600)//600 corresponds to roughly 1.5m/s
	{
		float offset = ChassisSpeedRef.rotate_ref;
		CM_sref.CM1 -= offset;
		CM_sref.CM2 -= offset;
		CM_sref.CM3 -= offset;
		CM_sref.CM4 -= offset;
	} // To speed up the robot.
	
	CM1SpeedPID.ref = CM_sref.CM1;
	CM2SpeedPID.ref = CM_sref.CM2;
	CM3SpeedPID.ref = CM_sref.CM3;
	CM4SpeedPID.ref = CM_sref.CM4; 
	
	
/*
	CM1SpeedPID.ref =  -ChassisSpeedRef.forward_back_ref*0.075 + ChassisSpeedRef.left_right_ref*0.075;
	CM2SpeedPID.ref = ChassisSpeedRef.forward_back_ref*0.075 + ChassisSpeedRef.left_right_ref*0.075;
	CM3SpeedPID.ref = ChassisSpeedRef.forward_back_ref*0.075 - ChassisSpeedRef.left_right_ref*0.075;
	CM4SpeedPID.ref = -ChassisSpeedRef.forward_back_ref*0.075 - ChassisSpeedRef.left_right_ref*0.075;
*/
	CM1SpeedPID.fdb = CM1Encoder.filter_rate;
	CM2SpeedPID.fdb = CM2Encoder.filter_rate;
	CM3SpeedPID.fdb = CM3Encoder.filter_rate;
	CM4SpeedPID.fdb = CM4Encoder.filter_rate;
	
	CM1SpeedPID.Calc(&CM1SpeedPID);
	CM2SpeedPID.Calc(&CM2SpeedPID);
	CM3SpeedPID.Calc(&CM3SpeedPID);
	CM4SpeedPID.Calc(&CM4SpeedPID);
	
		
	
	if(CM1SpeedPID.output-CM1Speed_l_output>800||CM1SpeedPID.output-CM1Speed_l_output<-800)
	{
		if(CM1SpeedPID.output-CM1Speed_l_output>800) CM1SpeedPID.output = CM1Speed_l_output + 800;
		else CM1SpeedPID.output = CM1Speed_l_output - 800;
	}
	if(CM2SpeedPID.output-CM2Speed_l_output>800||CM2SpeedPID.output-CM2Speed_l_output<-800)
	{
		if(CM2SpeedPID.output-CM2Speed_l_output>800) CM2SpeedPID.output = CM2Speed_l_output + 800;
		else CM2SpeedPID.output = CM2Speed_l_output - 800;
	}
	if(CM3SpeedPID.output-CM3Speed_l_output>800||CM3SpeedPID.output-CM3Speed_l_output<-800)
	{
		if(CM3SpeedPID.output-CM3Speed_l_output>800) CM3SpeedPID.output = CM3Speed_l_output + 800;
		else CM3SpeedPID.output = CM3Speed_l_output - 800;
	}
	if(CM4SpeedPID.output-CM4Speed_l_output>800||CM4SpeedPID.output-CM4Speed_l_output<-800)
	{
		if(CM4SpeedPID.output-CM4Speed_l_output>800) CM4SpeedPID.output = CM4Speed_l_output + 800;
		else CM4SpeedPID.output = CM4Speed_l_output - 800;
	}
	
	 if((GetWorkState() == STOP_STATE) ||Is_Serious_Error() || GetWorkState() == CALI_STATE || GetWorkState() == PREPARE_STATE)    //|| dead_lock_flag == 1����ͣ����������У׼���޿�������ʱ����ʹ���̿���ֹͣ
	 {
		 Set_CM_Speed(CAN2, 0,0,0,0);
	 }
	 else
	 {
		 //Set_CM_Speed(CAN2, CHASSIS_SPEED_ATTENUATION * CM1SpeedPID.output, CHASSIS_SPEED_ATTENUATION * CM2SpeedPID.output, CHASSIS_SPEED_ATTENUATION * CM3SpeedPID.output, CHASSIS_SPEED_ATTENUATION * CM4SpeedPID.output);	
		 //Set_CM_Speed(CAN2,-ChassisSpeedRef.forward_back_ref*100,ChassisSpeedRef.forward_back_ref*100,ChassisSpeedRef.forward_back_ref*100,-ChassisSpeedRef.forward_back_ref*100);
		 //Set_CM_Speed(CAN2, CHASSIS_SPEED_ATTENUATION * CM1SpeedPID.ref*100, CHASSIS_SPEED_ATTENUATION * CM2SpeedPID.ref*100, CHASSIS_SPEED_ATTENUATION * CM3SpeedPID.ref*100, CHASSIS_SPEED_ATTENUATION * CM4SpeedPID.ref*100);
		 //Set_CM_Speed(CAN2, CHASSIS_SPEED_ATTENUATION * CM1SpeedPID.output, 0,0,0);
	 } 
	CM1Speed_l_output = CM1SpeedPID.output;
	CM2Speed_l_output = CM2SpeedPID.output;
	CM3Speed_l_output = CM3SpeedPID.output;
	CM4Speed_l_output = CM4SpeedPID.output;
	 
}
//�����������������
int16_t pwm_ccr = 0;
int16_t shoot_timer = 0;//shoot cycle: approx 1000ms per cycle
//ShootMotorState_e shooterState = PREPARE;SHOOTING;COOLING

void ShooterMControl(void)	
{	 
	ShootMotorSpeedPID.fdb = GetQuadEncoderDiff();
	ShootMotorPositionPID.fdb -= (float)ShootMotorSpeedPID.fdb*360.0f/5041.0f;// Reduction ratio 13:1
	ShootMotorPositionPID.Calc(&ShootMotorPositionPID);
	if(shooterState == SHOOT_SHOOTING)
		ShootMotorSpeedPID.ref =  ShootMotorPositionPID.output;
	else
		ShootMotorSpeedPID.ref = 0;
	ShootMotorSpeedPID.Calc(&ShootMotorSpeedPID);
	PWM3 = ShootMotorSpeedPID.output;	
}

void ShooterMControlLoop(void)
{
	if(shooterState == SHOOT_PREPARE && GetShootState() == SHOOTING)
	{
		shooterState = SHOOT_SHOOTING;
		if(GetShootMode() == BURST)
			ShootMotorPositionPID.ref += 180;
		else if(GetShootMode() == NORMAL)
			ShootMotorPositionPID.ref +=60;
	}
	else if(ShootMotorSpeedPID.fdb<0xf && ShootMotorPositionPID.ref<3)
		shooterState = SHOOT_COOLING;
	if(shooterState!=SHOOT_PREPARE)
		shoot_timer++;
	if(shoot_timer == 1000 && shooterState == SHOOT_COOLING)
		shooterState = SHOOT_PREPARE;
	ShooterMControl();
}



static uint32_t time_tick_1ms = 0;
//�������񣬷���timer6 1ms��ʱ�ж���ִ��
void Control_Task(void)
{
	time_tick_1ms++; //Keep track of time
	WorkStateFSM(); //State machine: PREPARE NORMAL STANDBY CALI STOP
	WorkStateSwitchProcess();
	//��������ݴ����Ƶ����ݳ�ʼ����Ԫ��
	if(time_tick_1ms <100)	
	{
		Init_Quaternion();	// Only carried out in the first 100ms
	}
	//ƽ̨�ȶ������󣬸�λ������ģ��
	if(time_tick_1ms == PREPARE_TIME_TICK_MS/2) //time_tick == 2000ms
	{
		GYRO_RST();
	}
		
	//step 1: ��̨����
	GimbalYawControlModeSwitch();   //ģʽ�л������õ�λ�û����趨ֵ�͸���ֵ
	GMPitchControlLoop();
	GMYawControlLoop();
	SetGimbalMotorOutput();
	CalibrateLoop();   //У׼���񣬵����յ�У׼��������Чִ�У�����ֱ������
	//chassis motor control
	if(time_tick_1ms%4 == 0)         //motor control frequency 4ms
	{
		//�������
		SuperviseTask();    
		//���̿�������
		CMControlLoop();			 
		ShooterMControlLoop();       //���������������
	}
	
}
/**********************************************************
*����״̬�л�״̬��,��1ms��ʱ�ж�ͬƵ��
**********************************************************/

void WorkStateFSM(void)
{
	lastWorkState = workState;
	switch(workState)
	{
		case PREPARE_STATE:
		{
			if(GetInputMode() == STOP || Is_Serious_Error())   /*InputMode = remote_input or key_mouse_input or stop. Specify how 
																														the system is controlled 
																													Serious_error = MPU6050_ERR or Deadclock_ERR or ZGYRO_ERR or NOCALL_ERR(?)*/
			{
				workState = STOP_STATE;
			}
			else if(GetCaliCmdFlagGrp())   
			{
				workState = CALI_STATE;
			}
			else if(time_tick_1ms > PREPARE_TIME_TICK_MS)
			{
				workState = NORMAL_STATE;
			}			
		}break;
		case NORMAL_STATE:     
		{
			if(GetInputMode() == STOP || Is_Serious_Error() || emergency == -1)
			{
				workState = STOP_STATE;
			}
			else if(GetCaliCmdFlagGrp())   
			{
				workState = CALI_STATE;
			}
			else if((!IsRemoteBeingAction() ||(Get_Lost_Error(LOST_ERROR_RC) == LOST_ERROR_RC)) && GetShootState() != SHOOTING)
			{
				workState = STANDBY_STATE;      
			}			
		}break;
		case STANDBY_STATE:     
		{
			if(GetInputMode() == STOP || Is_Serious_Error() || emergency == -1)
			{
				workState = STOP_STATE;
			}
			else if(GetCaliCmdFlagGrp())   
			{
				workState = CALI_STATE;
			}
			else if(IsRemoteBeingAction() || (GetShootState()==SHOOTING) || GetFrictionState() == FRICTION_WHEEL_START_TURNNING)
			{
				workState = NORMAL_STATE;
			}				
		}break;
		case STOP_STATE:   
		{
			if(GetInputMode() != STOP && !Is_Serious_Error() && emergency != -1)
			{
				workState = PREPARE_STATE;   
			}
		}break;
		case CALI_STATE:      
		{
			if(GetInputMode() == STOP || Is_Serious_Error())
			{
				workState = STOP_STATE;
			}
		}break;	    
		default:
		{
			
		}
	}	
}

static void WorkStateSwitchProcess(void)
{
	//���������ģʽ�л���prapareģʽ��Ҫ��һϵ�в�����ʼ��
	if((lastWorkState != workState) && (workState == PREPARE_STATE))  
	{
		ControtLoopTaskInit();
		RemoteTaskInit();
	}
}

/*
************************************************************************************************************************
*Name        : GimbalYawControlModeSwitch
* Description: This function process the yaw angle ref and fdb according to the WORKSTATE.
* Arguments  : void     
* Returns    : void
* Note(s)    : 1) from NORMAL to STANDBY it need to delay a few seconds to wait for the IMU to be steady.  
                  STATE_SWITCH_DELAY_TICK represents the delay time.
************************************************************************************************************************
*/

void GimbalYawControlModeSwitch(void)
{
	static uint8_t normalFlag = 0;   //��������ģʽ��־
	static uint8_t standbyFlag = 1;  //IMU����ģʽ��־
	static uint32_t modeChangeDelayCnt = 0;
	//static float angleSave = 0.0f;    //�����л�ģʽʱ�����л�ǰ�ĽǶ�ֵ�����ڽǶȸ���ֵ�л�
	switch(GetWorkState())
	{
		case PREPARE_STATE:   //�������̣�����б��
		{
			GMYPositionPID.ref = 0.0f;
			GMYPositionPID.fdb = -GMYawEncoder.ecd_angle*GMYawRamp.Calc(&GMYawRamp);
			standbyFlag = 1; //for emergency, make it robust.
			//angleSave = ZGyroModuleAngle;			
		}break;
		case NORMAL_STATE:
		{
			if(standbyFlag == 1)
			{
				standbyFlag = 0;
				normalFlag = 1;
				GimbalRef.yaw_angle_dynamic_ref = yaw_angle;   //�޸��趨ֵΪSTANDBY״̬�¼�¼�����һ��ZGYROMODULEAngleֵ
				modeChangeDelayCnt = 0;   //delay����
			}
			GMYPositionPID.ref = GimbalRef.yaw_angle_dynamic_ref;   //�趨����ֵ
			GMYPositionPID.fdb = yaw_angle; 					//�趨����ֵ
			//angleSave = yaw_angle;   //ʱ�̱���IMU��ֵ���ڴ�NORMAL��STANDBYģʽ�л�
		}break;
		case STANDBY_STATE:   //IMUģʽ
		{
			modeChangeDelayCnt++;
			if(modeChangeDelayCnt < STATE_SWITCH_DELAY_TICK)    //delay�����ʱ����NORMAL_STATEһ��
			{
				GMYPositionPID.ref = GimbalRef.yaw_angle_dynamic_ref;   //�趨����ֵ
				GMYPositionPID.fdb = yaw_angle; 					//�趨����ֵ
				//angleSave = yaw_angle;
			}
			else     //delayʱ�䵽���л�ģʽ��IMU
			{
				if(normalFlag == 1)   //�޸�ģʽ��־
				{
					normalFlag = 0;
					standbyFlag = 1;
					GimbalRef.yaw_angle_dynamic_ref = yaw_angle;    //�������delayʱ����ڱ����
				}
				GMYPositionPID.ref = GimbalRef.yaw_angle_dynamic_ref;   //�趨����ֵ
				GMYPositionPID.fdb = yaw_angle; 					//�趨����ֵ	
				//angleSave = ZGyroModuleAngle;           //IMUģʽʱ������ZGyro��ֵ��ģʽ�л�ʱ�޸ĸ���ֵʹ��						
			}
		}break;
		case STOP_STATE:    //ֹͣ����ģʽ
		{
			
		}break;
		case CALI_STATE:    //У׼ģʽ
		{
			
		}break;
	}	
}

//��̨pitch����Ƴ���
void GMPitchControlLoop(void)
{
	GMPPositionPID.kp = PITCH_POSITION_KP_DEFAULTS + PitchPositionSavedPID.kp_offset;
	GMPPositionPID.ki = PITCH_POSITION_KI_DEFAULTS + PitchPositionSavedPID.ki_offset;
	GMPPositionPID.kd = PITCH_POSITION_KD_DEFAULTS + PitchPositionSavedPID.kd_offset;
		
	GMPSpeedPID.kp = PITCH_SPEED_KP_DEFAULTS + PitchSpeedSavedPID.kp_offset;
	GMPSpeedPID.ki = PITCH_SPEED_KI_DEFAULTS + PitchSpeedSavedPID.ki_offset;
	GMPSpeedPID.kd = PITCH_SPEED_KD_DEFAULTS + PitchSpeedSavedPID.kd_offset;
	
	GMPPositionPID.ref = GimbalRef.pitch_angle_dynamic_ref;
	GMPPositionPID.fdb = -GMPitchEncoder.ecd_angle * GMPitchRamp.Calc(&GMPitchRamp);    //����б�º���
	GMPPositionPID.Calc(&GMPPositionPID);   //�õ�pitch��λ�û����������
	//pitch speed control
	GMPSpeedPID.ref = GMPPositionPID.output;
	GMPSpeedPID.fdb = MPU6050_Real_Data.Gyro_Y;
	GMPSpeedPID.Calc(&GMPSpeedPID);
}

void GMYawControlLoop(void)
{
		/*
	GMYPositionPID.kp = YAW_POSITION_KP_DEFAULTS + YawPositionSavedPID.kp_offset;//  gAppParamStruct.YawPositionPID.kp_offset;  //may be bug if more operation  done
	GMYPositionPID.ki = YAW_POSITION_KI_DEFAULTS + YawPositionSavedPID.ki_offset;
	GMYPositionPID.kd = YAW_POSITION_KD_DEFAULTS + YawPositionSavedPID.kd_offset;
	*/
	GMYPositionPID.kp = YAW_POSITION_KP_DEFAULTS;// Manually set PID parameters
	GMYPositionPID.ki = YAW_POSITION_KI_DEFAULTS;
	GMYPositionPID.kd = YAW_POSITION_KD_DEFAULTS;
	
	/*GMYSpeedPID.kp = YAW_SPEED_KP_DEFAULTS + YawSpeedSavedPID.kp_offset;
	GMYSpeedPID.ki = YAW_SPEED_KI_DEFAULTS + YawSpeedSavedPID.ki_offset;
	GMYSpeedPID.kd = YAW_SPEED_KD_DEFAULTS + YawSpeedSavedPID.kd_offset;*/
	GMYSpeedPID.kp = YAW_SPEED_KP_DEFAULTS; // Manually set
	GMYSpeedPID.ki = YAW_SPEED_KI_DEFAULTS;
	GMYSpeedPID.kd = YAW_SPEED_KD_DEFAULTS;
	GMYPositionPID.Calc(&GMYPositionPID);
	//yaw speed control
	GMYSpeedPID.ref = GMYPositionPID.output;
	GMYSpeedPID.fdb = MPU6050_Real_Data.Gyro_Z;
	GMYSpeedPID.Calc(&GMYSpeedPID);			
}

void SetGimbalMotorOutput(void)
{
	//��̨�������								
	if((GetWorkState() == STOP_STATE) ||Is_Serious_Error() || GetWorkState() == CALI_STATE)   
	{
		Set_Gimbal_Current(CAN2, 0, 0);     //yaw + pitch			
	}
	else
	{		
		//Set_Gimbal_Current(CAN2, -(int16_t)GMYSpeedPID.output, (int16_t)GMPSpeedPID.output);     //yaw + pitch		
		//Set_Gimbal_Current(CAN2, 0, (int16_t)GMPSpeedPID.output);		
	}		
}
//���������ʼ������
void ControtLoopTaskInit(void)
{
	//������ʼ��
	time_tick_1ms = 0;   //�ж��еļ�������
	//���������ʼ��
	AppParamInit();
	//У׼�����ƫ��ֵ��ʼ��
	Sensor_Offset_Param_Init(&gAppParamStruct);
	//���ù���ģʽ
	SetWorkState(PREPARE_STATE);
	//б�³�ʼ��
	GMPitchRamp.SetScale(&GMPitchRamp, PREPARE_TIME_TICK_MS);
	GMYawRamp.SetScale(&GMYawRamp, PREPARE_TIME_TICK_MS);
	GMPitchRamp.ResetCounter(&GMPitchRamp);
	GMYawRamp.ResetCounter(&GMYawRamp);
	//��̨�����Ƕȳ�ʼ��
	GimbalRef.pitch_angle_dynamic_ref = 0.0f;
	GimbalRef.yaw_angle_dynamic_ref = 0.0f;
    //��������ʼ��
    LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_RC));
    LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_IMU));
    //LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_ZGYRO));
    LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR1));
    LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR2));
    LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR3));
    LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR4));
    LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR5));
    LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_MOTOR6));
    LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_DEADLOCK));
    LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_NOCALI));
    
	//PID��ʼ��
	ShootMotorSpeedPID.Reset(&ShootMotorSpeedPID);
	GMPPositionPID.Reset(&GMPPositionPID);
	GMPSpeedPID.Reset(&GMPSpeedPID);
	GMYPositionPID.Reset(&GMYPositionPID);
	GMYSpeedPID.Reset(&GMYSpeedPID);
	CMRotatePID.Reset(&CMRotatePID);
	CM1SpeedPID.Reset(&CM1SpeedPID);
	CM2SpeedPID.Reset(&CM2SpeedPID);
	CM3SpeedPID.Reset(&CM3SpeedPID);
	CM4SpeedPID.Reset(&CM4SpeedPID);
}
