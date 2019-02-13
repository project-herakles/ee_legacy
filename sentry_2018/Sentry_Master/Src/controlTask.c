#include "controlTask.h"
#include "remoteTask.h"
#include "can.h"
#include "gpio.h"
#include "tim.h"
#include "pid.h"
#include "sineWave.h"
#include "fuzzy.h"
#include "ramp.h"
#include "gun.h"
#include "COM_Task.h"
#include "sensor.h"

const float GMControlFreq = 1000.0f; // a parameter of ramping function to translate angular speed into discrete increments
const float YAW_W = 45.0f; // specify the maximum angular velocity for yaw [was 45]
const float PITCH_W = 5.0f; // specify the maximum angular velocity for pitch [was 36]
const float YAW_INIT = -60.0f;
const float PITCH_INIT = -30.0f;
workState_e workState;
frictionState_e fricState = FRIC_STATIC;
uint32_t time_tick_ms = 0;

PID_Regulator_t CM1SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM2SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t GMYPositionPID = YAW_POSITION_PID_DEFAULT;
PID_Regulator_t GMPPositionPID = PITCH_POSITION_PID_DEFAULT;
PID_Regulator_t GMYSpeedPID = YAW_SPEED_PID_DEFAULT;
PID_Regulator_t GMPSpeedPID = PITCH_SPEED_PID_DEFAULT;

PID_Regulator_t ShootSpeedPID = SHOOT_SPEED_PID_DEFAULT;
PID_Regulator_t ShootPositionPID = SHOOT_POSITION_PID_DEFAULT;

extern volatile Encoder CM1Encoder;
extern volatile Encoder CM2Encoder;
extern volatile Encoder GMYawEncoder;
extern volatile Encoder GMPitchEncoder;
extern volatile Encoder ShootEncoder;

extern Chassis_speed_ref_t chassis_speed_ref;
extern float m_INC;
extern PC_Command_t PC_cmd;
extern RC_Ctrl_t RC_CtrlData;
extern BumpSensors_t bSensors;

uint32_t getCurrentTimeTick(void)
{
	return time_tick_ms;
}
void setWorkState(workState_e state)
{
	workState = state;
}

workState_e getWorkState(void)
{
	return workState;
}

void workStateFSM(void)
{
	switch(workState)
	{
		case PREPARE_STATE:
			if(getInputMode() == STOP_MODE)
				setWorkState(STOP_STATE);
			else if(time_tick_ms > PREPARE_TIME_TICK_MS)
				setWorkState(NORMAL_STATE);
			break;
			
		case NORMAL_STATE:
			if (getInputMode() == STOP_MODE)
				setWorkState(STOP_STATE);
			break;
		
		case STOP_STATE:
			if(getInputMode() != STOP_MODE)
				setWorkState(PREPARE_STATE);
			break;
		default:;	
	}
}

void CM_Control(void)
{
	float CM_Speed  = 0;
	if(getWorkState() != NORMAL_STATE)
	{
		CM_Speed = 0;
	}
	else if(bSensors.reachLeft || bSensors.reachRight)
	{
		(bSensors.reachLeft==1) ? (CM_Speed=200) : (CM_Speed=-200);
	}
	else
	{
		switch(PC_cmd.Chassis.dir)
		{
			case 0: // stop
			{
				CM_Speed = 0;
			}break;
			case 1: // left_slow
			{
				CM_Speed = -200;
			}break;
			case 2: // left_fast
			{
				CM_Speed = -500;
			}break;
			case 3: // right_slow
			{
				CM_Speed = 200;
			}break;
			case 4: // right_fast
			{
				CM_Speed = 500;
			}break;
			case 5: // left track
			{
				CM_Speed = -fabs(GMYPositionPID.ref-GMYPositionPID.fdb) * 10; // linear factor of 10 to trak
			}break;
			case 6: // right track
			{
				CM_Speed = fabs(GMYPositionPID.ref-GMYPositionPID.fdb) * 10; // linear factor of 10 to trak
			}break;
			default:
			{
			}
		}
	}
	//CM1SpeedPID.ref =  (-chassis_speed_ref.forward_back_ref*0.075)*3;
	//CM2SpeedPID.ref = (chassis_speed_ref.forward_back_ref*0.075)*3;
	
	CM1SpeedPID.ref = -CM_Speed;
	CM2SpeedPID.ref = CM_Speed;
	
	CM1SpeedPID.fdb = CM1Encoder.filter_rate;
	CM2SpeedPID.fdb = CM2Encoder.filter_rate;
	
	PID_Calc(&CM1SpeedPID);
	PID_Calc(&CM2SpeedPID);
	//set_CM_speed(CM1SpeedPID.output*SPEED_OUTPUT_ATTENUATION,CM2SpeedPID.output*SPEED_OUTPUT_ATTENUATION);
	//set_CM_speed(CM1SpeedPID.output*SPEED_OUTPUT_ATTENUATION,-CM1SpeedPID.output*SPEED_OUTPUT_ATTENUATION);
	set_CM_speed((int16_t)CM_Speed*4,(int16_t)-CM_Speed*4);
}

float angleLimit(float angle,float lower, float upper)
{
	if(angle<lower) return lower;
	else if(angle>upper) return upper;
	return angle;
	
}

static float yaw_r = -60; // Initial Yaw value
static float pitch_r = -30; // Initial Pitch value
static uint32_t start_time = 0;
static char start_flag = 0;
static float compensate = 0;
static uint8_t c_cnt = 0;
static float reference = 0;
static float last_yaw_fdb = YAW_INIT;

void Gimbal_Control(void)
{
	//RED_LED_ON();
	switch(workState)
	{
		case PREPARE_STATE: // Init state (Yaw,Pitch) = (0,0)
		{
			yaw_r = YAW_INIT;
			pitch_r = PITCH_INIT; // Initial Yaw/Pitch value
			
			GMYPositionPID.ref = yaw_r;
			GMYPositionPID.fdb = GMYawEncoder.ecd_angle;
			//fuzzy test
			if(fabs(GMYPositionPID.ref-GMYPositionPID.fdb)<5.0f)
				PID_Calc_Debug(&GMYPositionPID,0.5,0.0006,0);
			else
				PID_Calc_Debug(&GMYPositionPID,0.2,0.00001,5); // a debug version of PID_Calc for testing parameters (P=0.6,I=0.0003,D=8)
			GMYSpeedPID.ref = GMYPositionPID.output;
			GMYSpeedPID.fdb = GMYawEncoder.filter_rate;
			PID_Calc_Debug(&GMYSpeedPID,100,0,0);
			
			GMPPositionPID.ref = pitch_r;
			GMPPositionPID.fdb = GMPitchEncoder.ecd_angle;			
			PID_Calc_Debug(&GMPPositionPID,0.5,0.004,1.5);
			GMPSpeedPID.ref = GMPPositionPID.output;
			GMPSpeedPID.fdb = GMPitchEncoder.filter_rate;
			PID_Calc_Debug(&GMPSpeedPID,20.0,0.0,0.0);
			set_GM_speed(-GMYSpeedPID.output,-GMPSpeedPID.output);
		}break;
		case NORMAL_STATE:
		{
			if(PC_cmd.Gimbal.update==1 && fabs(PC_cmd.Gimbal.yaw_angle) < 40)
			{
				yaw_r = GMYPositionPID.fdb - PC_cmd.Gimbal.yaw_angle; //stm32 cw:- ccw:+ ; Odroid cw:+ ccw:-
				yaw_r = angleLimit(yaw_r,-150,120); // Gimbal Limit
				//last_yaw_fdb = GMYPositionPID.fdb; // remember the fdb at the moment of update to counteract delay
			}
	
			GMYPositionPID.ref = yaw_r;
			GMYPositionPID.fdb = GMYawEncoder.ecd_angle;
			if(fabs(GMYPositionPID.ref-GMYPositionPID.fdb)<5.0f)
				PID_Calc_Debug(&GMYPositionPID,1.0,0.003,10);
			else
				PID_Calc_Debug(&GMYPositionPID,0.4,0.00,20); // a debug version of PID_Calc for testing parameters (P=0.6,I=0.0003,D=8)
			
			PID_Smart(&GMYPositionPID,10); // cope with non-linear inteval
			
			GMYSpeedPID.ref = GMYPositionPID.output;
			GMYSpeedPID.fdb = GMYawEncoder.filter_rate;
			PID_Calc_Debug(&GMYSpeedPID,100,0.0,0);
			
			if(PC_cmd.Gimbal.update==1)
			{
				pitch_r = GMPPositionPID.fdb + PC_cmd.Gimbal.pitch_angle;
				pitch_r = angleLimit(pitch_r,-30,0); // Gimbal Limit
				PC_cmd.Gimbal.update = 0; // Reset update flag to prevent undesired increment 
			}
			
			GMPPositionPID.ref = pitch_r;
			GMPPositionPID.fdb = GMPitchEncoder.ecd_angle;
			if(fabs(GMPPositionPID.ref-GMPPositionPID.fdb)<3.0f)
				PID_Calc_Debug(&GMPPositionPID,0.6,0.006,0);
			else
				PID_Calc_Debug(&GMPPositionPID,0.6,0.004,1.5);
			
			GMPSpeedPID.ref = GMPPositionPID.output;
			GMPSpeedPID.fdb = GMPitchEncoder.filter_rate;
			PID_Calc_Debug(&GMPSpeedPID,20.0,0.0,0.0);
			
			set_GM_speed(-GMYSpeedPID.output,-GMPSpeedPID.output);
		}break;
		default:
		{
			set_GM_speed(0,0);
		}
	}
	
		
}

static uint16_t fric_output_s;
// Friction wheel control
void FrictionWheelControl(void)
{
  //if(PC_cmd.shoot.fire == 1) //Receive shooting command from odroid
	//if(RC_CtrlData.rc.s2 == 1)
	if(getWorkState() == NORMAL_STATE)
	{
		if(fricState==FRIC_STATIC) fricState = FRIC_ACCELERATING;
	}
	else
	{
		fricState = FRIC_STATIC;
	}
	switch(fricState)
	{
		case FRIC_STATIC:
		{
			SetFrictionWheelSpeed(1000);
		}break;
		case FRIC_ACCELERATING: 
		{
			uint16_t fric_output = FrictionRamp(1000,1600,500); // start=1000,end=1600,duration=500ms
			fric_output_s = fric_output;
			if(fric_output==1600)	
				fricState = FRIC_MAX_SPEED;
			else
				SetFrictionWheelSpeed(fric_output);
		}break;
		case FRIC_MAX_SPEED:
		{
			SetFrictionWheelSpeed(1600);
		}break;
	}
}

void shootMotorControl(void)
{
	//if(PC_cmd.shoot.fire == 1) //Receive shooting command from odroid
	if(RC_CtrlData.rc.s1 == 1 || PC_cmd.shoot.fire == 1)
	{
		ShootSpeedPID.ref = 120; // was 160
		ShootSpeedPID.fdb = ShootEncoder.filter_rate;
		PID_Calc_Debug(&ShootSpeedPID,10,0.002,0); 
		set_Shoot_speed(ShootSpeedPID.output); //Trigger Motor PID here
	}
	else
	{ 
		set_Shoot_speed(0);
		// Trigger Motor stop
	}
}

void Shoot_Control(void)
{
	switch(workState)
	{
		case PREPARE_STATE:
		{
			InitFrictionWheel();
		}break;
		case NORMAL_STATE:
		{
			FrictionWheelControl();
			shootMotorControl();
		}break;
		case STOP_STATE:
		{
			SetFrictionWheelSpeed(1000); // FrictionWheel stops	
			set_Shoot_speed(0);	// Trigger Motor stop
		}break;
		default:
		{
		}
	}
}

void Control_Loop(void)
{
	time_tick_ms += 1;
	if(time_tick_ms%1000==0)
	{
		RED_LED_OFF();
	}
	//set_CM_speed(2000,2000,2000,2000);
	workStateFSM();
	if(time_tick_ms%4==0)//250Hz
	{
		CM_Control();
	}
	if(time_tick_ms%25==0)//40Hz
	{
		COM_Upload_PC();
	}
	Gimbal_Control();//1000Hz
	Shoot_Control();
	UpdateBumpSensors();
	GMShootControl();//send CAN msg
}
