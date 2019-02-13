#include "fuzzy.h"


const float bigerror = 25;
const float middleerror = 10;
const float smallerror = 3;

const float bigrate = 0;
const float middlerate = 0;
const float smallrate = 0;

const float Kpbig = 1;
const float Kpmiddle = 0.6; 
const float Kpsmall = 0.4;

const float Kibig = 0.000;
const float Kimiddle = 0.0015; 
const float Kismall = 0.001;

const float Kdbig = 0;
const float Kdmiddle = 0;
const float Kdsmall = 0;
void fuzzypid(PID_Regulator_t *pid)
{
	float error = pid->err[0];
	float changerate = changerate;
	if(fabs(error) > bigerror)
	{
		pid->kp = Kpmiddle;
		pid->ki = Kismall;
	}
	else if(fabs(error) > middleerror && fabs(error) < bigerror)
	{
		pid->kp = Kpmiddle;
		pid->ki = Kismall;
	}
	else if(fabs(error) > smallerror && fabs(error) < middleerror)
	{
		pid->kp = Kpsmall;
		pid->ki = Kimiddle;
	}
	else if(fabs(error) < smallerror)
	{
		pid->kp = Kpsmall;
		pid->ki = Kibig;
	}
	/*
	if (fabs(error) > bigerror && fabs(changerate) > bigrate)
	{
		pid->kp = Kpmiddle;
		pid->ki = 0;
		pid->kp = Kdsmall;
	}
	else if (fabs(error) > bigerror && fabs(changerate) <= bigrate && fabs(changerate) > middlerate)
	{
		pid->kp = Kpbig;
		pid->ki = 0;
		pid->kp = Kdmiddle;
	}
	else if (fabs(error) > bigerror && fabs(changerate) <= middlerate && fabs(changerate) >= smallrate)
	{
		pid->kp = Kpbig;
		pid->ki = 0;
		pid->kp = Kdbig;
	}
	else if (fabs(error) > bigerror && fabs(changerate) < smallrate)
	{
		pid->kp = Kpbig;
		pid->ki = 0;
		pid->kp = Kdbig;
	}
	//ÒÔÉÏÊÇÓëÄ¿±êÖµ²î¾àºÜ´óÊ±£
	else if (fabs(error) <= bigerror && fabs(error) > middleerror && fabs(changerate) > bigrate)
	{
		pid->kp = Kpsmall;
		pid->ki = Kismall;
		pid->kd = Kdmiddle;
	}
	else if (fabs(error) <= bigerror && fabs(error) > middleerror && fabs(changerate) <= bigrate && fabs(changerate) > middlerate)
	{
		pid->kp = Kpsmall;
		pid->ki = Kismall;
		pid->kd = Kdmiddle;
	}
	else if (fabs(error) <= bigerror && fabs(error) > middleerror && fabs(changerate) <= middlerate && fabs(changerate) >= smallrate)
	{
		pid->kp = Kpmiddle;
		pid->ki = 0;
		pid->kd = Kdbig;
	}
	else if (fabs(error) <= bigerror && fabs(error) > middleerror && fabs(changerate) < smallrate)
	{
		pid->kp = Kpsmall;
		pid->ki = 0;
		pid->kd = Kdbig;
	}
	//·Ö¸îÏß
	else if (fabs(error) <= middleerror && fabs(error) >= smallerror && fabs(changerate) > bigrate)
	{
		pid->kp = Kpmiddle;
		pid->ki = Kimiddle;
		pid->kd = 0;
	}
	else if (fabs(error) <= middleerror && fabs(error) >= smallerror && fabs(changerate) <= bigrate)
	{
		pid->kp = Kpbig;
		pid->ki = Kibig;
		pid->kd = Kdsmall;
	}
//·Ö¸îÏß
	else if (fabs(error) < smallerror  && fabs(changerate) > bigrate)
	{
		pid->kp = Kpmiddle;
		pid->ki = Kibig;
		pid->kd = 0;
	}
	else if (fabs(error) < smallerror  && fabs(changerate) <= bigrate && fabs(changerate) > middlerate)
	{
		pid->kp = Kpbig;
		pid->ki = Kibig;
		pid->kd = 0;
	}
	else if (fabs(error) < smallerror  && fabs(changerate) <= middlerate && fabs(changerate) >= smallrate)
	{
		pid->kp = Kpbig;
		pid->ki = Kibig;
		pid->kd = Kdsmall;
	}	
	else if (fabs(error) < smallerror  && fabs(changerate) < smallrate)
	{
		pid->kp = Kpbig;
		pid->ki = Kibig;
		pid->kd = 0;
	}	
	*/
}
