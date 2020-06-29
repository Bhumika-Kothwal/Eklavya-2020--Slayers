#include "vrep.h"

#define MAX_PWM 10
#define MIN_PWM 3

//Line Following Tuning Parameters
float yaw_kP= 0.65;
float yaw_kI= 0;
float yaw_kD= 0.17;

//FOR LINE FOLLOWING
float yaw_error=0, yaw_prev_error=0, yaw_difference=0, yaw_cumulative_error=0, yaw_correction=0;
int weights[4] = {3,1,-1,-3};
float left_pwm = 0, right_pwm = 0;

int clientID;
int leftMotorHandle;
int rightMotorHandle;
int sensor[5];
int direction = 1;

float *read_sensors(int clientID, int sensor[]);
float constrain(float k, float lower_limit, float higher_limit);
int move(int clientID,int leftMotorHandle,int rightMotorHandle,float left_pwm,float right_pwm);
int pid_control(float sensor_values[]);
int pid_control_slow(float sensor_values[]);
int turn(int clientID, int leftMotorHandle,int rightMotorHandle,int leftMotorSpeed,int rightMotorSpeed);
int pid_correction(float sensor_values[], int index);

int main(int argc,char* argv[])
{

	// all handles that will be passed from lua to this code
	int portNb=0;
	// number of variables passed from lua to this code + 1
	// (+1 because the first value in argv is always the location of this program)
	int n = 9;

	// save the handles
	if (argc>=n)
	{
		portNb=atoi(argv[1]); 
		leftMotorHandle = atoi(argv[2]);
		rightMotorHandle = atoi(argv[3]);
		sensor[0] = atoi(argv[4]);
		sensor[1] = atoi(argv[5]);
		sensor[2] = atoi(argv[6]);
		sensor[3] = atoi(argv[7]);
		sensor[4] = atoi(argv[8]);
	}
	else
	{
		printf("Sufficient arguments not provided'!\n");
		extApi_sleepMs(5000);
		return 0;
	}

	// variables for the code
	float turn_speed = 1;
	float* sensor_values;
	int lap_time=0;
	int turn_taken = -1;
	int turn_stored[100];
	int t=0;
	int ab;
	// functions or variable names starting with 'simx' are coppeliamsim functions
	// for details on these functions refer: https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctions.htm

	// start communication with coppeliasim
	clientID=simxStart((simxChar*)"127.0.0.1",portNb,true,true,2000,5);

	if (clientID!=-1)
	{
		printf("connection established\n");
		FILE *fpointer;
		fpointer = fopen("/home/bhumika/CoppeliaSim_Edu_V4_0_0_Ubuntu16_04/text_file.txt" , "r");
		if(fpointer == NULL)
			printf("File couldn't be opened \n");
		while(!feof(fpointer))
		{
			fscanf(fpointer, "%d", &turn_stored[t]);
			printf("turn = %d\n", turn_stored[t]);
			t++;
		}
		fclose(fpointer);
		ab = t-2;
		
		while (simxGetConnectionId(clientID)!=-1)
		{  
			//reading sensors
			sensor_values = read_sensors(clientID, sensor);
			if(sensor_values[0]>0 && sensor_values[1]>0 && sensor_values[2]>0 && sensor_values[3]>0)	//all the sensor readings should be greater than 0
			{
				if(sensor_values[0]<0.7 && sensor_values[1]<0.7 && sensor_values[2]<0.7 && sensor_values[3]<0.7 && sensor_values[4]<0.7)	//end of maze
				{
					int count = 0;
					/*while(count<=100)
					{
						count++;
						move(clientID, leftMotorHandle, rightMotorHandle, 0, 0);
					}*/
					turn(clientID, leftMotorHandle, rightMotorHandle, 0, 0);
					break;
				}
				else
				{
					pid_control(sensor_values);
					while(lap_time< 5000 && turn_taken==1)
					{
						lap_time++;
						sensor_values = read_sensors(clientID, sensor);
						pid_control(sensor_values);
					}
					turn_taken = 0;
					lap_time = 0;
					if(sensor_values[3]<0.3 && sensor_values[1]<0.3 && sensor_values[2]<0.3 && sensor_values[0]<0.3)					//for junction or t-shape
					{
						while(simxGetConnectionId(clientID)!=-1)
						{	//reading sensors
							sensor_values = read_sensors(clientID, sensor);
							if(sensor_values[0]<0.3 && sensor_values[1]<0.3 && sensor_values[2]<0.3 && sensor_values[3]<0.3)
								move(clientID, leftMotorHandle, rightMotorHandle, turn_speed, turn_speed);
							else
							{
								if((sensor_values[0]>0.7 && sensor_values[1]>0.7 && sensor_values[2]>0.7 && sensor_values[3]>0.7) || (sensor_values[0]>0.7 && sensor_values[1]<0.7 && sensor_values[2]<0.7 && sensor_values[3]>0.7))
								{

									if(turn_stored[ab] == 30)
									{
										turn(clientID, leftMotorHandle, rightMotorHandle,turn_speed,-1*turn_speed);
										direction = (direction==4)?1:(direction+1);
										turn_taken=1;
										ab--;
										break;
									}

									else if(turn_stored[ab] == 10)
									{
										turn(clientID, leftMotorHandle, rightMotorHandle,-1*turn_speed,turn_speed);
										direction = (direction==1)?4:(direction-1);
										turn_taken=1;
										ab--;
										break;
									}	

									else if(turn_stored[ab] == 20)
									{
										turn_taken=1;
										ab--;
										break;
									}
								}
							}
						}
					}

					else if(sensor_values[3]<0.4 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[0]>0.7)			//pure right or straight right turn detected
					{
						while(simxGetConnectionId(clientID)!=-1)
						{	//reading sensors
							sensor_values = read_sensors(clientID, sensor);	
							if(sensor_values[0]>0.7 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]<0.4)
								pid_correction(sensor_values,3);
							else
							{
								if((sensor_values[0]>0.5 && sensor_values[1]>0.2 && sensor_values[2]>0.2 && sensor_values[3]>0.5) || (sensor_values[0]>0.7 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]>0.7))
								{
					
									if(turn_stored[ab] == 30)
									{
										turn(clientID, leftMotorHandle, rightMotorHandle,turn_speed,-1*turn_speed);
										direction = (direction==4)?1:(direction+1);
										turn_taken=1;
										ab--;
										break;
									}

									else if(turn_stored[ab] == 20)
									{
										turn_taken=1;
										ab--;
										break;
									}
								}
							}
						}
					}

					else if(sensor_values[0]<0.4 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]>0.7)			//pure left or straight left turn
					{
						while(simxGetConnectionId(clientID)!=-1)
						{	//reading sensor values
							sensor_values = read_sensors(clientID, sensor);
							if(sensor_values[0]<0.4 && sensor_values[1]<0.4 && sensor_values[2]<0.4 && sensor_values[3]>0.7)	//getting till end of node
								pid_correction(sensor_values,0);
							else
							{
								if((sensor_values[0]>0.5 && sensor_values[1]>0.2 && sensor_values[2]>0.2 && sensor_values[3]>0.5) || (sensor_values[0]>0.7 && sensor_values[1]<0.5 && sensor_values[2]<0.5 && sensor_values[3]>0.7))
								{
					
									if(turn_stored[ab] == 10)
									{
										turn(clientID, leftMotorHandle, rightMotorHandle,-1*turn_speed,turn_speed);
										direction = (direction==1)?4:(direction-1);
										turn_taken=1;
										ab--;
										break;
									}

									else if(turn_stored[ab] == 20)
									{
										turn_taken=1;
										ab--;
										break;
									}
								}
							}
						}
					}
				}
			}
		}
		// end communication with lua
		simxFinish(clientID);
	}
	printf("Simulation Stopped!");
}


float *read_sensors(int clientID, int sensor[])
{
	static float sensor_value[5];
	float* auxValues = NULL;
	int* auxValuesCount = NULL;
	simxReadVisionSensor(clientID, sensor[0], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
	if( simxReadVisionSensor(clientID, sensor[0], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok)
		sensor_value[0] = auxValues[10];
	simxReadVisionSensor(clientID, sensor[1], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
	if( simxReadVisionSensor(clientID, sensor[1], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok)
		sensor_value[1] = auxValues[10];
	simxReadVisionSensor(clientID, sensor[2], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
	if( simxReadVisionSensor(clientID, sensor[2], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok)
		sensor_value[2] = auxValues[10];
	simxReadVisionSensor(clientID, sensor[3], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
	if( simxReadVisionSensor(clientID, sensor[3], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok)
		sensor_value[3] = auxValues[10];
	simxReadVisionSensor(clientID, sensor[4], NULL, &auxValues, &auxValuesCount,simx_opmode_streaming);
	if( simxReadVisionSensor(clientID, sensor[4], NULL, &auxValues, &auxValuesCount,simx_opmode_buffer) == simx_return_ok)
		sensor_value[4] = auxValues[10];
	printf("the sensor values: %f %f %f %f %f  %d\n",sensor_value[4], sensor_value[0],sensor_value[1],sensor_value[2],sensor_value[3],direction);
	return sensor_value;
}


float constrain(float k, float lower_limit, float higher_limit)
{
	if(k < lower_limit)
		k = lower_limit;
	else if(k > higher_limit)
		k = higher_limit;
	return k;
}


int move(int clientID,int leftMotorHandle,int rightMotorHandle,float left_pwm,float right_pwm)
{
	simxSetJointTargetVelocity(clientID,leftMotorHandle,left_pwm,simx_opmode_oneshot);
	simxSetJointTargetVelocity(clientID,rightMotorHandle,right_pwm,simx_opmode_oneshot);
}


int pid_control_slow(float sensor_values[])
{
	float yaw_kP_slow= 0.5;
	float yaw_kI_slow= 0.1;
	float yaw_kD_slow= 0.36;
	int all_black_flag = 1;
	float weighted_sum = 0, sum = 0, pos = 0;

	for(int i = 0; i < 4; i++)
	{
		if(sensor_values[i] < 0.4)
			all_black_flag = 0;
		weighted_sum += (float)(sensor_values[i]) * (weights[i]);
		sum += sensor_values[i];
	}
	if(sum != 0)
		pos = weighted_sum / sum;
	if(all_black_flag == 1)
	{
		if(yaw_error > 0)
			pos = 0.15;
		else
			pos = -0.15;
	}
	yaw_error = pos;

	//calculating_yaw_correction :-
	yaw_difference = (yaw_error - yaw_prev_error);
	yaw_cumulative_error += yaw_error;
	if(yaw_cumulative_error > 0.3)
		yaw_cumulative_error = 0.3;
	else if(yaw_cumulative_error < -0.3)
		yaw_cumulative_error = -0.3;
	yaw_correction = yaw_kP_slow*yaw_error + yaw_kI_slow*yaw_cumulative_error + yaw_kD_slow*yaw_difference;
	yaw_prev_error = yaw_error;

	right_pwm = constrain((yaw_correction), 1, 10);
	left_pwm = constrain((-yaw_correction), 1, 10);

	//Extra yaw correction during turns
	if(yaw_error>0.05)
	{
		right_pwm-=0.25;
		left_pwm+=0.25;
	}
	else if(yaw_error<0.05)
	{
		left_pwm-=0.25;
		right_pwm+=0.25;
	}

	move(clientID,leftMotorHandle,rightMotorHandle,left_pwm,right_pwm);
}


int pid_control(float sensor_values[])
{
	int all_black_flag = 1;
	float weighted_sum = 0, sum = 0, pos = 0;
	for(int i = 0; i < 4; i++)
	{
		if(sensor_values[i] < 0.4)
			all_black_flag = 0;
		weighted_sum += (float)(sensor_values[i]) * (weights[i]);
		sum += sensor_values[i];
	}	
	if(sum != 0)
		pos = weighted_sum / sum;

	if(all_black_flag == 1)
	{
		if(yaw_error > 0)
			pos = 0.15;
		else
			pos = -0.15;
	}
	yaw_error = pos;
	//calculating_yaw_correction :-
	yaw_difference = (yaw_error - yaw_prev_error);
	yaw_cumulative_error += yaw_error;
	if(yaw_cumulative_error > 0.3)
		yaw_cumulative_error = 0.3;

	else if(yaw_cumulative_error < -0.3)
		yaw_cumulative_error = -0.3;
	
	yaw_correction = yaw_kP*yaw_error + yaw_kI*yaw_cumulative_error + yaw_kD*yaw_difference;
	yaw_prev_error = yaw_error;
	right_pwm = constrain((yaw_correction), MIN_PWM, MAX_PWM);
	left_pwm = constrain((-yaw_correction), MIN_PWM, MAX_PWM);
	//Extra yaw correction during turns
	if(yaw_error>0.05)
	{
		right_pwm-=0.58;
		left_pwm+=0.58;   
	}
	else if(yaw_error<0.05)
	{
		left_pwm-=0.58;
		right_pwm+=0.58;
	}
	move(clientID,leftMotorHandle,rightMotorHandle,left_pwm,right_pwm);
}



int pid_correction(float sensor_values[], int index)
{
	if(index==0)
		sensor_values[0] = 0.76;
	else if(index==3)
		sensor_values[3] = 0.76;
	pid_control(sensor_values);
}


int turn(int clientID, int leftMotorHandle,int rightMotorHandle,int leftMotorSpeed,int rightMotorSpeed)
{
	int v=0, count=0;
	float *sensor_values;
	while(count<=20)
	{
		move(clientID,leftMotorHandle,rightMotorHandle,0.01,0.01);   	
		count++;
	}
	while(count<=600000)
	{
		move(clientID,leftMotorHandle,rightMotorHandle,leftMotorSpeed,rightMotorSpeed);
		count++;
	}
	
	while(simxGetConnectionId(clientID)!=-1)
	{
		sensor_values = read_sensors(clientID, sensor);
		if(sensor_values[0]>0.5 &&sensor_values[1]<0.5 &&sensor_values[2]<0.5 &&sensor_values[3]>0.5)
			break;
		else
			move(clientID,leftMotorHandle,rightMotorHandle,leftMotorSpeed,rightMotorSpeed);
	}
}
