#include "vrep.h"

// for more info refer to 
// https://www.coppeliarobotics.com/helpFiles/en/legacyRemoteApiOverview.htm 
int move(int clientID,int leftMotorHandle,int rightMotorHandle,float leftMotorSpeed,float rightMotorSpeed);
int main(int argc,char* argv[])
{

    // all handles that will be passed from lua to this code
    int portNb=-1;
    int leftMotorHandle;
    int rightMotorHandle;  
    int dummyHandle;
    // number of variables passed from lua to this code + 1
    // (+1 because the first value in argv is always the location of this program)
    int n = 4;

    // save the handles
    if (argc>=n){
        portNb=atoi(argv[1]); 
        leftMotorHandle = atoi(argv[2]);
        rightMotorHandle = atoi(argv[3]);
    }
    else{
        printf("Sufficient arguments not provided'!\n");
        extApi_sleepMs(5000);
        return 0;
    }

    // variables for the code
    float position[3] = {0,0,0};
    float leftMotorSpeed = 1;
    float rightMotorSpeed = 1;
    float* auxValues = NULL;
    int* auxValuesCount = NULL;
	char s;
	
    // for details on these functions refer: https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctions.htm

    // start communication with coppeliasim
    int clientID=simxStart((simxChar*)"127.0.0.1",portNb,true,true,2000,5);

    if (clientID!=-1){
        printf("connection established...\n");
	float time=0;
        /******************** SETUP ******************************/
        // write code here that you wish to perform ONCE
		printf("motion type:\n");
		scanf("%c",&s);
		while (simxGetConnectionId(clientID)!=-1){

			//simxGetObjectPosition(clientID, dummyHandle, -1, position, simx_opmode_streaming);
			//if( simxGetObjectPosition(clientID, dummyHandle, -1, position, simx_opmode_buffer) == simx_return_ok)
                	//	printf("the pos: %f %f %f\n",position[0],position[1],position[2]);

			if(s=='f')
        		move(clientID,leftMotorHandle,rightMotorHandle,leftMotorSpeed,rightMotorSpeed);
        	else if(s=='b')
       			move(clientID,leftMotorHandle,rightMotorHandle,-1*leftMotorSpeed,-1*rightMotorSpeed);
        	else if(s=='l')
        		move(clientID,leftMotorHandle,rightMotorHandle,-1*leftMotorSpeed,rightMotorSpeed);
        	else if(s=='r')
        		move(clientID,leftMotorHandle,rightMotorHandle,leftMotorSpeed,-1*rightMotorSpeed);
        	else{ 
        		printf("invalid input\n");
        		break;
        	}
        	extApi_sleepMs(25);
        	        /*********************************************************/
        }
        
        // end communication with lua
        simxFinish(clientID);

    }
    printf("Simulation Stopped!");
}
int move(int clientID,int leftMotorHandle,int rightMotorHandle,float leftMotorSpeed,float rightMotorSpeed)
{
	simxSetJointTargetVelocity(clientID,leftMotorHandle,leftMotorSpeed,simx_opmode_oneshot);
	simxSetJointTargetVelocity(clientID,rightMotorHandle,rightMotorSpeed,simx_opmode_oneshot);
}
