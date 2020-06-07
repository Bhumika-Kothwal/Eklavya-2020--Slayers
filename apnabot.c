#include "vrep.h"

// for more info refer to 
// https://www.coppeliarobotics.com/helpFiles/en/legacyRemoteApiOverview.htm 

int main(int argc,char* argv[])
{

    // all handles that will be passed from lua to this code
    int portNb=-1;
    int leftMotorHandle;
    int rightMotorHandle;  
    
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
    float leftMotorSpeed = 3000000;
    float rightMotorSpeed = 3000000;
    float* auxValues = NULL;
    int* auxValuesCount = NULL;

    // functions or variable names starting with 'simx' are coppeliamsim functions
    // for details on these functions refer: https://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctions.htm

    // start communication with coppeliasim
    int clientID=simxStart((simxChar*)"127.0.0.1",portNb,true,true,2000,5);

    if (clientID!=-1){
        printf("connection established...\n");

        /******************** SETUP ******************************/
        // write code here that you wish to perform ONCE
        simxSetJointTargetVelocity(clientID,leftMotorHandle,leftMotorSpeed,simx_opmode_oneshot);
        simxSetJointTargetVelocity(clientID,rightMotorHandle,rightMotorSpeed,simx_opmode_oneshot);
        /*********************************************************/
        
        }
        
        // end communication with lua
        simxFinish(clientID);

    }
    printf("Simulation Stopped!");
}
