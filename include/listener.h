#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "mathlib/MathLib.h"
#include "Utils.h"    

#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/JointState.h"
#include <dynamic_reconfigure/server.h>
#include "geometry_msgs/PointStamped.h"

#include "CDDynamics.h"

/*-- Booleans --*/

	bool _firstHandPoseReceived=false;
	bool _graspTypeReceived = false;
	bool _firstObjectPoseReceived=false;
	bool _firstTargetPoseReceived=false;
	bool _firstRealPoseReceived=false;

/*-- Variables related to the mocap system --*/

int mocapCounter=0;                                              // counter for messages from the mocap system
double lookBack=0.1;                                             // the timewindow to look back for the average velocity
double velThreshold=0.018;                                       // velocity threshold for destinguish the motion or no=motion of the hand
int mocapRate=200;                                               // the sample rate of the motion capture system

std::vector<double> mocapPosition(3,0);                          // vector for the position of the hand
std::vector<double> mocapVelocity(3,0);                          // vector for the velocity of the hand

std::vector< std::vector<double> > mocapHistoryPosition(3);      // vector for the position of the marker on hand
std::vector< std::vector<double> > mocapHistoryVelocity(3);      // vector for velocity of the marker on hand
std::vector<double> velocityNormHistory;                         // vector for the norm of the velocity of the hand

std::vector<double> mocapTime;                                   // timestamp for the mocap system
std::vector<double> checkVelocityHistory; // history of the velocity checking


int sRate=300;                                                   // set the sample rate (Hz)
int trialCounter=0;                                              // counter for the trials
double startTime;


    Eigen::Vector3f _x;      // Current position [m] (3x1)
//    Eigen::Vector3f _x0;     // Initial end effector postion (3x1)
//    Eigen::Vector3f _v;      // Current end effector velocity [m/s] (3x1)
    Matrix3 _wRb;    // Current rotation matrix (3x3)
    Eigen::Vector4f _q;      // Current end effector quaternion (4x1)

// ----------------- variables for the object -----------------

	double objectPoseStamp=0;
	float _objectZOffset=  0;
	float _objectYOffset= 0;
	float _objectXOffset= 0;
	float offset_orient_y= 0.0;
 				

	Eigen::Vector3f offset_world;											// offset in the world frame
	Eigen::Vector3f offset_object;

	Eigen::Vector3f hand_link;

	Eigen::Vector3f ee_offset_transformed;						// offset in the end effector frame

	Eigen::VectorXd object_position(3);								// the position of the object as received from the mocap system

	Eigen::VectorXd object_orientation(4);							// the orientation of the object as received from the mocap system

	Eigen::Vector4f object_orientation4f;

	Eigen::VectorXd object_position_filtered(3);					// the filtered position of object

	Eigen::VectorXd object_orientation_filtered(4);					// the filtered orientation of the object


// helper variables for the filtering

	MathLib::Vector object_position_filtered_mathlib;
	MathLib::Vector object_velocity_filted_mathlib;

	MathLib::Vector object_orientation_filtered_mathlib;
	MathLib::Vector object_angular_vel_filted_mathlib;

    CDDynamics *object_position_filter, *object_orientation_filter; // filters for the position and orientation of the object


    // Store the current and desired joint states.
    sensor_msgs::JointState current_joint_state;
    sensor_msgs::JointState desired_joint_state;
    Eigen::Vector3f mtarget_pos;
    Eigen::Vector4f mtarget_orient;
    Eigen::Matrix3f mtarget_rotation;
    Eigen::Vector3f mtarget_zaxis, mtarget_yaxis, mtarget_xaxis;

    //if using gaze tracking package
	Eigen::VectorXf _targetPosition(3);
	Eigen::VectorXf _targetOrientation(4);
	Eigen::MatrixXf _targetRotMatrix;

	Eigen::VectorXf desiredNextPosition(3);


	int grasp_type =0;


std::vector<double> calvAverageVelocity(std::vector< std::vector<double> > mVel, int samplesBack){
    /*
     * This function calculates the average velocity in a timw window of 'samplesBack' samples.
     *
     */

    std::vector<double> averVel(mVel.size(),0);

    if (samplesBack<(int)mVel[0].size()){
        for(int i=0;i<samplesBack;i++){
            for(int j=0;j<(int)mVel.size();j++){
                averVel[j]+=mVel[j][(int)mVel[j].size()-i];
            }
        }
        for(int j=0;j<(int)mVel.size();j++){
            averVel[j]=averVel[j]/samplesBack;
        }
    }else{
        for(int i=0;i<(int)mVel[0].size();i++){
            for(int j=0;j<(int)mVel.size();j++){
                averVel[j]+=mVel[j][(int)mVel[j].size()-i];
            }
        }
        for(int j=0;j<(int)mVel.size();j++){
            averVel[j]=averVel[j]/((int)mVel[0].size());
        }
    }
    return averVel;
}




std::vector<double> calcDtVelocity(std::vector<double> currentPos,std::vector<double> oldPos, double dt){
    /*
     * This function calculates the instantaneous velocity between two consecutive samples
     * Inputs:
     *
     *      currentPos: the current position
     *      oldPos:     the previous position
     *      dt:         time between samples (1/samplerate)
     *
     * Output:
     *
     *      a vector with the velocities to all the directions
     *
     */


    std::vector<double> velocity(currentPos.size(),0);

    for(int i=0;i<(int)currentPos.size();i++){
        velocity[i]=(currentPos[i]-oldPos[i])/dt;
    }

    return velocity;
}



double velocityNorm(std::vector< std::vector<double> > mVel, int samplesBack){
    /*
     * This function calculates the first norm of the average velocity in a time window of samplesBack' samples.
     *
     */

    std::vector<double> avVel=calvAverageVelocity(mVel, samplesBack);

    double norm=0;
    for(int i=0;i<(int)avVel.size();i++){
        norm+=avVel[i]*avVel[i];
    }

    return sqrt(norm);

}


bool check_velocity(double velocity, double threshold){
    /*
     * This function checks if the velocity of an object is larger than a threshold
     *
     */

    //std::cout<<"check vel okokkokoko\n";


    if(velocity>=threshold){
        return true;
    }else{
        return false;
    }

}

void targetListener(const geometry_msgs::Pose::ConstPtr& msg){

    //_msgRealPose = *msg;

	offset_object = Utils::quaternionToRotationMatrix(object_orientation4f) * offset_world;

    _targetPosition << -msg->position.x +offset_object[0], -msg->position.y + offset_object[1], msg->position.z+ offset_object[2];
      object_orientation << msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z;
//    _targetRotMatrix = Utils::quaternionToRotationMatrix(_targetOrientation);





    if(!_firstTargetPoseReceived)
    {
        _firstTargetPoseReceived = true;
        ROS_INFO("Target Pose received\n");

        
    }
}

//IIWA KUKA subscriber callback function for pose
void robotListener(const geometry_msgs::Pose::ConstPtr& msg){

	//_msgRealPose = *msg;


	if(!_firstRealPoseReceived)
	{
		_firstRealPoseReceived = true;
		ROS_INFO("Robot Pose received\n");
		// _targetPosition[0]=_eePosition[0]-0.1;
		// _targetPosition[1]=_eePosition[1]+0.05;
		// _targetPosition[2]=_eePosition[2]-0.05;

		
	}
	 geometry_msgs::Pose _msgRealPose = *msg;

	_x << _msgRealPose.position.x, _msgRealPose.position.y, _msgRealPose.position.z;
	_q << _msgRealPose.orientation.w, _msgRealPose.orientation.x, _msgRealPose.orientation.y, _msgRealPose.orientation.z;
	


}



void graspListener(const std_msgs::Int16::ConstPtr& msg)
{
	grasp_type = msg->data;

	if(!_graspTypeReceived)
	{
		_graspTypeReceived = true;
		ROS_INFO("Grasp type received\n");
	}
}



void handListener(const geometry_msgs::PoseStamped& mocapmsg){

    /*-- Callback function for subscriber of the mocap system --*/

    mocapPosition[0]=mocapmsg.pose.position.x;
    mocapPosition[1]=mocapmsg.pose.position.y;
    mocapPosition[2]=mocapmsg.pose.position.z;

    if(!_firstHandPoseReceived){
        _firstHandPoseReceived=true;
        ROS_INFO("Initial hand pose received\n");
    }


    mocapTime.push_back((ros::Time::now().toSec())-startTime);

    for(int i=0;i<3;i++){

        mocapHistoryPosition[i].push_back(mocapPosition[i]);

    }

    if(mocapCounter>0){
       std::vector<double> previousSample(3,0);
       for(int i=0;i<3;i++){
           previousSample[i]=mocapHistoryPosition[i][mocapCounter-1];
       }

       //std::cout<<"x: "<<previousSample[0]<<", y: "<<previousSample[1]<<",z: "<<previousSample[2]<<"\n";
       mocapVelocity=calcDtVelocity(mocapPosition,previousSample,(double)1/std::min((double)sRate,(double)mocapRate));

        for(int i=0;i<3;i++){
            mocapHistoryVelocity[i].push_back(mocapVelocity[i]);
        }
        velocityNormHistory.push_back(velocityNorm(mocapHistoryVelocity,(int)(lookBack*sRate)));
        //std::cout<<"\nvel: "<<velocityNormHistory.back()<<"\n";
        //checkVelocityHistory.push_back(check_velocity(velocityNormHistory.back(),velThreshold));
        //checkVelocityHistory.push_back(1);
       // std::cout<<"velocity: " << check_velocity(velocityNormHistory.back(),velThreshold) << "\n";// << velocityNormHistory.back() << " "



    }


    mocapCounter++;
    //ROS_INFO("I heard: [%d] messages from mocap\n", mocapCounter);
}
