#include "robocds.h"
#include "GMR.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "mathlib/MathLib.h"
#include "Utils.h"
#include "CDDynamics.h"

//#include "mathlib_eigen_conversions.h"


#include "Eigen/Eigen"

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/JointState.h"
#include <dynamic_reconfigure/server.h>
#include "geometry_msgs/PointStamped.h"

#include <sstream>

#include <dynamic_reconfigure/server.h>
#include <robocds/robocdsConfig.h>

#define RADIANS_TO_DEGREES(radians) ((radians) * (180.0 / M_PI))
#define DEGREES_TO_RADIANS(angle) ((angle) / 180.0 * M_PI)

#define DOF_JOINTS 16					//Degrees of freedom of allegro hand
#define DOF_HAND  5						//Degrees of freedom used for training



    Eigen::Vector3f _x;      // Current position [m] (3x1)
//    Eigen::Vector3f _x0;     // Initial end effector postion (3x1)
//    Eigen::Vector3f _v;      // Current end effector velocity [m/s] (3x1)
    Matrix3 _wRb;    // Current rotation matrix (3x3)
    Eigen::Vector4f _q;      // Current end effector quaternion (4x1)
    bool firstrosread = false;
    bool initOK=false;
    bool target_reached = false;
	double dim; // dimension of the sg filter
	float beta= 0.5;
	float alpha = 3.0;
	int grasp_type_temp = 0;



// ----------------- variables for the robot base -----------------

	bool _firstKukaBasePoseReceived=false;
	double robotBaseStamp=0;

	Eigen::VectorXd robot_base_position(3);							// the position of the robot's base as received from the mocap system

	Eigen::VectorXd robot_base_position_filtered(3);				// the filtered position of the robot's base

// helper variables for the filtering

	MathLib::Vector robot_base_position_filtered_mathlib;
	MathLib::Vector robot_base_velocity_filted_mathlib;

	CDDynamics *robot_base_pos_filter;								// the filter for the robot-base position


// ----------------- variables for the object -----------------

	bool _firstObjectPoseReceived=false;
	bool _firstTargetPoseReceived=false;
	bool _firstRealPoseReceived=false;
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


    // Rotational transform from End effector to allegro hand palm

    Eigen::Matrix3f EEtoAllegro;
    // EEtoAllegro<<1,0,0,0,0,1,0,-1,0;



    double home_position[3] = {-0.2, 0, 0.8};							// home postion of the end effector
    bool gohome = false;

	double home_pose[DOF_JOINTS] =
        {
                // Default (HOME) position (degrees), set at system start if
                // no 'initial_position.yaml' parameter is loaded.
                0.0, -10.0, 45.0, 45.0,  0.0, -10.0, 45.0, 45.0,
                5.0, -5.0, 50.0, 45.0, 60.0, 25.0, 15.0, 45.0
        };

    double hand_zero[DOF_JOINTS] = 
    	{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    double grasp_pose[DOF_JOINTS] =
    	{
    		-0.13904085296273158, 0.4826187068503408, 1.6620208902037987, 1.2789904358910884, 
    		-0.39427619217207055, 0.48878076617779026, 1.4955315609244169, 1.0788449536502536, 
    		-0.5396639991517933, 1.1277528199553202, 0.6679068288464648, 1.6920909992559443, 
    		1.2181386670994019, 0.23680379916656313, 1.1065501991473703, 1.064341795561589
    	};

    double home_orient[4] = {  0.678774434852, 0.0393797267332, -0.733290070573,  -0.000419657176474};
 

Eigen::VectorXd M2E_v(MathLib::Vector mm) {
  Eigen::Map<Eigen::VectorXd> ei( mm.Array(), mm.Size() );

  return ei;
}

//function to rescale joint positions of fingers from the sensor data to allegro hand
void rescale_finger(double *raw, double *processed)

{
	double raw_min[DOF_HAND] = {136, 229, 168, 92, 132};			
	double raw_max[DOF_HAND] = {99, 58, 52, 75, 56};
	double allegro_max[DOF_HAND] = {1.49, 1.744, 1.8, 1.73, 1.71};			//thumb0, thumb3, index2, index1, index3
	double allegro_min[DOF_HAND] = {0, 0, 0, 0, 0};

    for(int i =0; i < DOF_HAND; i++)
    	processed[i] = (allegro_max[i] - allegro_min[i])* (raw[i] - raw_min[i])/(raw_max[i] - raw_min[i]);
}


MathLib::Vector E2M_v(Eigen::VectorXd ev) 
{
    MathLib::Vector mv( ev.rows() );

  mv.Set( ev.data(), ev.rows() );
  return mv;
}


void CouplingFunction(double* input, double* output)
  {
    output[0] = sqrt(input[0]*input[0]+input[1]*input[1]+input[2]*input[2]);
  }



//-------------------------Dynamic reconfigure callback function --------------

void callback(robocds::robocdsConfig &config, uint32_t level) {
//  ROS_INFO("Reconfigure Request: %f %f %f %f %f", 
//           config.offset_x, config.offset_y, 
//          config.offset_z, config.beta, config.alpha);
//  offset_world[0] = config.offset_x;
//  offset_world[1] = config.offset_y;
//  offset_world[2] = config.offset_z;
  
  offset_world << config.offset_x, config.offset_y, config.offset_z;
  hand_link	<< config.hand_x, config.hand_y, config.hand_z;
  offset_orient_y = config.offset_orient;
 

  beta = config.beta;
  alpha = config.alpha;
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
		grasp_type_temp = msg->data;


}



void updateJointStates(const sensor_msgs::JointState  &joint_state_msg)
{
	current_joint_state = joint_state_msg;
}







int main(int argc, char **argv)

{	


	int temp =0;
	int count =0;
	double dt = 0.01;
	float speed = 0;
	float quat_mag = 1.0;
	float slerp_t = 0.5;
	float vel_scale =0.5;
	float distance = 0.0;
	float dist_xy = 0.0;
	float initial_distance = 0.0;
	double input[1] = {0.01};
	int l_rate = 200;
	int grasp_type =0;
	GMRDynamics *master_arm = new GMRDynamics("data/masterGMM_pos.txt");
	master_arm->printInfo();

	GMRDynamics *slave_hand = new GMRDynamics("data/slaveGMM_orient.txt");
	slave_hand->printInfo();

	GMRDynamics *slave_fingers = new GMRDynamics("data/slaveGMM_fingers.txt");
	CDDynamics *finger_dynamics = new CDDynamics(1, dt, 100);

	GMR *coupling_hand = new GMR("data/cplGMM_pos_orientv2.txt");

	GMR *coupling_fingers;

	
	cout<<"\nGrasp type (1 simple, 2 pinch, 3 lateral):";
	cin>>grasp_type;

	if(grasp_type == 3)
		coupling_fingers = new GMR("data/cplGMM_pos_finger_lateral.txt");
	else
		coupling_fingers = new GMR("data/cplGMM_pos_finger_simple.txt");

//--------------INITIALIZE COUPLING_HAND GMR ----------------------
/*
	std::vector<int> in_dim, out_dim;
	in_dim.resize(2);
	out_dim.resize(4);

//	if(in_dim.size() + out_dim.size() != coupling->getnVar())
//		return false;

	for(unsigned int i=0;i<in_dim.size();i++)
		in_dim[i] = i;

	for(unsigned int i=0;i<out_dim.size();i++)
		out_dim[i] = i + in_dim.size();

	// setup CDS GMR
	coupling_hand->initGMR(in_dim,out_dim);

	coupling_hand->printInfo();

*/
//--------------INITIALIZE COUPLING_FINGERS GMR ----------------------

	std::vector<int> in_dim2, out_dim2;
	in_dim2.resize(1);
	out_dim2.resize(DOF_HAND);

//	if(in_dim.size() + out_dim.size() != coupling->getnVar())
//		return false;

	for(unsigned int i=0;i<in_dim2.size();i++)
		in_dim2[i] = i;

	for(unsigned int i=0;i<out_dim2.size();i++)
		out_dim2[i] = i + in_dim2.size();

	// setup CDS GMR
	coupling_fingers->initGMR(in_dim2,out_dim2);

	coupling_fingers->printInfo();



//----------------USER INPUT ----------------------------------------

//	double mrel_vel[3], mrel_pos[3], mcurrent_pos[3];
	Eigen::Vector3f mrel_vel, mrel_pos, mcurrent_pos;
	double temp1[3], temp2[3];

	double starget_quaternion[4], scurrent_quaternion[4], sdesired_quaternion[4], srel_quaternion[4];
	Eigen::Vector4f slerp_quaternion;
	Eigen::Vector3f target;
	double Psix[2];
	REALTYPE target_orien[9], target_pos[3];
	float JOINT_LIMIT = 0.2;


	double fingers_current[DOF_HAND], fingers_desired[DOF_HAND], fingers_relpos[DOF_HAND];
	double fingers_relvel[DOF_HAND], fingers_nextpos[DOF_HAND];
	Vector finger_temp(1);
	
//	std::cout<<"\n Enter target translation:(vector 3)"<<endl;
//	for(int i=0; i<3; i++)
//		std::cin>>mtarget_pos[i];

//	Vector mtarget_pos(target_pos,3);

//	std::cout<<	"\n Enter target orientation:(matrix 3)"<<endl;
//	for(int i=0; i<9; i++)
//		std::cin>>target_orien[i];

//	Matrix3 starget_orien(target_orien);

//	starget_orien.GetQuaternionRepresentation(starget_quaternion);

//-----------------USER INPUT END-------------------------------------------

//-----------------ROS TOPICS HANDLERS---------------------------------------
	
    geometry_msgs::Pose _msgRealPose, _msgDesiredPose;
    geometry_msgs::Quaternion _msgDesiredOrientation;
    geometry_msgs::Twist _msgDesiredTwist;
//    std_msgs::Float64MultiArray _desiredJoints;

	ros::init(argc, argv, "robocds");

	dynamic_reconfigure::Server<robocds::robocdsConfig> server;
  	dynamic_reconfigure::Server<robocds::robocdsConfig>::CallbackType f;

  	f = boost::bind(&callback, _1, _2);
  	server.setCallback(f);
	
	ros::NodeHandle n1;

//  For KUKA LWR
//	ros::Publisher desiredtwist = n1.advertise<geometry_msgs::Twist>("/lwr/joint_controllers/passive_ds_command_vel", 400);
// 	ros::Publisher desiredOrientation = n1.advertise<geometry_msgs::Quaternion>("/lwr/joint_controllers/passive_ds_command_orient", 400);

//	For KUKA IIWA
	ros::Publisher _pubDesiredPose =n1.advertise<geometry_msgs::Pose>("IIWA/Desired_E_Pos", 10);


//	For Allegro Hand without robot haptic controller
// 	ros::Publisher jointcmd_pub = n1.advertise<sensor_msgs::JointState>("/allegroHand_0/joint_cmd", 400);

//	For allegro hand with robot haptic controller
 	ros::Publisher jointcmd_pub = n1.advertise<sensor_msgs::JointState>("/desired_hand_state", 400);



// 	ros::Publisher _pubDesiredJoints = n1.advertise<std_msgs::Float64MultiArray>("lwr/joint_controllers/command_joint_pos", 10);


// 	For KUKA LWR 
//	ros::Subscriber subRealTwist = n1.subscribe("/lwr/ee_vel", 1, updateRealTwist);
//	ros::Subscriber subRealPose = n1.subscribe("/lwr/ee_pose", 100, updateRealPose);
	ros::Subscriber jointstates_sub=n1.subscribe("allegroHand_1/joint_states", 100, updateJointStates);


//	If using mocap optitrack
//  ros::Subscriber objectSub=n1.subscribe("/object/pose", 100, objectListener);
//	ros::Subscriber robotBaseSub=n1.subscribe("/Robot/pose", 100, robotBaseListener);


//	If using gaze tracking
	ros::Subscriber targetSub=n1.subscribe("target", 10, targetListener);

//	Subscriber and Publisher for IIWA robot
	ros::Subscriber robotSub=n1.subscribe("IIWA/Real_E_Pos", 10, robotListener);

	ros::Subscriber graspSub = n1.subscribe("EMGinterfaceInt/grasp_type", 10, graspListener);

//	ros::Subscriber graspSub = n1.subscribe("")

	ros:: Rate loop_rate(200);

	while(!_firstRealPoseReceived)
	{
			ros::spinOnce();
			loop_rate.sleep();
	}

//--------------------ROS TOPICS HANDLERS END----------------------------------

		ROS_INFO("Success flag!");

//	mcurrent_pos = E2M_v(_x);
	ee_offset_transformed = Utils::quaternionToRotationMatrix(_q) * hand_link;

	cout<<"\n Hand link Offset in EE frame: \n"<< hand_link<<endl;
	cout<<"Hand link offset in world frame: \n"<< ee_offset_transformed<<endl;

	mcurrent_pos = _x;// + ee_offset_transformed;
//	mtarget_pos = object_position - robot_base_position;

//if using mocap optitrack
//	mtarget_pos[0] = object_position(0) - robot_base_position(0);
//	mtarget_pos[1] = object_position(1) - robot_base_position(1);
//	mtarget_pos[2] = object_position(2) - robot_base_position(2);

//if using gaze tracking



	mtarget_pos[0] = _targetPosition(0);
	mtarget_pos[1] = _targetPosition(1);
	mtarget_pos[2] = _targetPosition(2);

	std::cout<<"\nInitial_ee_pose Translation:"<<endl;
	for(int i =0;i<3;i++)
			cout<<mcurrent_pos[i]<< ", ";
	for(int i=0;i<3;i++)
		mrel_pos[i] = mtarget_pos[i] - mcurrent_pos[i] ;

	for(int i =0;i<3;i++)
		{
//			cout<<mrel_pos[i]<< ", ";
			initial_distance += mrel_pos[i]*mrel_pos[i];
		}
	initial_distance = sqrt(initial_distance);




	desired_joint_state.position.resize(DOF_JOINTS);
	desired_joint_state.velocity.resize(DOF_JOINTS);
	desired_joint_state.effort.resize(DOF_JOINTS);
	desired_joint_state.name.resize(DOF_JOINTS);

	
	



	// filter parameters
	double wn_filter_position, wn_filter_velocity, wn_filter_c, sample_time;
    wn_filter_position = 10.0;
    wn_filter_velocity = 200.0;//30
	wn_filter_c = 25.0;
	dim =3;
	// robot-base's position filter
	robot_base_pos_filter= new  CDDynamics(dim, sample_time, wn_filter_position);

	robot_base_pos_filter->SetStateTarget(E2M_v(robot_base_position), E2M_v(robot_base_position));

	// object's position filter
	object_position_filter= new  CDDynamics(dim, sample_time, wn_filter_position);

	object_position_filter->SetStateTarget(E2M_v(object_position), E2M_v(object_position));

	// object's orientation filter
	object_orientation_filter= new  CDDynamics(4, sample_time, wn_filter_position);

	object_orientation_filter->SetStateTarget(E2M_v(object_orientation), E2M_v(object_orientation));

	initOK=true;

	ROS_INFO("Initialization complete\n");

	cout<<"Slerp_quaternion rescale factor (<1):";
	cin>> input[0];

	while(grasp_type_temp == 0)
	{
			ROS_INFO("I heard grasp type: [%i]", grasp_type_temp);

			ros::spinOnce();
			loop_rate.sleep();
	}

	grasp_type = grasp_type_temp;


	while(ros::ok())
	{

		cout<<"grasp_type ="<<grasp_type<<endl;

//		mcurrent_pos = E2M_v(_x);

//------------ Transforming offset values from world frame to EE frame


//	cout<<"Transfromation matrix: \n"<< Utils::quaternionToRotationMatrix(_q).transpose();

	gohome = false;
	ee_offset_transformed = Utils::quaternionToRotationMatrix(_q) * hand_link;

	cout<<"\n Hand link Offset in EE frame: \n"<< hand_link<<endl;
	cout<<"Hand link offset in world frame: \n"<< ee_offset_transformed<<endl;


	mcurrent_pos[0] = _x[0]+ ee_offset_transformed(0);	
	mcurrent_pos[1] = _x[1] + ee_offset_transformed(1);
	mcurrent_pos[2] = _x[2] + ee_offset_transformed(2);

//if using mocap optitrack
//	mtarget_pos[0] = object_position(0) - robot_base_position(0) ;
//	mtarget_pos[1] = object_position(1) - robot_base_position(1) ;
//	mtarget_pos[2] = object_position(2) - robot_base_position(2) ;

//if using gaze tracking

	mtarget_pos[0] = _targetPosition(0);
	mtarget_pos[1] = _targetPosition(1);
	mtarget_pos[2] = _targetPosition(2);



	for(int i=0;i<3;i++)
		mrel_pos[i] = mtarget_pos[i] - mcurrent_pos[i] ;
	distance = 0.0;
		for(int i =0;i<3;i++)
		{
//			cout<<mrel_pos[i]<< ", ";
			distance += mrel_pos[i]*mrel_pos[i];
		}
	distance = sqrt(distance);
	std::cout<<"\nDistance from the target:"<<distance<<endl;
	std::cout<<"\nInitial distance: "<<initial_distance;



/*
	mtarget_orient[0] = object_orientation(0);
	mtarget_orient[1] = object_orientation(1);
	mtarget_orient[2] = object_orientation(2);
	mtarget_orient[3] = object_orientation(3);
*/

//------------------ Constructing target orientation from mrel_pos and object orientation

// if using mocap optitrack
	object_orientation4f << object_orientation(0), object_orientation(1), object_orientation(2), object_orientation(3);

// if using gaze tracking

//	object_orientation4f << _targetOrientation(0), _targetOrientation(1), _targetOrientation(2), _targetOrientation(3);

	if(distance > 0.1)			//stop changing desired orientation when near the object
	{
			if(grasp_type == 3)			//lateral grasp
			{
				mtarget_zaxis = - Utils::quaternionToRotationMatrix(object_orientation4f).col(0);		//hand pointed along x-axis of object
				dist_xy = sqrt(mrel_pos[0]*mrel_pos[0] + (mrel_pos[1] -offset_orient_y)*(mrel_pos[1] -offset_orient_y));	//z-axis ofEE in xy plane
																													//pointing slightly away from object

				mtarget_xaxis << mrel_pos[0]/dist_xy, (mrel_pos[1] -offset_orient_y)/dist_xy, 0;
//			mtarget_zaxis = mtarget_zaxis - mtarget_zaxis.dot(mtarget_xaxis) * mtarget_xaxis;
//			mtarget_zaxis = mtarget_zaxis/mtarget_zaxis.norm();

				mtarget_xaxis = mtarget_xaxis - mtarget_xaxis.dot(mtarget_zaxis) * mtarget_zaxis;
				mtarget_xaxis = mtarget_xaxis/mtarget_xaxis.norm();
				mtarget_yaxis = mtarget_zaxis.cross(mtarget_xaxis);


			}
			else
			{
									//simple grasp
				mtarget_xaxis =  Utils::quaternionToRotationMatrix(object_orientation4f).col(0);		//hand along x-axis of object

				dist_xy = sqrt(mrel_pos[0]*mrel_pos[0] + (mrel_pos[1] -offset_orient_y)*(mrel_pos[1] -offset_orient_y));	//z-axis ofEE in xy plane
																													//pointing slightly away from object

				mtarget_zaxis << mrel_pos[0]/dist_xy, (mrel_pos[1] -offset_orient_y)/dist_xy, 0;
//			mtarget_zaxis = mtarget_zaxis - mtarget_zaxis.dot(mtarget_xaxis) * mtarget_xaxis;
//			mtarget_zaxis = mtarget_zaxis/mtarget_zaxis.norm();

				mtarget_zaxis = mtarget_zaxis - mtarget_zaxis.dot(mtarget_xaxis) * mtarget_xaxis;
				mtarget_zaxis = mtarget_zaxis/mtarget_zaxis.norm();
				mtarget_yaxis = mtarget_zaxis.cross(mtarget_xaxis);

			}


	}



	mtarget_rotation.col(0)<< -mtarget_xaxis;		//EE_x = Allegro_x
	mtarget_rotation.col(1)<< -mtarget_zaxis;		//EE_y = Allegro_z
	mtarget_rotation.col(2)<< -mtarget_yaxis;		//EE_z = -Allegro_y

//	EEtoAllegro<<0,1,0,					//rotation transform from ee to allegro palm
//				 -1,0,0,
//				 0,0,1;

	cout<<"\n object rotation : \n"<< Utils::quaternionToRotationMatrix(object_orientation4f);

	cout<<"\n target rotation : \n"<< mtarget_rotation;


	cout<<"\n End effector rotation: \n"<<  Utils::quaternionToRotationMatrix(_q);
	cout<<"\n relative position "<<mrel_pos[0]/distance<<", "<<mrel_pos[1]/distance<<", "<< mrel_pos[2]/distance;



	mtarget_orient =  Utils::rotationMatrixToQuaternion( mtarget_rotation);



//--------------------- OUT OF RANGE ---------------------------------------------
	if(mtarget_pos[0]*mtarget_pos[0] + mtarget_pos[1]*mtarget_pos[1]  + mtarget_pos[2]*mtarget_pos[2] > 1.5)
	{

		ROS_INFO("\n Target out of reach (>0.10)!!! Going home!");
		gohome = true;
		mcurrent_pos[0] = _x[0] ;	
		mcurrent_pos[1] = _x[1] ;
		mcurrent_pos[2] = _x[2] ;

		mtarget_pos[0] = home_position[0] ;
		mtarget_pos[1] = home_position[1] ;
		mtarget_pos[2] = home_position[2] ;

		mtarget_orient[0] = home_orient[0];
		mtarget_orient[1] = home_orient[1];
		mtarget_orient[2] = home_orient[2];
		mtarget_orient[3] = home_orient[3];




	for(int i=0;i<3;i++)
		{
			mrel_pos[i] = mtarget_pos[i] - mcurrent_pos[i] ;
			initial_distance += mrel_pos[i]*mrel_pos[i];
		}

	initial_distance = sqrt(initial_distance);
	}



//	if(mrel_pos[0]*mrel_pos[0]+ mrel_pos[1]*mrel_pos[1]+mrel_pos[2]*mrel_pos[2] < 0.005)
//	{
//		cout<<"\nTarget reached";
//		_msgDesiredTwist.linear.x  = 0;
//		_msgDesiredTwist.linear.y  = 0;
//		_msgDesiredTwist.linear.z  = 0;
//	}
//	else
//	{
		for(int k =0; k <3; k++)
		{
			temp1[k] = -mrel_pos[k];
		}
		master_arm->getGMROutput(temp1, temp2);

		mrel_vel<< temp2[0], temp2[1], temp2[2];


			speed = sqrt(mrel_vel[0]*mrel_vel[0] + mrel_vel[1]*mrel_vel[1] + mrel_vel[2]*mrel_vel[2]) ;
		
		if(speed>vel_scale)
			{
				vel_scale = speed;
				cout<<"\n!!!!High speed:"<<speed;
			}

			mrel_vel[0] = 0.3*mrel_vel[0]/vel_scale;
			mrel_vel[1] = 0.3*mrel_vel[1]/vel_scale;
			mrel_vel[2] = 0.3*mrel_vel[2]/vel_scale;

		speed = sqrt(mrel_vel[0]*mrel_vel[0] + mrel_vel[1]*mrel_vel[1] + mrel_vel[2]*mrel_vel[2]) ;

		if((speed<0.3) && (distance>0.005) && gohome==false)
		{
			mrel_vel[0] = 0.3*mrel_vel[0]/speed;
			mrel_vel[1] = 0.3*mrel_vel[1]/speed;
			mrel_vel[2] = 0.3*mrel_vel[2]/speed;

			cout<<"\n !!!!Low speed:"<<speed;
		}

		if(distance < 0.03)
		{
			target_reached = true;
			ROS_INFO("Target reached (<0.05)!");
		}

		if(distance > 0.05)
			target_reached = false;

		if(target_reached)
		{
			mrel_vel[0] = 0;
			mrel_vel[1] = 0;
			mrel_vel[2] = 0;
		}
		std::cout<<"\nDesired velocity:"<<endl;
		for(int i =0;i<3;i++)
//			cout<<curr_ee_pose.GetTranslation()[i]<< ", ";
		cout<<mrel_vel[i]<< ", ";

		speed = sqrt(mrel_vel[0]*mrel_vel[0] + mrel_vel[1]*mrel_vel[1] + mrel_vel[2]*mrel_vel[2]) ;

		cout<<"Speed: "<<speed<<endl;

//		for KUKA LWR
//		_msgDesiredTwist.linear.x  = mrel_vel[0];
//		_msgDesiredTwist.linear.y  = mrel_vel[1];
//		_msgDesiredTwist.linear.z  = mrel_vel[2];
//		_msgDesiredTwist.angular.x = 0;
//		_msgDesiredTwist.angular.y = 0;
//		_msgDesiredTwist.angular.z = 0;

			desiredNextPosition[0]=mrel_vel[0]*(1.0/50)+ _x[0];
			desiredNextPosition[1]=mrel_vel[1]*(1.0/50)+ _x[1];
			desiredNextPosition[2]=mrel_vel[2]*(1.0/50)+ _x[2];

/*			desiredNextPosition[0]=(mtarget_pos[0]- mcurrent_pos[0])*(0.1)+ _x[0];
			desiredNextPosition[1]=(mtarget_pos[1]- mcurrent_pos[1])*(0.1)+ _x[1];
			desiredNextPosition[2]=(mtarget_pos[2]- mcurrent_pos[2])*(0.1)+ _x[2];
*/


			_msgDesiredPose.position.x=desiredNextPosition[0];
			_msgDesiredPose.position.y=desiredNextPosition[1];
			_msgDesiredPose.position.z=desiredNextPosition[2];


		
//	}

/*
	CouplingFunction(mrel_pos, Psix);

	
	coupling_hand->getGMROutput(Psix, sdesired_quaternion);
/*


//		std::cout<<"\nDesired_ee_pose Translation:"<<endl;
//		for(int i =0;i<3;i++)
//			cout<<des_ee_pose.GetTranslation()[i]<< ", ";
//		std::cout<<"\nDesired_ee_pose Quaternion:"<<endl;
//		for(int i =0;i<4;i++)
//			cout<<des_ee_quaternion[i]<<", ";	

/*	quat_mag = sqrt(sdesired_quaternion[0]*sdesired_quaternion[0] + 
		sdesired_quaternion[1]*sdesired_quaternion[1] + 
		sdesired_quaternion[2] * sdesired_quaternion[2] + 
		sdesired_quaternion[3] * sdesired_quaternion[3]);
*/

/*	slerp_quaternion[0] = sdesired_quaternion[0]/quat_mag;
	slerp_quaternion[1] = sdesired_quaternion[1]/quat_mag;
	slerp_quaternion[2] = sdesired_quaternion[2]/quat_mag;
	slerp_quaternion[3] = sdesired_quaternion[3]/quat_mag;
*/

/*	quat_mag = sqrt(mtarget_orient[0]*mtarget_orient[0] + 
		mtarget_orient[1]*mtarget_orient[1] + 
		mtarget_orient[2] * mtarget_orient[2] + 
		mtarget_orient[3] * mtarget_orient[3]);

	slerp_quaternion[0] = mtarget_orient[0]/quat_mag;
	slerp_quaternion[1] = mtarget_orient[1]/quat_mag;
	slerp_quaternion[2] = mtarget_orient[2]/quat_mag;
	slerp_quaternion[3] = mtarget_orient[3]/quat_mag;
*/
//	slerp_quaternion = Utils::slerpQuaternion(_q, slerp_quaternion, input[0]*(1- input[0]*distance/initial_distance));
	slerp_quaternion = Utils::slerpQuaternion(_q, mtarget_orient, input[0]);


	std::cout<<"\nCurrent Quaternion:"<<endl;
	for(int i =0;i<4;i++)
		cout<<_q[i]<< ", ";
//

	std::cout<<"\nDesired Quaternion:"<<endl;
	for(int i =0;i<4;i++)
		cout<<slerp_quaternion[i]<< ", ";


//	_msgDesiredOrientation.w = slerp_quaternion[0];
//	_msgDesiredOrientation.x = slerp_quaternion[1];
//	_msgDesiredOrientation.y = slerp_quaternion[2];
//	_msgDesiredOrientation.z = slerp_quaternion[3];

	_msgDesiredPose.orientation.w= slerp_quaternion[0];
	_msgDesiredPose.orientation.x= slerp_quaternion[1];
	_msgDesiredPose.orientation.y= slerp_quaternion[2];
	_msgDesiredPose.orientation.z= slerp_quaternion[3];
/*
	_msgDesiredPose.orientation.w= _q[0];
	_msgDesiredPose.orientation.x= _q[1];
	_msgDesiredPose.orientation.y= _q[2];
	_msgDesiredPose.orientation.z= _q[3];
*/
//--------------------------FINGERS POSITION UPDATE-----------------




		fingers_current[0] = current_joint_state.position[12];		//thumb 0
		fingers_current[1] = current_joint_state.position[13];		//thumb 1
		fingers_current[2] = current_joint_state.position[2];		//finger 1.2
		fingers_current[3] = current_joint_state.position[3];		//finger 1.1
		fingers_current[4] = current_joint_state.position[1];		//finger 1.3

//		for(int i =0;i<3;i++)
//			mrel_pos[i] = alpha*mrel_pos[i];



//		CouplingFunction(mrel_pos, Psix);
		Psix[0] = alpha*distance;
		coupling_fingers->getGMROutput(Psix, fingers_desired);

//		cout<<"GMR Input: ";
//		cin>>input[0];
//		coupling_fingers->getGMROutput(input, fingers_desired);

		cout<<"\n Finger desired  position: ";
		for(int i=0; i<DOF_HAND;i++)
			cout<<fingers_desired[i]<<",";


//		finger_temp(0) = fingers_desired[0];
//		finger_dynamics->SetTarget(finger_temp);
//		finger_dynamics->Update();
//		finger_dynamics->GetState(finger_temp);


//		for(int i=0; i<DOF_HAND;i++)
//			fingers_relpos[i] = fingers_desired[i] - fingers_current[i]; 	
//		slave_fingers->getGMROutput(fingers_relpos, fingers_relvel);

//		rescale_finger(fingers_desired, fingers_nextpos);	//fingers next_pos = fingers desired
		for(int i = 0; i<DOF_HAND;i++)
			fingers_nextpos[i] = fingers_desired[i];
		desired_joint_state.position[0] = 0 ;
		desired_joint_state.position[1] =  beta*fingers_nextpos[4] ;
		desired_joint_state.position[2] =  beta*fingers_nextpos[2] ;
		desired_joint_state.position[3] =  beta*fingers_nextpos[3] ;
		desired_joint_state.position[4] = 0;
		desired_joint_state.position[5] = beta*fingers_nextpos[4];
		desired_joint_state.position[6] = beta*fingers_nextpos[2];
		desired_joint_state.position[7] =  beta*fingers_nextpos[3];
		desired_joint_state.position[8] = 0;
		desired_joint_state.position[9] =  beta*fingers_nextpos[4];
		desired_joint_state.position[10] =  beta*fingers_nextpos[2];
		desired_joint_state.position[11] =  beta*fingers_nextpos[3];
		desired_joint_state.position[12] =  beta*fingers_nextpos[0];
		desired_joint_state.position[13] =  beta*fingers_nextpos[1];
		desired_joint_state.position[14] =  beta*fingers_nextpos[2];
		desired_joint_state.position[15] =  beta*fingers_nextpos[3];

		if(grasp_type == 2)			//Pinch grasp
{
		desired_joint_state.position[4] = 0;
		desired_joint_state.position[5] = 0;
		desired_joint_state.position[6] = 0;
		desired_joint_state.position[7] =  0;
		desired_joint_state.position[8] = 0;
		desired_joint_state.position[9] =  0;
		desired_joint_state.position[10] =  0;
		desired_joint_state.position[11] =  0;

}



/*		for(int i=12; i< DOF_JOINTS; i++)
			{
				desired_joint_state.position[i] = (1 - distance/initial_distance)*(1 - distance/initial_distance)*grasp_pose[i];
			}
*/

//		for(int i=0; i< 16; i++)
//		{
//			desired_joint_state.position[i] += grasp_pose[i];
//		}

//		desired_joint_state.position[14] = beta*fingers_nextpos[0]+ grasp_pose[14];
//		desired_joint_state.position[15] = beta*fingers_nextpos[0]+ grasp_pose[15];


//for KUKA IIWA
		_pubDesiredPose.publish(_msgDesiredPose);

// For KUKA LWR
//		desiredtwist.publish(_msgDesiredTwist);

//		desiredOrientation.publish(_msgDesiredOrientation);




if(gohome==true)
{		
		ROS_INFO("Object out of range!!!! Going to home position!");
//		finger_temp(0) = 0;
//		finger_dynamics->SetTarget(finger_temp);
//		finger_dynamics->Update();
//		finger_dynamics->GetState(finger_temp);

		for(int i=0; i< DOF_JOINTS; i++)
			desired_joint_state.position[i] = 0;
//			if(abs(finger_temp(0)-0)<0.01)
//				desired_joint_state.position[i] = finger_temp(0);
//			else
//				desired_joint_state.position[i] = finger_temp(0) ;





}
//		cout<<"beta: "<<beta<<endl;

//		cout<<"\n Finger next position: ";
//		for(int i=0; i<DOF_HAND;i++)
//			cout<<beta*fingers_nextpos[i]*180/3.14<<",";

		cout<<"\n EE Current Position: "<< mcurrent_pos[0]<<", "<< mcurrent_pos[1]<<", "<< mcurrent_pos[2] << endl;

		cout<<"Target: ";
		for(int i=0;i<3;i++)
			cout<<", "<< mtarget_pos[i];
	

		cout<<"\n Desired Next Position: "<< desiredNextPosition(0)<<", "<< desiredNextPosition(1)<<", "<<desiredNextPosition(2)<<endl;	


/*	for(int i=0;i<DOF_JOINTS;i++)
	{
		if(desired_joint_state.position[i]- current_joint_state.position[i]>JOINT_LIMIT)
				desired_joint_state.position[i] =  current_joint_state.position[i] + JOINT_LIMIT ;
		else if(desired_joint_state.position[i]- current_joint_state.position[i]< - JOINT_LIMIT)
				desired_joint_state.position[i] =  current_joint_state.position[i] - JOINT_LIMIT ;
	}
*/
		jointcmd_pub.publish(desired_joint_state);




		ros::spinOnce();
		loop_rate.sleep();


	}
	

}

