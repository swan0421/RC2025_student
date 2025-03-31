/*
 * RoK-3 Gazebo Simulation Code 
 * 
 * Robotics & Control Lab.
 * 
 * Master : BKCho
 * First developer : Yunho Han
 * Second developer : Minho Park
 * 
 * ======
 * Update date : 2022.03.16 by Yunho Han
 * ======
 */
//* Header file for C++
#include <stdio.h>
#include <iostream>
#include <boost/bind.hpp>



//* Header file for Gazebo and Ros
#include <gazebo/gazebo.hh>
#include <ros/ros.h>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <functional>
#include <ignition/math/Vector3.hh>

//* Header file for RBDL and Eigen
#include <rbdl/rbdl.h> // Rigid Body Dynamics Library (RBDL)
#include <rbdl/addons/urdfreader/urdfreader.h> // urdf model read using RBDL
#include <Eigen/Dense> // Eigen is a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.

#define PI      3.141592
#define D2R     PI/180.
#define R2D     180./PI

//Print color
#define C_BLACK   "\033[30m"
#define C_RED     "\x1b[91m"
#define C_GREEN   "\x1b[92m"
#define C_YELLOW  "\x1b[93m"
#define C_BLUE    "\x1b[94m"
#define C_MAGENTA "\x1b[95m"
#define C_CYAN    "\x1b[96m"
#define C_RESET   "\x1b[0m"

//Eigen//
using Eigen::MatrixXd;
using Eigen::VectorXd;

//RBDL//
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

using namespace std;


namespace gazebo
{

    class rok3_plugin : public ModelPlugin
    {
        //*** Variables for RoK-3 Simulation in Gazebo ***//
        //* TIME variable
        common::Time last_update_time;
        event::ConnectionPtr update_connection;
        double dt;
        double time = 0;

        //* Model & Link & Joint Typedefs
        physics::ModelPtr model;

        physics::JointPtr L_Hip_yaw_joint;
        physics::JointPtr L_Hip_roll_joint;
        physics::JointPtr L_Hip_pitch_joint;
        physics::JointPtr L_Knee_joint;
        physics::JointPtr L_Ankle_pitch_joint;
        physics::JointPtr L_Ankle_roll_joint;

        physics::JointPtr R_Hip_yaw_joint;
        physics::JointPtr R_Hip_roll_joint;
        physics::JointPtr R_Hip_pitch_joint;
        physics::JointPtr R_Knee_joint;
        physics::JointPtr R_Ankle_pitch_joint;
        physics::JointPtr R_Ankle_roll_joint;
        physics::JointPtr torso_joint;

        physics::JointPtr LS, RS;

        //* Index setting for each joint
        
        enum
        {
            WST = 0, LHY, LHR, LHP, LKN, LAP, LAR, RHY, RHR, RHP, RKN, RAP, RAR
        };

        //* Joint Variables
        int nDoF; // Total degrees of freedom, except position and orientation of the robot

        typedef struct RobotJoint //Joint variable struct for joint control 
        {
            double targetDegree; //The target deg, [deg]
            double targetRadian; //The target rad, [rad]

            double targetVelocity; //The target vel, [rad/s]
            double targetTorque; //The target torque, [N·m]

            double actualDegree; //The actual deg, [deg]
            double actualRadian; //The actual rad, [rad]
            double actualVelocity; //The actual vel, [rad/s]
            double actualRPM; //The actual rpm of input stage, [rpm]
            double actualTorque; //The actual torque, [N·m]

            double Kp;
            double Ki;
            double Kd;

        } ROBO_JOINT;
        ROBO_JOINT* joint;

    public:
        //*** Functions for RoK-3 Simulation in Gazebo ***//
        void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/); // Loading model data and initializing the system before simulation 
        void UpdateAlgorithm(); // Algorithm update while simulation

        void jointController(); // Joint Controller for each joint

        void GetJoints(); // Get each joint data from [physics::ModelPtr _model]
        void GetjointData(); // Get encoder data of each joint

        void initializeJoint(); // Initialize joint variables for joint control
        void SetJointPIDgain(); // Set each joint PID gain for joint control
    };
    GZ_REGISTER_MODEL_PLUGIN(rok3_plugin);
}

//-----------------------------------------------------------------//
//* Practice 2. Forward Kinematics
//-----------------------------------------------------------------//

MatrixXd jointToTransform01(VectorXd q){
    
    // Frame 0 to Frame 1 : C_01 = C_z(q0)
    
    MatrixXd tmp_m(4,4);
    Vector3d r = {0,0.105,-0.1512};
    double q1 = q(0);
    
    tmp_m(0,0) = cos(q1);   tmp_m(0,1) = -sin(q1);  tmp_m(0,2) = 0; tmp_m(0,3) = r(0);
    tmp_m(1,0) = sin(q1);   tmp_m(1,1) =  cos(q1);  tmp_m(1,2) = 0; tmp_m(1,3) = r(1);
    tmp_m(2,0) = 0;         tmp_m(2,1) = 0;         tmp_m(2,2) = 1; tmp_m(2,3) = r(2);
    tmp_m(3,0) = 0;         tmp_m(3,1) = 0;         tmp_m(3,2) = 0; tmp_m(3,3) = 1;

    //std::cout << tmp_m << endl;
    
    return tmp_m;
}

MatrixXd jointToTransform12(VectorXd q){
    
    // Frame 0 to Frame 1 : C_01 = C_z(q0)
    
    MatrixXd tmp_m(4,4);
    Vector3d r = {0,0,0};
    double qq = q(1);
    
    tmp_m(0,0) = 1; tmp_m(0,1) = 0;         tmp_m(0,2) = 0;         tmp_m(0,3) = r(0);
    tmp_m(1,0) = 0; tmp_m(1,1) = cos(qq);   tmp_m(1,2) = -sin(qq);  tmp_m(1,3) = r(1);
    tmp_m(2,0) = 0; tmp_m(2,1) = sin(qq);   tmp_m(2,2) = cos(qq);   tmp_m(2,3) = r(2);
    tmp_m(3,0) = 0; tmp_m(3,1) = 0;         tmp_m(3,2) = 0;         tmp_m(3,3) = 1;

    //std::cout << tmp_m << endl;
    
    return tmp_m;
}

MatrixXd jointToTransform23(VectorXd q){
    
    // Frame 0 to Frame 1 : C_01 = C_z(q0)
    
    MatrixXd tmp_m(4,4);
    Vector3d r = {0,0,0};
    double qq = q(2);
    
    tmp_m(0,0) = cos(qq);   tmp_m(0,1) = 0;     tmp_m(0,2) = sin(qq);   tmp_m(0,3) = r(0);
    tmp_m(1,0) = 0;         tmp_m(1,1) = 1;     tmp_m(1,2) = 0;         tmp_m(1,3) = r(1);
    tmp_m(2,0) = -sin(qq);  tmp_m(2,1) = 0;     tmp_m(2,2) = cos(qq);   tmp_m(2,3) = r(2);
    tmp_m(3,0) = 0;         tmp_m(3,1) = 0;     tmp_m(3,2) = 0;         tmp_m(3,3) = 1;

    //std::cout << tmp_m << endl;
    
    return tmp_m;
}

MatrixXd jointToTransform34(VectorXd q){
    
    // Frame 0 to Frame 1 : C_01 = C_z(q0)
    
    MatrixXd tmp_m(4,4);
    Vector3d r = {0,0,-0.35};
    double qq = q(3);
    
    tmp_m(0,0) = cos(qq);   tmp_m(0,1) = 0;     tmp_m(0,2) = sin(qq);   tmp_m(0,3) = r(0);
    tmp_m(1,0) = 0;         tmp_m(1,1) = 1;     tmp_m(1,2) = 0;         tmp_m(1,3) = r(1);
    tmp_m(2,0) = -sin(qq);  tmp_m(2,1) = 0;     tmp_m(2,2) = cos(qq);   tmp_m(2,3) = r(2);
    tmp_m(3,0) = 0;         tmp_m(3,1) = 0;     tmp_m(3,2) = 0;         tmp_m(3,3) = 1;

    //std::cout << tmp_m << endl;
    
    return tmp_m;
}

MatrixXd jointToTransform45(VectorXd q){
    
    // Frame 0 to Frame 1 : C_01 = C_z(q0)
    
    MatrixXd tmp_m(4,4);
    Vector3d r = {0,0,-0.35};
    double qq = q(4);
    
    tmp_m(0,0) = cos(qq);   tmp_m(0,1) = 0;     tmp_m(0,2) = sin(qq);   tmp_m(0,3) = r(0);
    tmp_m(1,0) = 0;         tmp_m(1,1) = 1;     tmp_m(1,2) = 0;         tmp_m(1,3) = r(1);
    tmp_m(2,0) = -sin(qq);  tmp_m(2,1) = 0;     tmp_m(2,2) = cos(qq);   tmp_m(2,3) = r(2);
    tmp_m(3,0) = 0;         tmp_m(3,1) = 0;     tmp_m(3,2) = 0;         tmp_m(3,3) = 1;

    //std::cout << tmp_m << endl;
    
    return tmp_m;
}

MatrixXd jointToTransform56(VectorXd q){
    
    // Frame 0 to Frame 1 : C_01 = C_z(q0)
    
    MatrixXd tmp_m(4,4);
    Vector3d r = {0,0,0};
    double qq = q(5);
    
    tmp_m(0,0) = 1; tmp_m(0,1) = 0;         tmp_m(0,2) = 0;         tmp_m(0,3) = r(0);
    tmp_m(1,0) = 0; tmp_m(1,1) = cos(qq);   tmp_m(1,2) = -sin(qq);  tmp_m(1,3) = r(1);
    tmp_m(2,0) = 0; tmp_m(2,1) = sin(qq);   tmp_m(2,2) = cos(qq);   tmp_m(2,3) = r(2);
    tmp_m(3,0) = 0; tmp_m(3,1) = 0;         tmp_m(3,2) = 0;         tmp_m(3,3) = 1;

    //std::cout << tmp_m << endl;
    
    return tmp_m;
}

MatrixXd getTransform6E(){
    MatrixXd tmp_m(4,4);
    Vector3d r = {0,0,-0.09};

    tmp_m = MatrixXd::Identity(4,4);
    
    tmp_m << 1, 0, 0, 0, \
             0, 1, 0, 0, \
             0, 0, 1, 0, \
             0, 0, 0, 1;
    
    tmp_m(0,0) = 1; tmp_m(0,1) = 0; tmp_m(0,2) = 0; tmp_m(0,3) = r(0);
    tmp_m(1,0) = 0; tmp_m(1,1) = 1; tmp_m(1,2) = 0; tmp_m(1,3) = r(1);
    tmp_m(2,0) = 0; tmp_m(2,1) = 0; tmp_m(2,2) = 1; tmp_m(2,3) = r(2);
    tmp_m(3,0) = 0; tmp_m(3,1) = 0; tmp_m(3,2) = 0; tmp_m(3,3) = 1;

    return tmp_m;
}


VectorXd jointToPosition(VectorXd q){
    VectorXd tmp_v = VectorXd::Zero(3);
    MatrixXd tmp_m(4,4);
    
    tmp_m = jointToTransform01(q)* 
            jointToTransform12(q)* 
            jointToTransform23(q)* 
            jointToTransform34(q)* 
            jointToTransform45(q)* 
            jointToTransform56(q)* 
            getTransform6E();
        
    tmp_v = tmp_m.block(0,3,3,1);
    
    return tmp_v;
}

MatrixXd jointToRotMat(VectorXd q){
    MatrixXd tmp_m(3,3);
    MatrixXd T_IE(4,4);
    
    T_IE =  jointToTransform01(q)* 
            jointToTransform12(q)* 
            jointToTransform23(q)* 
            jointToTransform34(q)* 
            jointToTransform45(q)* 
            jointToTransform56(q)* 
            getTransform6E();
    
    tmp_m = T_IE.block(0,0,3,3);
    
    return tmp_m;
}

VectorXd rotToEuler(MatrixXd rotMat){
    // ZYX Euler Angle - yaw-pitch-roll
    VectorXd tmp_v(3);
    
        tmp_v(0) = atan2(rotMat(1,0),rotMat(0,0));
    tmp_v(1) = atan2(-rotMat(2,0),sqrt(rotMat(2,1)*rotMat(2,1)+rotMat(2,2)*rotMat(2,2)));
    tmp_v(2) = atan2(rotMat(2,1),rotMat(2,2));
            
    std::cout << tmp_v << endl;
    
    return tmp_v;
    
}



void Practice()
{
    VectorXd q=VectorXd::Zero(6);
    MatrixXd T0E(4,4);
    MatrixXd C0E(3,3);

    //* Practice 1. Forward Kinematics    
    q << 10,20,30,40,50,60;
    q = q*3.14/180;
    
    T0E = jointToTransform01(q)*jointToTransform12(q)*jointToTransform23(q)*jointToTransform34(q)*jointToTransform45(q)*jointToTransform56(q)*getTransform6E();
    
    C0E = jointToRotMat(q);
    rotToEuler(C0E);
    
    
    std::cout <<"T0E = " << T0E << std::endl;
    jointToTransform01(q);
}




void gazebo::rok3_plugin::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
    /*
     * Loading model data and initializing the system before simulation 
     */

    //* model.sdf file based model data input to [physics::ModelPtr model] for gazebo simulation
    model = _model;

    //* [physics::ModelPtr model] based model update
    GetJoints();



    //* RBDL API Version Check
    int version_test;
    version_test = rbdl_get_api_version();
    printf(C_GREEN "RBDL API version = %d\n" C_RESET, version_test);

    //* model.urdf file based model data input to [Model* rok3_model] for using RBDL
    Model* rok3_model = new Model();
    Addons::URDFReadFromFile("home/user_name/.gazebo/models/rok3_model/urdf/rok3_model.urdf", rok3_model, true, true);
    //↑↑↑ Check File Path ↑↑↑
    nDoF = rok3_model->dof_count - 6; // Get degrees of freedom, except position and orientation of the robot
    joint = new ROBO_JOINT[nDoF]; // Generation joint variables struct

    //* initialize and setting for robot control in gazebo simulation
    initializeJoint();
    SetJointPIDgain();


    //* setting for getting dt
    last_update_time = model->GetWorld()->SimTime();
    update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&rok3_plugin::UpdateAlgorithm, this));
    
    
    //* Kinematics Test
    Practice();
    


}

void gazebo::rok3_plugin::UpdateAlgorithm()
{
    /*
     * Algorithm update while simulation
     */

    //* UPDATE TIME : 1ms
    common::Time current_time = model->GetWorld()->SimTime();
    dt = current_time.Double() - last_update_time.Double();
    //    cout << "dt:" << dt << endl;
    time = time + dt;
    //    cout << "time:" << time << endl;

    //* setting for getting dt at next step
    last_update_time = current_time;


    //* Read Sensors data
    GetjointData();
    
    
    joint[LHY].targetRadian = 10*3.14/180;
    joint[LHR].targetRadian = 20*3.14/180;
    joint[LHP].targetRadian = 30*3.14/180;
    joint[LKN].targetRadian = 40*3.14/180;
    joint[LAP].targetRadian = 50*3.14/180;
    joint[LAR].targetRadian = 60*3.14/180;
    
    
    //* Joint Controller
    jointController();
}

void gazebo::rok3_plugin::jointController()
{
    /*
     * Joint Controller for each joint
     */

    // Update target torque by control
    for (int j = 0; j < nDoF; j++) {
        joint[j].targetTorque = joint[j].Kp * (joint[j].targetRadian-joint[j].actualRadian)\
                              + joint[j].Kd * (joint[j].targetVelocity-joint[j].actualVelocity);
    }

    // Update target torque in gazebo simulation     
    L_Hip_yaw_joint->SetForce(0, joint[LHY].targetTorque);
    L_Hip_roll_joint->SetForce(0, joint[LHR].targetTorque);
    L_Hip_pitch_joint->SetForce(0, joint[LHP].targetTorque);
    L_Knee_joint->SetForce(0, joint[LKN].targetTorque);
    L_Ankle_pitch_joint->SetForce(0, joint[LAP].targetTorque);
    L_Ankle_roll_joint->SetForce(0, joint[LAR].targetTorque);

    R_Hip_yaw_joint->SetForce(0, joint[RHY].targetTorque);
    R_Hip_roll_joint->SetForce(0, joint[RHR].targetTorque);
    R_Hip_pitch_joint->SetForce(0, joint[RHP].targetTorque);
    R_Knee_joint->SetForce(0, joint[RKN].targetTorque);
    R_Ankle_pitch_joint->SetForce(0, joint[RAP].targetTorque);
    R_Ankle_roll_joint->SetForce(0, joint[RAR].targetTorque);

    torso_joint->SetForce(0, joint[WST].targetTorque);
}

void gazebo::rok3_plugin::GetJoints()
{
    /*
     * Get each joints data from [physics::ModelPtr _model]
     */

    //* Joint specified in model.sdf
    L_Hip_yaw_joint = this->model->GetJoint("L_Hip_yaw_joint");
    L_Hip_roll_joint = this->model->GetJoint("L_Hip_roll_joint");
    L_Hip_pitch_joint = this->model->GetJoint("L_Hip_pitch_joint");
    L_Knee_joint = this->model->GetJoint("L_Knee_joint");
    L_Ankle_pitch_joint = this->model->GetJoint("L_Ankle_pitch_joint");
    L_Ankle_roll_joint = this->model->GetJoint("L_Ankle_roll_joint");
    R_Hip_yaw_joint = this->model->GetJoint("R_Hip_yaw_joint");
    R_Hip_roll_joint = this->model->GetJoint("R_Hip_roll_joint");
    R_Hip_pitch_joint = this->model->GetJoint("R_Hip_pitch_joint");
    R_Knee_joint = this->model->GetJoint("R_Knee_joint");
    R_Ankle_pitch_joint = this->model->GetJoint("R_Ankle_pitch_joint");
    R_Ankle_roll_joint = this->model->GetJoint("R_Ankle_roll_joint");
    torso_joint = this->model->GetJoint("torso_joint");

    //* FTsensor joint
    LS = this->model->GetJoint("LS");
    RS = this->model->GetJoint("RS");
}

void gazebo::rok3_plugin::GetjointData()
{
    /*
     * Get encoder and velocity data of each joint
     * encoder unit : [rad] and unit conversion to [deg]
     * velocity unit : [rad/s] and unit conversion to [rpm]
     */
    joint[LHY].actualRadian = L_Hip_yaw_joint->Position(0);
    joint[LHR].actualRadian = L_Hip_roll_joint->Position(0);
    joint[LHP].actualRadian = L_Hip_pitch_joint->Position(0);
    joint[LKN].actualRadian = L_Knee_joint->Position(0);
    joint[LAP].actualRadian = L_Ankle_pitch_joint->Position(0);
    joint[LAR].actualRadian = L_Ankle_roll_joint->Position(0);

    joint[RHY].actualRadian = R_Hip_yaw_joint->Position(0);
    joint[RHR].actualRadian = R_Hip_roll_joint->Position(0);
    joint[RHP].actualRadian = R_Hip_pitch_joint->Position(0);
    joint[RKN].actualRadian = R_Knee_joint->Position(0);
    joint[RAP].actualRadian = R_Ankle_pitch_joint->Position(0);
    joint[RAR].actualRadian = R_Ankle_roll_joint->Position(0);

    joint[WST].actualRadian = torso_joint->Position(0);

    for (int j = 0; j < nDoF; j++) {
        joint[j].actualDegree = joint[j].actualRadian*R2D;
    }


    joint[LHY].actualVelocity = L_Hip_yaw_joint->GetVelocity(0);
    joint[LHR].actualVelocity = L_Hip_roll_joint->GetVelocity(0);
    joint[LHP].actualVelocity = L_Hip_pitch_joint->GetVelocity(0);
    joint[LKN].actualVelocity = L_Knee_joint->GetVelocity(0);
    joint[LAP].actualVelocity = L_Ankle_pitch_joint->GetVelocity(0);
    joint[LAR].actualVelocity = L_Ankle_roll_joint->GetVelocity(0);

    joint[RHY].actualVelocity = R_Hip_yaw_joint->GetVelocity(0);
    joint[RHR].actualVelocity = R_Hip_roll_joint->GetVelocity(0);
    joint[RHP].actualVelocity = R_Hip_pitch_joint->GetVelocity(0);
    joint[RKN].actualVelocity = R_Knee_joint->GetVelocity(0);
    joint[RAP].actualVelocity = R_Ankle_pitch_joint->GetVelocity(0);
    joint[RAR].actualVelocity = R_Ankle_roll_joint->GetVelocity(0);

    joint[WST].actualVelocity = torso_joint->GetVelocity(0);


    //    for (int j = 0; j < nDoF; j++) {
    //        cout << "joint[" << j <<"]="<<joint[j].actualDegree<< endl;
    //    }

}

void gazebo::rok3_plugin::initializeJoint()
{
    /*
     * Initialize joint variables for joint control
     */
    
    for (int j = 0; j < nDoF; j++) {
        joint[j].targetDegree = 0;
        joint[j].targetRadian = 0;
        joint[j].targetVelocity = 0;
        joint[j].targetTorque = 0;
        
        joint[j].actualDegree = 0;
        joint[j].actualRadian = 0;
        joint[j].actualVelocity = 0;
        joint[j].actualRPM = 0;
        joint[j].actualTorque = 0;
    }
}

void gazebo::rok3_plugin::SetJointPIDgain()
{
    /*
     * Set each joint PID gain for joint control
     */
    joint[LHY].Kp = 2000;
    joint[LHR].Kp = 9000;
    joint[LHP].Kp = 2000;
    joint[LKN].Kp = 5000;
    joint[LAP].Kp = 3000;
    joint[LAR].Kp = 3000;

    joint[RHY].Kp = joint[LHY].Kp;
    joint[RHR].Kp = joint[LHR].Kp;
    joint[RHP].Kp = joint[LHP].Kp;
    joint[RKN].Kp = joint[LKN].Kp;
    joint[RAP].Kp = joint[LAP].Kp;
    joint[RAR].Kp = joint[LAR].Kp;

    joint[WST].Kp = 2.;

    joint[LHY].Kd = 2.;
    joint[LHR].Kd = 2.;
    joint[LHP].Kd = 2.;
    joint[LKN].Kd = 4.;
    joint[LAP].Kd = 2.;
    joint[LAR].Kd = 2.;

    joint[RHY].Kd = joint[LHY].Kd;
    joint[RHR].Kd = joint[LHR].Kd;
    joint[RHP].Kd = joint[LHP].Kd;
    joint[RKN].Kd = joint[LKN].Kd;
    joint[RAP].Kd = joint[LAP].Kd;
    joint[RAR].Kd = joint[LAR].Kd;

    joint[WST].Kd = 2.;
}

