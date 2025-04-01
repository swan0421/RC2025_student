# [2025-1] Robot Control 실습


#### Gazebo Simulation for RoK-3.
본 패키지는 휴머노이드 로봇 RoK-3의 교육용 Gazebo Simulation 입니다.
이 문서는 패키지의 설명 문서이며, 구성은 다음과 같습니다.

* What to do before simulation

* Simulation Manual
  1. [Download](https://github.com/swan0421/RC2025_student) and Setting RC2025_student
  2. Libraries used in rok3_study_pkgs Package
  3. How to run rok3_study_pkgs package, **Please read section 2 before proceeding.**
----

## What to do before simulation 
1. [RoS-Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu) install, link : http://wiki.ros.org/noetic/Installation/Ubuntu
3. [Visual Studio Code](https://code.visualstudio.com/) install, link : https://code.visualstudio.com/

---
> **catkin workspace setting**
##### **1. Setup catkin workspace**
###### *`(catkin workspace가 설정되어있다면 Simulation Manual로 이동)`*
* terminal:
> ```js
> mkdir -p ~/catkin_ws/src
> cd ~/catkin_ws/src
> catkin_init_workspace
> cd ~/catkin_ws
> catkin_make

##### **2. .bashrc 파일에 내용 추가하기**

* terminal:
> ```js
> gedit ~/.bashrc
 내용 추가하기
> ```js
> # SET ROS Neotic
> source /opt/ros/noetic/setup.bash
> source ~/catkin_ws/devel/setup.bash
> 
> # SET ROS NETWORK
> export ROS_MASTER_URI=http://localhost:11311
> export ROS_HOSTNAME=localhost
> 
> # SET ROS NETWORK (다른 컴퓨터의 Master에 접속할 때)
> #export ROS_MASTER_URI=http://192.168.1.100:11311
> #export ROS_HOSTNAME=192.168.1.100
> 
> # SET ROS ALIAS
> alias cw='cd ~/catkin_ws'
> alias cs='cd ~/catkin_ws/src'
> alias cm='cd ~/catkin_ws && catkin_make'
* terminal:
> ```js
> source ~/.bashrc
----
## Simulation Manual 
### 1.[Download](https://github.com/swan0421/RC2025_student) and Setting RC2025_student
1. [RC2025_student Repository](https://github.com/swan0421/RC2025_student)에 접속, link : https://github.com/swan0421/RC2025_student

2. 복제된 Repository에 접속 후에, `Code ▼`라는 초록색 버튼이 있는데 클릭하여 URL 주소 (https:/~)을 복사합니다.

3.  주소를 복사하였다면 `Home/catkin_ws/src/` 위치에서 터미널 창을 열어 다음 명령어를 입력합니다.
* terminal:
  
	> ```js
	> git clone https://github.com/swan0421/RC2025_student.git rok3_study_pkgs
	
	
4. `rok3_model` 폴더를 `Home/.gazebo/models/` 폴더로 가져와서 시뮬레이션을 위한 파일 셋팅을 마무리합니다.  
***(`.gazebo` 폴더가 보이지 않으면,  Home 폴더에서, `Ctrl+H` 를 눌러서 폴더 숨김 해제를 할 것)***  
***(Gazebo를 실행한 적이 없는 경우, 숨김해제를 하여도 폴더가 보이지 않을 수 있음. Terminal 에서 `gazebo`를 입력하여 한번 실행해준 후 다시 확인할 것)***
         
5. 패키지를 컴파일하기 전에, section 2를 진행하시기 바랍니다.
----


### 2.Libraries used in rok3_study_pkgs Package

| Library | Description |
| ------ | ----------- |
| [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)   | Eigen is a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.|
| [RBDL](https://rbdl.github.io/) | RBDL library contains highly efficient code for both forward and inverse dynamics for kinematic chains and branched models. |

**Recommended to install RBDL**

RBDL의 설치를 권장합니다. 

**RBDL Install**

1. `Home/catkin_ws/src/rok3_study_pkgs/src`  위치에서 터미널을 실행합니다.
* terminal :
  
	> ```js
	> git clone --recursive https://github.com/rbdl/rbdl
	> cd rbdl
	> mkdir build
	> cd build
	> cmake -D RBDL_BUILD_ADDON_URDFREADER=ON CMAKE_BUILD_TYPE=Release ..
	> sudo make
	> sudo make install

2. 그리고 다시 패키지를 컴파일하기 위해 새로운 터미널 창을 열어 `cd ~/catkin_ws && catkin_make`을 입력하여 컴파일을 진행합니다.
----

### 3.How to run rok3_study_pkgs package
#### **!! 시뮬레이션 실행 전에 확인 해야하거나 셋팅 !!**

* #### Setting for Fixed / Floating Dynamics

`Home/.gazebo/models/rok3_model`폴더에 있는 `model.sdf`를 엽니다. 그리고 Fixed / Floating Dynamics을 위해 `<fixed to world>`의 joint를 다음과 같이 셋팅 합니다.

**Setting Fixed Dynamics in `model.sdf`**
``` js
<?xml version="1.0"?>
<sdf version='1.6'>
  <model name='rok3_model'>
  <joint name="fixed to world" type="fixed">
      <parent>world</parent>
      <child>base_link</child>
    </joint>
.
.
.
  </model>
</sdf>
```


**Setting Floating Dynamics in `model.sdf`**
``` js
<?xml version="1.0"?>
<sdf version='1.6'>
  <model name='rok3_model'>
  <!-- <joint name="fixed to world" type="fixed">
      <parent>world</parent>
      <child>base_link</child>
    </joint> -->
.
.
.
  </model>
</sdf>
```

다음으로, `catkin_ws/src/rok3_study_pkgs/worlds`폴더에 있는 `rok3.world`를 엽니다. 그리고 Fixed / Floating Dynamics을 위해 모델의 `<pose frame>`을 다음과 같이 셋팅 합니다.

**Setting Fixed Dynamics in `rok3.world`**
``` js
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="rok3">
.
.
.
    <include>
      <uri>model://rok3_model</uri>
      <pose frame=''>0 0 1.947 0 0 0</pose>
      <plugin name="rok3_plugin" filename="librok3_study_pkgs.so"/> 
    </include>
  </world>
</sdf>
```

**Setting Floating Dynamics in `rok3.world`**
``` js
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="rok3">
.
.
.
    <include>
      <uri>model://rok3_model</uri>
      <pose frame=''>0 0 0.947 0 0 0</pose>
      <plugin name="rok3_plugin" filename="librok3_study_pkgs.so"/> 
    </include>
  </world>
</sdf>
```


* #### Check `model.urdf` file path for using RBDL in `rok3_plugin.cc`
* `rok3_plugin.cc`는 Gazebo main code 이며, `/catkin_ws/src/rok3_study_pkgs/src`에 있습니다.
* **그리고, `rok3_plugin.cc`에서 사용자는 반드시 `Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)`함수에서, 아래 코드 예시와 같이 `Addons::URDFReadFromFile()` 함수 안에 적용되어 있는 `rok3_model.urdf`의 경로를 확인해주시고, 다르다면 바로잡아주시기 바랍니다.**

* **`rok3_model.urdf`는 `/home/.gazebo/models/rok3_model/urdf` 폴더에 있으며, 파일 속성 확인을 통해 정확한 경로 확인하시기 바랍니다.**  
`rok3_model.sdf` 파일 오른쪽 클릭 -> `Properties` -> `Location` 확인

**In `rok3_plugin.cc`**
``` js
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
    Addons::URDFReadFromFile("/home/user_name/.gazebo/models/rok3_model/urdf/rok3_model.urdf", rok3_model, true, true);
    //↑↑↑ Check File Path ↑↑↑
    nDoF = rok3_model->dof_count - 6; // Get degrees of freedom, except position and orientation of the robot
    joint = new ROBO_JOINT[nDoF]; // Generation joint variables struct

    //* initialize and setting for robot control in gazebo simulation
    SetJointPIDgain();


    //* setting for getting dt
    last_update_time = model->GetWorld()->SimTime();
    update_connection = event::Events::ConnectWorldUpdateBegin(boost::bind(&rok3_plugin::UpdateAlgorithm, this));

}
```

**모든 준비 과정이 끝나면, `catkin_make`을 입력하여 컴파일을 진행합니다.**
* terminal :
  
```
cd ~/catkin_ws && catkin_make
```

**최종적으로, 다음과 같은 명령어를 통해 시뮬레이션 실행**
* terminal :
  
```
roslaunch rok3_study_pkgs rok3.launch
```

## 1. 실습 1 : 3-Link Planar Arm의 Forward Kinematics

* void Practice() 함수 만들기
```c
void Practice()
```
* Practice() 함수안에 matrix 생성 및 터미널창에 인쇄하기
* cout 사용
~~~c
std::cout << "C_IE = " << C_IE << std::endl;
~~~
* Load 함수에 Practice() 함수 사용
* 3-link planar arm를 위한 Homogeneous Transformation Matrix 만들기
* 모든 링크의 길이는 1m로 가정
~~~c
MatrixXd getTransformI0()
MatrixXd jointToTransform01(VectorXd q)
MatrixXd jointToTransform12(VectorXd q)
MatrixXd jointToTransform23(VectorXd q)
MatrixXd getTransform3E()
~~~
* Forward Kinematics 만들기
~~~c
VectorXd jointToPosition(VectorXd q)
MatrixXd jointToRotMat(VectorXd q)
VectorXd rotToEuler(MatrixXd rotMat)	// EulerZYX
~~~

## 2. 실습 2 : RoK-3의 Forward Kinematics  

<img width="700" src="./RoK-3_img/RoK-3 Frame.jpg" alt="rok-3 frame">  
    
* Homogeneous Transformation Matrix 만들기
~~~c
MatrixXd getTransformI0()
MatrixXd jointToTransform01(VectorXd q)
MatrixXd jointToTransform12(VectorXd q)
MatrixXd jointToTransform23(VectorXd q)
MatrixXd jointToTransform34(VectorXd q)
MatrixXd jointToTransform45(VectorXd q)
MatrixXd jointToTransform56(VectorXd q)
MatrixXd getTransform6E()
~~~
* Forward Kinematics 만들기
~~~c
VectorXd jointToPosition(VectorXd q)
MatrixXd jointToRotMat(VectorXd q)
VectorXd rotToEuler(MatrixXd rotMat)
~~~
* q=[10;20;30;40;50;60] 일때, End-effector의 position과 Rotation Matrix 구하기
* 이때, Euler Angle 구하기

### 제출자료
1. source 코드
2. 출력된 결과물 capture 파일


## 3. 실습 3 : RoK-3의 Geometric Jacobian

* jointToPosJac 함수 만들기
~~~c
MatrixXd jointToPosJac(VectorXd q)
{
    // Input: vector of generalized coordinates (joint angles)
    // Output: J_P, Jacobian of the end-effector translation which maps joint velocities to end-effector linear velocities in I frame.
    MatrixXd J_P = MatrixXd::Zero(3,6);
    MatrixXd T_I0(4,4), T_01(4,4), T_12(4,4), T_23(4,4), T_34(4,4), T_45(4,4), T_56(4,4), T_6E(4,4);
    MatrixXd T_I1(4,4), T_I2(4,4), T_I3(4,4), T_I4(4,4), T_I5(4,4), T_I6(4,4);
    MatrixXd R_I1(3,3), R_I2(3,3), R_I3(3,3), R_I4(3,3), R_I5(3,3), R_I6(3,3);
    Vector3d r_I_I1(3), r_I_I2(3), r_I_I3(3), r_I_I4(3), r_I_I5(3), r_I_I6(3);
    Vector3d n_1(3), n_2(3), n_3(3), n_4(3), n_5(3), n_6(3);
    Vector3d n_I_1(3),n_I_2(3),n_I_3(3),n_I_4(3),n_I_5(3),n_I_6(3);
    Vector3d r_I_IE(3);


    //* Compute the relative homogeneous transformation matrices.
    T_I0 = 
    T_01 = 
    T_12 = 
    T_23 = 
    T_34 =
    T_45 = 
    T_56 = 
    T_6E = 

    //* Compute the homogeneous transformation matrices from frame k to the inertial frame I.
    T_I1 = 
    T_I2 = 
    T_I3 = 
    T_I4 = 
    T_I5 =
    T_I6 = 

    //* Extract the rotation matrices from each homogeneous transformation matrix. Use sub-matrix of EIGEN. https://eigen.tuxfamily.org/dox/group__QuickRefPage.html
    R_I1 = T_I1.block(0,0,3,3);
    R_I2 = 
    R_I3 = 
    R_I4 = 
    R_I5 = 
    R_I6 = 

    //* Extract the position vectors from each homogeneous transformation matrix. Use sub-matrix of EIGEN.
    r_I_I1 = 
    r_I_I2 = 
    r_I_I3 = 
    r_I_I4 = 
    r_I_I5 = 
    r_I_I6 = 

    //* Define the unit vectors around which each link rotate in the precedent coordinate frame.
    n_1 << 0,0,1;
    n_2 << 
    n_3 << 
    n_4 << 
    n_5 << 
    n_6 << 

    //* Compute the unit vectors for the inertial frame I.
    n_I_1 = R_I1*n_1;
    n_I_2 = 
    n_I_3 = 
    n_I_4 = 
    n_I_5 = 
    n_I_6 = 

    //* Compute the end-effector position vector.
    r_I_IE = 


    //* Compute the translational Jacobian. Use cross of EIGEN.
    J_P.col(0) << n_I_1.cross(r_I_IE-r_I_I1);
    J_P.col(1) << 
    J_P.col(2) << 
    J_P.col(3) << 
    J_P.col(4) << 
    J_P.col(5) << 

    //std::cout << "Test, JP:" << std::endl << J_P << std::endl;

    return J_P;
}
~~~
* jointToRotJac 함수 만들기
~~~c
MatrixXd jointToRotJac(VectorXd q)
{
   // Input: vector of generalized coordinates (joint angles)
    // Output: J_R, Jacobian of the end-effector orientation which maps joint velocities to end-effector angular velocities in I frame.
    MatrixXd J_R(3,6);
    MatrixXd T_I0(4,4), T_01(4,4), T_12(4,4), T_23(4,4), T_34(4,4), T_45(4,4), T_56(4,4), T_6E(4,4);
    MatrixXd T_I1(4,4), T_I2(4,4), T_I3(4,4), T_I4(4,4), T_I5(4,4), T_I6(4,4);
    MatrixXd R_I1(3,3), R_I2(3,3), R_I3(3,3), R_I4(3,3), R_I5(3,3), R_I6(3,3);
    Vector3d n_1(3), n_2(3), n_3(3), n_4(3), n_5(3), n_6(3);

    //* Compute the relative homogeneous transformation matrices.


    //* Compute the homogeneous transformation matrices from frame k to the inertial frame I.


    //* Extract the rotation matrices from each homogeneous transformation matrix.


    //* Define the unit vectors around which each link rotate in the precedent coordinate frame.


    //* Compute the translational Jacobian.


    //std::cout << "Test, J_R:" << std::endl << J_R << std::endl;

    return J_R;
}
~~~
* q=[10;20;30;40;50;60] 일때, Geometric Jacobian 구하기

### 제출자료
1. source 코드
2. 출력된 결과물 capture 파일


## 실습 4 : RoK-3의 Pseudo-Inverse 함수와 rotTatToRotVec 함수 만들리

* pseudoInverseMat 함수 만들기
~~~c
MatrixXd pseudoInverseMat(MatrixXd A, double lambda)
{
    // Input: Any m-by-n matrix
    // Output: An n-by-m pseudo-inverse of the input according to the Moore-Penrose formula
    MatrixXd pinvA;





    return pinvA;
}
~~~    

* rotMatToRotVec 함수 만들기 : rotation matrix를 입력으로 받아서 rotation vector를 내보내는 함수
~~~c
VectorXd rotMatToRotVec(MatrixXd C)
{
    // Input: a rotation matrix C
    // Output: the rotational vector which describes the rotation C
    Vector3d phi,n;
    double th;
    
    
    if(fabs(th)<0.001){
         n << 0,0,0;
    }
    else{

    }
        
    phi = th*n;
    
   
    return phi;
}
~~~    

### 과제
* q=[10;20;30;40;50;60] 일때, Jacobian의 pseudoInverse 구하기
~~~c
pinvJ = pseudoInverseMat(J);
~~~    

* q_des=[10;20;30;40;50;60], q_init=0.5q_des 일때, C_err(dph)=C_des*C_init.transpose() 구하기
* rotMatToRotVec 함수로 dph구하기
~~~c
dph = rotMatToRotVec(C_err);
~~~    

### 제출자료
1. source 코드
2. 출력된 결과물 capture 파일 => dph = [0.00;1.14;-0.19]



## 5. 실습 5 : RoK-3의 Numerical Inverse Kinematics

* inverseKinematics 함수 만들기
~~~c
VectorXd inverseKinematics(Vector3d r_des, MatrixXd C_des, VectorXd q0, double tol)
{
    // Input: desired end-effector position, desired end-effector orientation, initial guess for joint angles, threshold for the stopping-criterion
    // Output: joint angles which match desired end-effector position and orientation
    double num_it;
    MatrixXd J_P(6,6), J_R(6,6), J(6,6), pinvJ(6,6), C_err(3,3), C_IE(3,3);
    VectorXd q(6),dq(6),dXe(6);
    Vector3d dr(3), dph(3);
    double lambda;
    
    //* Set maximum number of iterations
    double max_it = 200;
    
    //* Initialize the solution with the initial guess
    q=q0;
    C_IE = ...;
    C_err = ...;
    
    //* Damping factor
    lambda = 0.001;
    
    //* Initialize error
    dr = ... ;
    dph = ... ;
    dXe << dr(0), dr(1), dr(2), dph(0), dph(1), dph(2);
    
    ////////////////////////////////////////////////
    //** Iterative inverse kinematics
    ////////////////////////////////////////////////
    
    //* Iterate until terminating condition
    while (num_it<max_it && dXe.norm()>tol)
    {
        
        //Compute Inverse Jacobian
        J_P = ...;
        J_R = ...;

        J.block(0,0,3,6) = J_P;
        J.block(3,0,3,6) = J_R; // Geometric Jacobian
        
        // Convert to Geometric Jacobian to Analytic Jacobian
        dq = pseudoInverseMat(J,lambda)*dXe;
        
        // Update law
        q += 0.5*dq;
        
        // Update error
        C_IE = ...;
        C_err = ...;
        
        dr = ...;
        dph = ...;
        dXe << dr(0), dr(1), dr(2), dph(0), dph(1), dph(2);
                   
        num_it++;
    }
    std::cout << "iteration: " << num_it << ", value: " << q << std::endl;
    
    return q;
}
~~~

### 과제 1
* q=[10;20;30;40;50;60] 일때, 이 관절각도에 해당하는 end-effoctor의 값을 r_des와 C_des로 설정하고,
* r_des와 C_des에 대한 joint angle 구하기

~~~c
void Practice()
{
        ...
        // q = [10;20;30;40;50;60]*pi/180;
        r_des = jointToPostion(q);
        C_des = jointToRotMat(q);
        
        q_cal = inverseKinematics(r_des, C_des, q*0.5, 0.001);
}
~~~  

### 제출자료
1. source 코드
2. 출력된 결과물 capture 파일

### 과제 2
* Desired Pos = [0;0.105;-0.55] & Desired Orientation = Base
* Result = [0;0;-63.756;127.512;-63.756]


## 6. 실습 6 : RoK-3의 Motion Control

* 1-cos 함수로 trajectory 생성하기

~~~c
double func_1_cos(double t, double, init, double final, double T)
{
        double des;
        
        ...
        
        return des;
}
~~~  


### 과제
0. 5초동안, 초기자세에서 실습5-2의 자세로 움직이기 in Joint Coordinates
1. 5초동안, z방향으로 0.2m 이동하기(다리들기) in Cartesian Coordinates
2. 5초동안 0.2m 다리들기, 5초동안 0.2m 다리내리기 in Cartesian Coordinates
3. 5초동안 0.2m 다리들기, 5초동안, z축으로 90도 회전하기 in Cartesian Coordinates


## 7. 실습 7 : Static walking in the air (2-step walking)

1. 5초동안, Walk ready 자세 만들기 
	* Right foot : Desired Pos = [0;0.105;-0.55] & Desired Orientation = Base) (Joint Coordinates)
	* Left foot : Desried Pos = [0;-0.105;-0.55] & Desired Orientation = Base) (Joint Coordinates)
3. 오른발 지지하며, 왼발 들기 (Cartesian Coordinates)
4. 두발 지지 (Cartesian Coordinates)
5. 왼발 지지하며, 오른발 들기 (Cartesian Coordinates)
6. 두발 지지 (Cartesian Coordinates)

### 과제
* 땅에서 실습 7 수행하기
