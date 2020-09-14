/*
 * Copyright 2018 Giuseppe Silano, University of Sannio in Benevento, Italy
 * Copyright 2018 Emanuele Aucone, University of Sannio in Benevento, Italy
 * Copyright 2018 Benjamin Rodriguez, MIT, USA
 * Copyright 2018 Luigi Iannelli, University of Sannio in Benevento, Italy
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "rotors_control/aggressive_controller.h"
#include "rotors_control/transform_datatypes.h"
#include "rotors_control/Matrix3x3.h"
#include "rotors_control/Quaternion.h"
#include "rotors_control/stabilizer_types.h"
#include "rotors_control/sensfusion6.h"

#include <math.h> 
#include <ros/ros.h>
#include <time.h>
#include <chrono>
#include <cmath>

#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <tf_conversions/tf_eigen.h>

#define GRAVITY                                  9.81 /* g [m/s^2]*/
#define M_PI                                     3.14159265358979323846  /* pi [rad]*/
#define MAX_PROPELLERS_ANGULAR_VELOCITY          6104 /* MAX PROPELLERS ANGULAR VELOCITY [rad/s]*/
#define SAMPLING_TIME                            0.001 /* SAMPLING TIME [s] */

namespace rotors_control{

AggressiveController::AggressiveController()
    : hover_is_active(false),
      path_is_active(false),
    controller_active_(false),
    state_estimator_active_(false),
    active(true),
    error_x(0),
    error_y(0),
    error_z(0){

      // The control variables are initialized to zero
      control_t_.roll = 0;
      control_t_.pitch = 0;
      control_t_.yawRate = 0;
      control_t_.thrust = 0;

      state_.angularAcc.x = 0; // Angular Acceleration x
      state_.angularAcc.y = 0; // Angular Acceleration y
      state_.angularAcc.z = 0; // Angular Acceleration z

      state_.attitude.roll = 0; // Roll
      state_.attitude.pitch = 0; // Pitch
      state_.attitude.yaw = 0; // Yaw

      state_.position.x = 0; // Position.x
      state_.position.y = 0; // Position.y
      state_.position.z = 0; // Position.z

      state_.angularVelocity.x = 0; // Angular velocity x
      state_.angularVelocity.y = 0; // Angular velocity y
      state_.angularVelocity.z = 0; // Angular velocity z

      state_.linearVelocity.x = 0; //Linear velocity x
      state_.linearVelocity.y = 0; //Linear velocity y
      state_.linearVelocity.z = 0; //Linear velocity z

      state_.attitudeQuaternion.x = 0; // Quaternion x
      state_.attitudeQuaternion.y = 0; // Quaternion y
      state_.attitudeQuaternion.z = 0; // Quaternion z
      state_.attitudeQuaternion.w = 0; // Quaternion w

      velocity_ = Eigen::Vector3d::Zero();
      position_ = Eigen::Vector3d::Zero();
      ang_vel_ = Eigen::Vector3d::Zero();
      previous_error_w = Eigen::Vector3d::Zero();
      orientation_ = Eigen::Quaterniond::Identity();
      attitude_error_ = Eigen::Vector3d::Zero();

}

AggressiveController::~AggressiveController() {}

// Controller gains are entered into local global variables
void AggressiveController::SetControllerGains(){

        hover_xyz_stiff_kp_ = Eigen::Vector3f(controller_parameters_.hover_xyz_stiff_kp_.x(),
                                            controller_parameters_.hover_xyz_stiff_kp_.y(),
                                            controller_parameters_.hover_xyz_stiff_kp_.z());

        hover_xyz_stiff_ki_ = Eigen::Vector3f(controller_parameters_.hover_xyz_stiff_ki_.x(),
                                            controller_parameters_.hover_xyz_stiff_ki_.y(),
                                            controller_parameters_.hover_xyz_stiff_ki_.z());


        hover_xyz_stiff_kd_ = Eigen::Vector3f(  controller_parameters_.hover_xyz_stiff_kd_.x(),
                                                controller_parameters_.hover_xyz_stiff_kd_.y(),
                                                controller_parameters_.hover_xyz_stiff_kd_.z());

        hover_xyz_stiff_angle_kp_ = Eigen::Vector3f(    controller_parameters_.hover_xyz_stiff_angle_kp_.x(),
                                                        controller_parameters_.hover_xyz_stiff_angle_kp_.y(),
                                                        controller_parameters_.hover_xyz_stiff_angle_kp_.z());

        hover_xyz_stiff_angle_kd_ = Eigen::Vector3f(    controller_parameters_.hover_xyz_stiff_angle_kd_.x(),
                                                        controller_parameters_.hover_xyz_stiff_angle_kd_.y(),
                                                        controller_parameters_.hover_xyz_stiff_angle_kd_.z());


        path_angle_kp_ = Eigen::Vector3f( controller_parameters_.path_angle_kp_.x(),
                controller_parameters_.path_angle_kp_.y(),
                controller_parameters_.path_angle_kp_.z());
        path_angle_kd_ = Eigen::Vector3f( controller_parameters_.path_angle_kd_.x(),
                controller_parameters_.path_angle_kd_.y(),
                controller_parameters_.path_angle_kd_.z());

        path_kp_ = Eigen::Vector3f(controller_parameters_.path_kp_.x(),
                controller_parameters_.path_kp_.y(),
                controller_parameters_.path_kp_.z());
        path_kd_ = Eigen::Vector3f(controller_parameters_.path_kd_.x(),
                controller_parameters_.path_kd_.y(),
                controller_parameters_.path_kd_.z());

        bm = controller_parameters_.bm;
        bf = controller_parameters_.bf;
        l = controller_parameters_.l;

        ROS_INFO("%f, %f", bf,bm);

        Eigen::Matrix4d Conversion_fw;
        Conversion_fw << bf, bf, bf, bf,
                         cos(M_PI/4)*-bf*l, cos(M_PI/4)*-bf*l, cos(M_PI/4)*bf*l, cos(M_PI/4)*bf*l,
                         cos(M_PI/4)*-bf*l, cos(M_PI/4)*bf*l, cos(M_PI/4)*bf*l, cos(M_PI/4)*-bf*l,
                         -bm, bm, -bm, bm;
        Conversion <<  1/(4*bf),        -1/(4*bf*l*cos(M_PI/4)),      -1/(4*bf*l*cos(M_PI/4)),      -1/(4*bm),
                       1/(4*bf),        -1/(4*bf*l*cos(M_PI/4)),       1/(4*bf*l*cos(M_PI/4)),       1/(4*bm),
                       1/(4*bf),         1/(4*bf*l*cos(M_PI/4)),       1/(4*bf*l*cos(M_PI/4)),      -1/(4*bm),
                       1/(4*bf),         1/(4*bf*l*cos(M_PI/4)),      -1/(4*bf*l*cos(M_PI/4)),       1/(4*bm);


        attitude_kp = hover_xyz_stiff_angle_kp_;
        attitude_kd = hover_xyz_stiff_angle_kd_;


        Inertia = controller_parameters_.Inertia;


}

void AggressiveController::SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory) {

    command_trajectory_= command_trajectory;
    
    if(active)
    controller_active_= true;
    else
    controller_active_ = false;
}

void AggressiveController::SetGainP(const double *GainP) {

     GainP_ << GainP[0], GainP[1], GainP[2],
               GainP[3], GainP[4], GainP[5],
               GainP[6], GainP[7], GainP[8];


}

void AggressiveController::SetGainD(const double *GainD) {

  GainD_ << GainD[0], GainD[1], GainD[2],
            GainD[3], GainD[4], GainD[5],
            GainD[6], GainD[7], GainD[8];

}

void AggressiveController::SetGainPT(const double *GainPT) {

     GainPT_ << GainPT[0], GainPT[1], GainPT[2],
               GainPT[3], GainPT[4], GainPT[5],
               GainPT[6], GainPT[7], GainPT[8];


}

void AggressiveController::SetGainDT(const double *GainDT) {

  GainDT_ << GainDT[0], GainDT[1], GainDT[2],
            GainDT[3], GainDT[4], GainDT[5],
            GainDT[6], GainDT[7], GainDT[8];

}

void AggressiveController::SetGainPeta(const double *GainPeta) {

     GainPeta_ << GainPeta[0], GainPeta[1], GainPeta[2];

}

void AggressiveController::SetGainDom(const double *GainDom) {

  GainDom_ << GainDom[0], GainDom[1], GainDom[2];

}

void AggressiveController::SetGainDD(const double *GainDD) {

     GainDD_ << GainDD[0], GainDD[1], GainDD[2];

}

void AggressiveController::SetGainPP(const double *GainPP) {

  GainPP_ << GainPP[0], GainPP[1], GainPP[2];

}


void AggressiveController::setHover()
{
  hover_is_active = true;
  attitude_error_ = Eigen::Vector3d::Zero();

  attitude_kp = hover_xyz_stiff_angle_kp_;
  attitude_kd = hover_xyz_stiff_angle_kd_;

}

void AggressiveController::resetHover()
{
  hover_is_active = false;
  attitude_error_ = Eigen::Vector3d::Zero();

  if(!path_is_active){
    attitude_kp = hover_xyz_stiff_angle_kp_;
    attitude_kd = hover_xyz_stiff_angle_kd_;
 }
}

void AggressiveController::setPathFollow()
{
  path_is_active = true;
  attitude_error_ = Eigen::Vector3d::Zero();

    attitude_kp = path_angle_kp_;
    attitude_kd = path_angle_kd_;

}

void AggressiveController::resetPathFollow()
{
  path_is_active = false;
  attitude_error_ = Eigen::Vector3d::Zero();

  if (!hover_is_active){
    attitude_kp = path_angle_kp_;
    attitude_kd = path_angle_kd_;
  }

}


void AggressiveController::Clamp(double* value, double min, double max){
    assert(value);

    if (*value < min) *value = min;
    if (*value > max) *value = max;
}

void AggressiveController::CalculateRotorVelocities(Eigen::Vector4d* rotor_velocities,Eigen::Vector4d* forces) {

    if(!controller_active_){
      *rotor_velocities = Eigen::Vector4d::Zero(rotor_velocities->rows());
       error_x = 0;
       error_y = 0;
       error_z = 0;
       attitude_error_ = Eigen::Vector3d::Zero();
        return;
    }


    ControlMixer(&(forces->x()), &(forces->y()), &(forces->z()), &(forces->w()));


    Eigen::Vector4d omega;
    omega = Conversion*(*forces);

    if(omega(0) < 0)
       omega(0) = 0;
    if(omega(1) < 0)
       omega(1) = 0;
    if(omega(2) < 0)
       omega(2) = 0;
    if(omega(3) < 0)
       omega(3) = 0;

    omega(0) = std::sqrt(omega(0));
    omega(1) = std::sqrt(omega(1));
    omega(2) = std::sqrt(omega(2));
    omega(3) = std::sqrt(omega(3));

    //The omega values are saturated considering physical constraints of the system
    if(!(omega(0) < MAX_PROPELLERS_ANGULAR_VELOCITY ))
        if(omega(0) > MAX_PROPELLERS_ANGULAR_VELOCITY)
           omega(0) = MAX_PROPELLERS_ANGULAR_VELOCITY;


    if(!(omega(1) < MAX_PROPELLERS_ANGULAR_VELOCITY ))
        if(omega(1) > MAX_PROPELLERS_ANGULAR_VELOCITY)
           omega(1) = MAX_PROPELLERS_ANGULAR_VELOCITY;


    if(!(omega(2) < MAX_PROPELLERS_ANGULAR_VELOCITY ))
        if(omega(2) > MAX_PROPELLERS_ANGULAR_VELOCITY)
           omega(2) = MAX_PROPELLERS_ANGULAR_VELOCITY;

    if(!(omega(3) < MAX_PROPELLERS_ANGULAR_VELOCITY ))
        if(omega(3) > MAX_PROPELLERS_ANGULAR_VELOCITY)
           omega(3) = MAX_PROPELLERS_ANGULAR_VELOCITY;

    ROS_DEBUG("Omega_1: %f Omega_2: %f Omega_3: %f Omega_4: %f", omega(0), omega(1), omega(2), omega(3));
    *rotor_velocities = omega;
}

void AggressiveController::Quaternion2Euler(double* roll, double* pitch, double* yaw) const {
    assert(roll);
    assert(pitch);
    assert(yaw);

    // The estimated quaternion values
    double x, y, z, w;
    x = state_.attitudeQuaternion.x;
    y = state_.attitudeQuaternion.y;
    z = state_.attitudeQuaternion.z;
    w = state_.attitudeQuaternion.w;
    
    tf::Quaternion q(x, y, z, w);
    tf::Matrix3x3 m(q);
    m.getRPY(*roll, *pitch, *yaw);

    ROS_DEBUG("Roll: %f, Pitch: %f, Yaw: %f", *roll, *pitch, *yaw);

}

void AggressiveController::ControlMixer(double* PWM_1, double* PWM_2, double* PWM_3, double* PWM_4) {
    assert(PWM_1);
    assert(PWM_2);
    assert(PWM_3);
    assert(PWM_4);

    // When the state estimator is disable, the delta_omega_ value is computed as soon as the new odometry message is available.
    //The timing is managed by the publication of the odometry topic
    if (!state_estimator_active_)
          CallbackHightLevelControl();

    // Control signals are sent to the on board control architecture if the state estimator is active
    double delta_phi, delta_theta, delta_psi;

    if (!hover_is_active && path_is_active)
        PathFollowing3D(control_t_.thrust);
    else if (hover_is_active && !path_is_active)
    {
      HoverControl( &c_a.x, &c_a.y, &c_a.z);
      RPThrustControl(attitude_t_.roll, attitude_t_.pitch, control_t_.thrust);
    }

    if (!path_is_active && !hover_is_active)
          AggressiveControl(control_t_.thrust,&delta_phi, &delta_theta, &delta_psi);
    else
          AttitudeController(&delta_phi, &delta_theta, &delta_psi);


    *PWM_1 = control_t_.thrust;
    *PWM_2 = delta_phi;
    *PWM_3 = delta_theta;
    *PWM_4 = delta_psi;


    ROS_DEBUG("Omega: %f, Delta_theta: %f, Delta_phi: %f, delta_psi: %f", control_t_.thrust, delta_theta, delta_phi, delta_psi);
    ROS_DEBUG("PWM1: %f, PWM2: %f, PWM3: %f, PWM4: %f", *PWM_1, *PWM_2, *PWM_3, *PWM_4);
}


    void AggressiveController::ErrorBodyFrame(double* x_error_, double* y_error_,double* z_error_) const {
        assert(x_error_);
        assert(y_error_);
        assert(z_error_);

        // X and Y reference coordinates
        double x_r = command_trajectory_.position_W[0];
        double y_r = command_trajectory_.position_W[1];
        double z_r = command_trajectory_.position_W[2];

        // Position error
        *x_error_ = x_r - state_.position.x;
        *y_error_ = y_r - state_.position.y;
        *z_error_ = z_r - state_.position.z;
    }


    void AggressiveController::ErrorBodyFrame(double* x_error_, double* y_error_,double* z_error_, Eigen::Vector3d &velocity_error) const {
        assert(x_error_);
        assert(y_error_);
        assert(z_error_);

        ErrorBodyFrame(x_error_,y_error_,z_error_);

        Eigen::Vector3d velocity = Eigen::Vector3d(state_.linearVelocity.x, state_.linearVelocity.y,state_.linearVelocity.z);

        velocity_error = command_trajectory_.velocity_W - velocity;

    }


void AggressiveController::PathFollowing3D(double &delta_F) {

    double mass = controller_parameters_.mass;

    double x_error_, y_error_, z_error_;
    Eigen::Vector3d ev;
    ErrorBodyFrame(&x_error_,&y_error_,&z_error_,ev);

    // Aggressive controll paper with snap trajectory pag.3 col. 1

    Eigen::Vector3d F;
    error_x += x_error_ * SAMPLING_TIME;
    error_y += y_error_ * SAMPLING_TIME;
    error_z += z_error_ * SAMPLING_TIME;

    Clamp(&error_x,-1,1);
    Clamp(&error_y,-1,1);
    Clamp(&error_z,-1,1);

    F(0) = path_kp_.x() * x_error_ + path_kd_.x() * ev(0) + mass * command_trajectory_.acceleration_W(0) + hover_xyz_soft_ki_.x()*error_x;
    F(1) = path_kp_.y() * y_error_ + path_kd_.y() * ev(1) + mass * command_trajectory_.acceleration_W(1) + hover_xyz_soft_ki_.y()*error_y;
    F(2) =  path_kp_.z() * z_error_ + path_kd_.z() * ev(2) + mass * command_trajectory_.acceleration_W(2) + mass*GRAVITY + hover_xyz_soft_ki_.z()*error_z;

    Eigen::Vector3d z_b = F/F.norm();

    Eigen::Vector3d x_c = Eigen::Vector3d(cos(attitude_t_.yaw), sin(attitude_t_.yaw),0 );

    Eigen::Vector3d y_b = z_b.cross(x_c);
    y_b = y_b/y_b.norm();

    Eigen::Vector3d x_b = y_b.cross(z_b);

    Rotation_des << x_b,y_b,z_b;

    Eigen::Matrix3d R_t = Rotation_wb;
    delta_F = F.dot(R_t.col(2));

}


/* FROM HERE THE FUNCTIONS EMPLOYED WHEN THE STATE ESTIMATOR IS UNABLE ARE REPORTED */

//Such function is invoked by the position controller node when the state estimator is not in the loop
void AggressiveController::SetOdometryWithoutStateEstimator(const EigenOdometry& odometry) {
    
    odometry_ = odometry; 

    // Such function is invoked when the ideal odometry sensor is employed
    SetSensorData();
}

// Odometry values are put in the state structure. The structure contains the aircraft state
void AggressiveController::SetSensorData() {
    
    // Only the position sensor is ideal, any virtual sensor or systems is available to get it
    state_.position.x = odometry_.position[0];
    state_.position.y = odometry_.position[1];
    state_.position.z = odometry_.position[2];

    state_.attitudeQuaternion.x = odometry_.orientation.x();
    state_.attitudeQuaternion.y = odometry_.orientation.y();
    state_.attitudeQuaternion.z = odometry_.orientation.z();
    state_.attitudeQuaternion.w = odometry_.orientation.w();

    setRotation();

    double theta, phi, psi;
    Quaternion2Euler(&phi, &theta, &psi);

    state_.linearVelocity.x = (cos(theta) * cos(psi) * odometry_.velocity[0]) +
            ( ( (sin(phi) * sin(theta) * cos(psi) ) - ( cos(phi) * sin(psi) ) ) * odometry_.velocity[1]) +
            ( ( (cos(phi) * sin(theta) * cos(psi) ) + ( sin(phi) * sin(psi) ) ) *  odometry_.velocity[2]);

    state_.linearVelocity.y = (cos(theta) * sin(psi) * odometry_.velocity[0]) +
            ( ( (sin(phi) * sin(theta) * sin(psi) ) + ( cos(phi) * cos(psi) ) ) * odometry_.velocity[1]) +
            ( ( (cos(phi) * sin(theta) * sin(psi) ) - ( sin(phi) * cos(psi) ) ) *  odometry_.velocity[2]);

    state_.linearVelocity.z = (-sin(theta) * odometry_.velocity[0]) + ( sin(phi) * cos(theta) * odometry_.velocity[1]) +
            (cos(phi) * cos(theta) * odometry_.velocity[2]);


    state_.angularVelocity.x = odometry_.angular_velocity[0];
    state_.angularVelocity.y = odometry_.angular_velocity[1];
    state_.angularVelocity.z = odometry_.angular_velocity[2];
}

void AggressiveController::setRotation()
{
  double x, y, z, w;
  x = state_.attitudeQuaternion.x;
  y = state_.attitudeQuaternion.y;
  z = state_.attitudeQuaternion.z;
  w = state_.attitudeQuaternion.w;

  Rotation_wb <<  1-2*pow(y,2) -2*pow(z,2), 2*x*y - 2*z*w ,2*x*z + 2*y*w,
                  2*x*y + 2*z*w, 1- 2*pow(x,2) - 2*pow(z,2), 2*y*z - 2*x*w,
                  2*x*z - 2*y*w, 2*y*z + 2*x*w, 1- 2*pow(x,2) -2*pow(y,2);
}

void AggressiveController::HoverControl( double* acc_x, double* acc_y, double* acc_z) {
    assert(acc_x);
    assert(acc_y);
    assert(acc_z);

    double error_px, error_py, error_pz;
    ErrorBodyFrame(&error_px,&error_py,&error_pz);
    double mass = controller_parameters_.mass;

    bool stiff = true;

    if (!path_is_active && !hover_is_active )
      stiff = false;

    if(stiff == true)
    {
        // Stiff hover
        error_x += error_px * SAMPLING_TIME;
        error_y += error_py * SAMPLING_TIME;
        error_z += error_pz * SAMPLING_TIME;

        Clamp(&error_x,-1,1);
        Clamp(&error_y,-1,1);
        Clamp(&error_z,-0.4,0.4);

        error_px = hover_xyz_stiff_kp_.x() * error_px;
        error_py = hover_xyz_stiff_kp_.y() * error_py;
        error_pz = hover_xyz_stiff_kp_.z() * error_pz;


        *acc_x = error_px + hover_xyz_stiff_ki_.x()*error_x - hover_xyz_stiff_kd_.x() * state_.linearVelocity.x;
        *acc_y = error_py + hover_xyz_stiff_ki_.y()*error_y - hover_xyz_stiff_kd_.y() * state_.linearVelocity.y;
        *acc_z = error_pz + hover_xyz_stiff_ki_.z()*error_z - hover_xyz_stiff_kd_.z() * state_.linearVelocity.z;
    }

}

void AggressiveController::AttitudeController(double* delta_roll, double* delta_pitch, double* delta_yaw) {
    assert(delta_roll);
    assert(delta_pitch);
    assert(delta_yaw);

    Eigen::Vector3d errorAngle;
    Eigen::Vector3d errorAngularVelocity;    
    Eigen::Vector3d addition_error;

    AttitudeError(errorAngle,addition_error,errorAngularVelocity);

    Clamp(&attitude_error_.x(),-3,3);
    Clamp(&attitude_error_.y(),-3,3);
    Clamp(&attitude_error_.z(),-1500,1500);

    *delta_roll = -attitude_kp.x() * errorAngle.x() - attitude_kd.x() * errorAngularVelocity.x();
    *delta_pitch = -attitude_kp.y() * errorAngle.y() - attitude_kd.y() * errorAngularVelocity.y();
    *delta_yaw = -attitude_kp.z() * errorAngle.z() - attitude_kd.z() * errorAngularVelocity.z();

    previous_error_w = Eigen::Vector3d(*delta_roll,*delta_pitch,*delta_yaw);

    ROS_DEBUG("roll_c: %f, roll_e: %f, ", *delta_roll, errorAngle(0));
    ROS_DEBUG("pitch_c: %f, pitch_e: %f", *delta_pitch, errorAngle(1));
    ROS_DEBUG("pitch_c: %f, pitch_e: %f", *delta_yaw, errorAngle(2));


}


Eigen::Vector3d lnQuat(quaternion_t q)
{
  Eigen::Vector3d error= Eigen::Vector3d(q.x,q.y,q.z);

  if(error.norm()!=0.0)
    error = error/error.norm()*std::acos(q.w);
  else
    error = Eigen::Vector3d::Zero();
  return error;

}


// CASTILO paper robust recover
void AggressiveController::AttitudeRecoverController(double* delta_roll, double* delta_pitch, double* delta_yaw) {
    assert(delta_roll);
    assert(delta_pitch);
    assert(delta_yaw);

    Eigen::Vector3d errorAngle;
    Eigen::Vector3d errorAngularVelocity;
    Eigen::Vector3d addition_error;

    Eigen::Vector3d torques;
    AttitudeError(errorAngle,addition_error,errorAngularVelocity);

    Eigen::Vector3d Temp;
    Temp.x() = state_.angularVelocity.x;
    Temp.y() = state_.angularVelocity.y;
    Temp.z() = state_.angularVelocity.z;
    Temp = Temp.cross(Inertia*Temp);

    torques.x() = -attitude_kp.x() * errorAngle.x() - attitude_kd.x() * errorAngularVelocity.x();
    torques.y() = -attitude_kp.y() * errorAngle.y() - attitude_kd.y() * errorAngularVelocity.y();
    torques.z() = -attitude_kp.z() * errorAngle.z() - attitude_kd.z() * errorAngularVelocity.z();

    *delta_roll = 6*(-attitude_kp.x() * errorAngle.x() - attitude_kd.x() * errorAngularVelocity.x())
                                      *2*std::atan(torques.norm())/M_PI/torques.norm()+ Temp(0) -attitude_kd_.x()*state_.angularVelocity.x;
    *delta_pitch =6*(-attitude_kp.y() * errorAngle.y() - attitude_kd.y()* errorAngularVelocity.y())
                                      *2*std::atan(torques.norm())/M_PI/torques.norm()+ Temp(1) -attitude_kd_.y()*state_.angularVelocity.y;
    *delta_yaw = 6*(-attitude_kp.z() * errorAngle.z() - attitude_kd.z() * errorAngularVelocity.z())
                                      *2*std::atan(torques.norm())/M_PI/torques.norm()+ Temp(2) -attitude_kd_.z()*state_.angularVelocity.z;
    ROS_DEBUG("roll_c: %f, roll_e: %f, ", *delta_roll, errorAngle(0));
    ROS_DEBUG("pitch_c: %f, pitch_e: %f", *delta_pitch, errorAngle(1));
    ROS_DEBUG("pitch_c: %f, pitch_e: %f", *delta_yaw, errorAngle(2));


}




void AggressiveController::AttitudeError(Eigen::Vector3d &errorAngle,
                                        Eigen::Vector3d &additionError,Eigen::Vector3d &errorAngularVelocity)
{

   Eigen::Vector3d test = command_trajectory_.angular_velocity_W ;

  if (!path_is_active && !hover_is_active)
      {

          tf::Quaternion q( command_trajectory_.orientation_W_B.x(),
                        command_trajectory_.orientation_W_B.y(),
                        command_trajectory_.orientation_W_B.z(),
                        command_trajectory_.orientation_W_B.w());
          tf::Matrix3x3 m(q);

          tf::matrixTFToEigen(m,Rotation_des);

   }
    if(hover_is_active)
    {
        attitude_t_.yaw = command_trajectory_.getYaw();
        tf::Matrix3x3 m;
        m.setRPY(attitude_t_.roll,attitude_t_.pitch,attitude_t_.yaw);
        tf::matrixTFToEigen(m,Rotation_des);

    }
        // Mellinger controll paper with snap trajectory pag.3 col. 1

     errorAngle = 0.5*veeOp(Rotation_des.transpose()*Rotation_wb
                           - Rotation_wb.transpose()*Rotation_des);

     Eigen::Vector3d temp;

     temp = command_trajectory_.angular_velocity_W;

    errorAngularVelocity.x() = state_.angularVelocity.x - temp(0);
    errorAngularVelocity.y() = state_.angularVelocity.y - temp(1);
    errorAngularVelocity.z() = state_.angularVelocity.z - temp(2);

    additionError = Inertia*command_trajectory_.angular_acceleration_W;

    if (!hover_is_active)
    {
      if( test.norm() < 18)
       {
          ROS_INFO("Path3D");
          path_is_active = true;
       }
      else
       {
          ROS_INFO("Aggressive");
          path_is_active = false;
       }
    }

}

Eigen::Vector3d AggressiveController::veeOp(Eigen::Matrix3d M){

  Eigen::Vector3d error;

  error(0) = M(2,1);
  error(1) = M(0,2);
  error(2) = M(1,0);

  return error;
}

void AggressiveController::RPThrustControl(double &phi_des, double &theta_des,double &delta_F)
{
    // Linearization of the system under the hypothesis to have small pitch and roll angles

    double mass = controller_parameters_.mass;

    phi_des = (1/(c_a.z + GRAVITY))*(c_a.x * sin(command_trajectory_.getYaw()) - c_a.y * cos(command_trajectory_.getYaw()));
    theta_des = (1/(c_a.z + GRAVITY))*(c_a.x * cos(command_trajectory_.getYaw()) + c_a.y * sin(command_trajectory_.getYaw()));
    delta_F = mass * (c_a.z + GRAVITY);

    if (phi_des < 0){
      if(phi_des < -M_PI/2+0.1 && phi_des > -2*M_PI)
        phi_des = -M_PI/2+0.1;
    }
    else{
      if(phi_des > M_PI/2-0.1 && phi_des < 2*M_PI)
        phi_des = +M_PI/2-0.1;
    }

    if (theta_des < 0){
      if(theta_des < -M_PI/2+0.1 && theta_des > -2*M_PI)
        theta_des = -M_PI/2+0.1;
    }
    else{
      if(theta_des > M_PI/2-0.1 && theta_des < 2*M_PI)
        theta_des = +M_PI/2-0.1;
    }

}

void  AggressiveController::AggressiveControl(double &thrust,double* delta_roll, double* delta_pitch, double* delta_yaw)
{


  // translation error

  Eigen::Vector3d ev;
  Eigen::Vector3d error_p;

  ErrorBodyFrame(&error_p(0),&error_p(1),&error_p(2),ev);

  error_x += error_p(0) * SAMPLING_TIME;
  error_y += error_p(1) * SAMPLING_TIME;
  error_z += error_p(2) * SAMPLING_TIME;

  Clamp(&error_x,-1,1);
  Clamp(&error_y,-1,1);
  Clamp(&error_z,-1,1);

  // Angular error

  Eigen::Vector3d errorAngle;
  Eigen::Vector3d errorAngularVelocity;
  Eigen::Vector3d addition_error;

  AttitudeError(errorAngle,addition_error,errorAngularVelocity);

  double mass = controller_parameters_.mass;

  // feedfarward control
  Eigen::Vector3d F;

  F(0) =  mass*command_trajectory_.acceleration_W(0);
  F(1) =  mass*command_trajectory_.acceleration_W(1);
  F(2) =  mass*command_trajectory_.acceleration_W(2) + mass*GRAVITY;


  Eigen::Matrix3d R_t = Rotation_des.transpose();
  F = R_t*F;

  // thrust LQR

  thrust = F(2) - ( -GainPP_.dot(error_p)-GainDD_.dot(ev) + GainPeta_.dot(errorAngle) + GainDom_.dot(errorAngularVelocity));

  // Torques LQR

  Eigen::Vector3d  torques;

  torques = -GainP_*errorAngle -GainD_*errorAngularVelocity+GainPT_*error_p+GainDT_*ev;

  *delta_roll = torques(0);
  *delta_pitch = torques(1);
  *delta_yaw = torques(2);

}

/* FROM HERE THE FUNCTIONS EMPLOYED WHEN THE STATE ESTIMATOR IS ABLED ARE REPORTED */

// Such function is invoked by the position controller node when the state estimator is considered in the loop
void AggressiveController::SetOdometryWithStateEstimator(const EigenOdometry& odometry) {
    
    odometry_ = odometry;    
}


// The aircraft attitude is computed by the complementary filter with a frequency rate of 250Hz
void AggressiveController::CallbackAttitudeEstimation() {

    // Angular velocities updating
    complementary_filter_crazyflie_.EstimateAttitude(&state_, &sensors_);

    ROS_DEBUG("Attitude Callback");

}

// The high level control runs with a frequency of 100Hz
void AggressiveController::CallbackHightLevelControl() {


    ROS_DEBUG("Position_x: %f, Position_y: %f, Position_z: %f", state_.position.x, state_.position.y, state_.position.z);

    ROS_DEBUG("Angular_velocity_x: %f, Angular_velocity_y: %f, Angular_velocity_z: %f", state_.angularVelocity.x, 
             state_.angularVelocity.y, state_.angularVelocity.z);

    ROS_DEBUG("Linear_velocity_x: %f, Linear_velocity_y: %f, Linear_velocity_z: %f", state_.linearVelocity.x, 
             state_.linearVelocity.y, state_.linearVelocity.z);

}

// The aircraft angular velocities are update with a frequency of 500Hz
void AggressiveController::SetSensorData(const sensorData_t& sensors) {
  
    // The functions runs at 500Hz, the same frequency with which the IMU topic publishes new values (with a frequency of 500Hz)
    sensors_ = sensors;
    complementary_filter_crazyflie_.EstimateRate(&state_, &sensors_);
    
    if(!state_estimator_active_)
        state_estimator_active_= true;

    // Only the position sensor is ideal, any virtual sensor or systems is available to get these data
    // Every 0.002 seconds the odometry message values are copied in the state_ structure, but they change only 0.01 seconds
    state_.position.x = odometry_.position[0];
    state_.position.y = odometry_.position[1];
    state_.position.z = odometry_.position[2];

    state_.linearVelocity.x = odometry_.velocity[0];
    state_.linearVelocity.y = odometry_.velocity[1];
    state_.linearVelocity.z = odometry_.velocity[2];

}


}
