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

#ifndef CRAZYFLIE_2_POSITION_CONTROLLER_H
#define CRAZYFLIE_2_POSITION_CONTROLLER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include "common.h"
#include "parameters.h"
#include "stabilizer_types.h"
#include "crazyflie_complementary_filter.h"
#include "crazyflie_onboard_controller.h"
#include "sensfusion6.h"
#include "aggressive_parameters.h"

#include <time.h>

namespace rotors_control {

    struct control_acc{
        double x;
        double y;
        double z;
    };

    class AggressiveController{
        public:
            AggressiveController();
            ~AggressiveController();
            void CalculateRotorVelocities(Eigen::Vector4d* rotor_velocities,Eigen::Vector4d* forces);

            void SetOdometryWithStateEstimator(const EigenOdometry& odometry);
	          void SetOdometryWithoutStateEstimator(const EigenOdometry& odometry);
            void SetSensorData(const sensorData_t& sensors);
            void SetTrajectoryPoint(const mav_msgs::EigenTrajectoryPoint& command_trajectory);
	          void SetControllerGains();
            void CallbackAttitudeEstimation();
            void CallbackHightLevelControl();
            void resetPathFollow();
            void setPathFollow();
            void resetHover();
            void setHover();


            void SetGainP(const double *GainP);
            void SetGainD(const double *GainD);
            void SetGainPT(const double *GainPT);
            void SetGainDT(const double *GainDT);
            void SetGainPeta(const double *GainPeta);
            void SetGainDom(const double *GainDom);
            void SetGainPP(const double *GainPP);
            void SetGainDD(const double *GainDD);
            AggressiveControllerParameters controller_parameters_;
            ComplementaryFilterCrazyflie2 complementary_filter_crazyflie_;
            CrazyflieOnboardController crazyflie_onboard_controller_;

            bool hover_is_active;
            bool path_is_active;
            bool active;

            Eigen::Quaterniond orientation_;
            Eigen::Vector3d position_;
            Eigen::Vector3d ang_vel_;
            Eigen::Vector3d velocity_;


      private:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            bool controller_active_;
            bool state_estimator_active_;

            // control acceleration for RPThrustControl
            struct control_acc c_a;

            control_s control_t_;
            attitude_s attitude_t_;

            // Actual attitude gains
            Eigen::Vector3f attitude_kp;
            Eigen::Vector3f attitude_kd;



            //Integrator state
           double error_x;
           double error_y;
           double error_z;

           //motor parameter
           double bf;
           double bm;
           double l;

           Eigen::Vector3d attitude_error_;
           Eigen::Matrix3d Inertia;
           Eigen::Vector3d previous_error_w;
           Eigen::Matrix3d GainP_;   // Torques gains of angular displacement
           Eigen::Matrix3d GainD_;   // Torques gains of angular velocity
           Eigen::Matrix3d GainPT_;  // Torques gains of position
           Eigen::Matrix3d GainDT_;  // Torques gains of velocity
           Eigen::Vector3d GainPP_;  // Thrust gains of positions
           Eigen::Vector3d GainDD_;  // Thrust gains of velocities
           Eigen::Vector3d GainDom_; // Thrust gains of angular deisplacement
           Eigen::Vector3d GainPeta_;  // Thrust gains of angular velocity

           Eigen::Matrix3d Rotation_des;
           // matrix conversion
           Eigen::Matrix4d Conversion;
           Eigen::Matrix3d Rotation_wb;

            //Controller gains
            Eigen::Vector3f hover_xyz_stiff_kp_;
            Eigen::Vector3f hover_xyz_soft_kp_;

            Eigen::Vector3f hover_xyz_stiff_ki_;
            Eigen::Vector3f hover_xyz_soft_ki_;

            Eigen::Vector3f hover_xyz_stiff_kd_;
            Eigen::Vector3f hover_xyz_soft_kd_;

            Eigen::Vector3f hover_xyz_stiff_angle_kp_;
            Eigen::Vector3f hover_xyz_soft_angle_kp_;

            Eigen::Vector3f hover_xyz_stiff_angle_kd_;
            Eigen::Vector3f hover_xyz_soft_angle_kd_;

            Eigen::Vector3f attitude_kp_;
            Eigen::Vector3f attitude_kd_;

            Eigen::Vector3f path_angle_kp_;
            Eigen::Vector3f path_angle_kd_;

            Eigen::Vector3f path_kp_;
            Eigen::Vector3f path_kd_;

            void SetSensorData();

            mav_msgs::EigenTrajectoryPoint command_trajectory_;
            EigenOdometry odometry_;
            sensorData_t sensors_;
            state_t state_;


            Eigen::Vector3d veeOp(Eigen::Matrix3d );
            // compute the thrust when hover and path control are not active
            void AggressiveControl(double &thrust,double* delta_roll, double* delta_pitch, double* delta_yaw);

            // compute the thrust and the desired angles according to the mellinger control for aggressive maneuvers
            void RPThrustControl(double &phi_des, double &theta_des,double &delta_F);

            void Clamp(double* value, double min, double max);

            void HoverControl( double* acc_x, double* acc_y, double* acc_z);

            void AttitudeError(Eigen::Vector3d &errorAngle,Eigen::Vector3d &additionError,Eigen::Vector3d &errorAngularVelocity);

            void AttitudeRecoverController(double* delta_roll, double* delta_pitch, double* delta_yaw);
            void AttitudeController(double* delta_roll, double* delta_pitch, double* delta_yaw);

            void ErrorBodyFrame(double* x_error_, double* y_error_,double* z_error_) const;
            void ErrorBodyFrame(double* x_error_, double* y_error_,double* z_error_, Eigen::Vector3d &velocity_error) const;

            void PathFollowing3D(double &delta_F);

            // compute the forces/torques applying the attitude control
            void ControlMixer(double* PWM_1, double* PWM_2, double* PWM_3, double* PWM_4);

            void Quaternion2Euler(double* roll, double* pitch, double* yaw) const;
            void setRotation();

    };

}
#endif // CRAZYFLIE_2_POSITION_CONTROLLER_H
