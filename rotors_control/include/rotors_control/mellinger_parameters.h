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

#ifndef MELLINGER_PARAMETERS_H
#define MELLINGER_PARAMETERS_H

// Default values for the position controller of the Crazyflie2. XYController [x,y], AttitudeController [phi,theta] 
//RateController [p,q,r], YawController[yaw], HoveringController[z]
const Eigen::Vector3f kPDefaultXYZHoverStiff = Eigen::Vector3f(1.0, 1.0, 1.0);
const Eigen::Vector3f kIDefaultXYZHoverStiff = Eigen::Vector3f(1.0, 1.0, 1.0);
const Eigen::Vector3f kDDefaultXYZHoverStiff = Eigen::Vector3f(1.0, 1.0, 1.0);

const Eigen::Vector3f kPDefaultXYZHoverSoft = Eigen::Vector3f(1.0, 1.0, 1.0);
const Eigen::Vector3f kIDefaultXYZHoverSoft = Eigen::Vector3f(1.0, 1.0, 1.0);
const Eigen::Vector3f kDDefaultXYZHoverSoft = Eigen::Vector3f(1.0, 1.0, 1.0);

const Eigen::Vector3f kPDefaultXYZHoverStiffAngle = Eigen::Vector3f(1.0, 1.0, 1.0);
const Eigen::Vector3f kDDefaultXYZHoverStiffAngle = Eigen::Vector3f(1.0, 1.0, 1.0);

const Eigen::Vector3f kPDefaultXYZHoverSoftAngle = Eigen::Vector3f(1.0, 1.0, 1.0);
const Eigen::Vector3f kDDefaultXYZHoverSoftAngle = Eigen::Vector3f(1.0, 1.0, 1.0);

const Eigen::Vector3f kPDefaultAttitudeGains = Eigen::Vector3f(1.0, 1.0, 1.0);
const Eigen::Vector3f kDDefaultAttitudeGains = Eigen::Vector3f(1.0, 1.0 ,1.0);

const Eigen::Vector3f kPDefaultXYZPathAngle = Eigen::Vector3f(1.0, 1.0, 1.0);
const Eigen::Vector3f kDDefaultXYZPathAngle = Eigen::Vector3f(1.0, 1.0, 1.0);
const Eigen::Vector3f kPDefaultXYZPath = Eigen::Vector3f(1.0, 1.0, 1.0);
const Eigen::Vector3f kDDefaultXYZPath = Eigen::Vector3f(1.0, 1.0, 1.0);

namespace rotors_control {

	class MellingerControllerParameters {
	 public:
	  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        MellingerControllerParameters()
	      : hover_xyz_stiff_kp_(kPDefaultXYZHoverStiff),
            hover_xyz_soft_kp_(kPDefaultXYZHoverSoft),
            hover_xyz_stiff_ki_(kIDefaultXYZHoverStiff),
            hover_xyz_soft_ki_(kIDefaultXYZHoverSoft),
            hover_xyz_stiff_kd_(kDDefaultXYZHoverStiff),
            hover_xyz_soft_kd_(kDDefaultXYZHoverSoft),
            hover_xyz_stiff_angle_kp_(kPDefaultXYZHoverStiffAngle),
            hover_xyz_soft_angle_kp_(kPDefaultXYZHoverSoftAngle),
            hover_xyz_stiff_angle_kd_(kDDefaultXYZHoverStiffAngle),
            hover_xyz_soft_angle_kd_(kDDefaultXYZHoverSoftAngle),
            attitude_kp_(kPDefaultAttitudeGains),
            attitude_kd_(kDDefaultAttitudeGains),
            path_angle_kp_(kPDefaultXYZPathAngle),
            path_angle_kd_(kDDefaultXYZPathAngle),
            path_kp_(kPDefaultXYZPath),
            path_kd_(kDDefaultXYZPath){
	  }

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

	};

}

#endif // MELLINGER_PARAMETERS_H
