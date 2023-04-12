/* 
 * Author: Giorgio Medico
 *
 * Created on April 9, 2023
 * 
 * UR Forward Kinematics and Inverse Kinematics
 * 
 * Based on the paper: A General Analytical Algorithm for Collaborative Robot with 6 Degrees of Freedom
 * 
 * Authors: S. Chen, M. Luo, O. Abdelaziz and G. Jiang
 *  
 */

#ifndef IK_ROBOTICS_H
#define IK_ROBOTICS_H

#include <ros/ros.h>
#include <stdio.h>  
#include <vector>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <math.h> 
#include <cmath>
#include <Eigen/Dense>
#include "ur_kinematics/UrInverseKinematics.h"
#include "ur_kinematics/UrForwardKinematics.h"


using V6f = Eigen::Matrix<float,6,1>;
using V4f = Eigen::Matrix<float,4,1>;
using M86f = Eigen::Matrix<float,8,6>;
using M4f = Eigen::Matrix4f;


namespace inverse_kinem{

    class FIKServer{

        public:

            FIKServer(int rate); // Class Constructor
            virtual ~FIKServer(); // Class Destructor
            void run();

        private:

            //Variables
            int rate;
            int num_sols;
            float d1, d4, d5,d6;
            float a2, a3;
            float alph1, alph4, alph5;
           
            M4f T_0_6 = M4f::Identity();
            V6f q;
            M86f ik_solutions = M86f::Zero();

            ros::NodeHandle n;
            ros::ServiceServer fk_service ;
            ros::ServiceServer ik_service ;
            std::string ur_type;

            ur_kinematics::UrInverseKinematics::Response res_new;
            


            //Methods
            void initialiseRos();
            void coefficients();

            bool fk_cb(ur_kinematics::UrForwardKinematics::Request &req, ur_kinematics::UrForwardKinematics::Response &res);
            bool ik_cb(ur_kinematics::UrInverseKinematics::Request &req, ur_kinematics::UrInverseKinematics::Response &res);

            bool HTrans();
            M4f AH(float alpha, float a, float d, float theta);
            bool invKine(M4f desired_pos);

            bool checkq6(ur_kinematics::UrInverseKinematics::Response &res);

    };
}


#endif //IK_ROBOTICS_H
