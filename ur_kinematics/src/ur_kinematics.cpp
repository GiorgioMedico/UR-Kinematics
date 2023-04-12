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

#include "ik_ur.h"



int main(int argc, char **argv)
{
    ros::init(argc, argv, "F_I_Kinematics_UR");
    inverse_kinem::FIKServer Kinematics(500);

    // boost::thread t1(boost::bind(&inverse_kinem::FIKServer::run, &Kinematics));
    // t1.join();

    return 0;
}
