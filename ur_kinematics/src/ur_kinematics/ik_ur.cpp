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
 * 
 * Configuration of the UR robot
 *
 * Conf 0 = shoulder right, elbow up, wrist up
 * Conf 1 = shoulder right, elbow down, wrist up
 * Conf 2 = shoulder right, elbow up, wrist down
 * Conf 3 = shoulder right, elbow down, wrist down
 * Conf 4 = shoulder left, elbow down, wrist down
 * Conf 5 = shoulder left, elbow up, wrist down
 * Conf 6 = shoulder left, elbow down, wrist up
 * Conf 7 = shoulder left, elbow up, wrist up
 * 
 * Singular Configuration: q5 = 0 or q5 = pi
 * 
 */

#include "ik_ur.h"


namespace inverse_kinem{

    FIKServer::FIKServer(int rate)
    {
        this->rate = rate;
        
        this->initialiseRos();

        std::cout<<"RATE Kine: "<<rate<<std::endl;

        this->run();
    }

    FIKServer::~FIKServer()
    {
    }

    void FIKServer::run()
    {
        ros::Rate rs(this->rate);
        while(ros::ok())
        {
            ros::spinOnce();
            rs.sleep();
        }
    }

    void FIKServer::coefficients()
    {   
        if(this->ur_type == "UR5e")
        {
            this->a2 = -0.4253630103489888;
            this->a3 = -0.3924349641369202;

            this->d1 =  0.162710833731481;
            this->d4 =  0.1337873018461449;
            this->d5 =  0.09960901169401305;
            this->d6 =  0.09949603061275286;

            this->alph1 = M_PI/2;
            this->alph4 = M_PI/2;
            this->alph5 = -M_PI/2;
        }
        else if(this->ur_type == "UR5")
        {
            this->a2 = -0.4255204973093404;
            this->a3 = -0.3920641883612647;

            this->d1 =  0.08944066670827282;
            this->d4 =  0.1109001909756448;
            this->d5 =  0.09482875198638333;
            this->d6 =  0.08256209008760497;

            this->alph1 = M_PI/2;
            this->alph4 = M_PI/2;
            this->alph5 = -M_PI/2;
        }
        else
        {   
            ROS_ERROR("Robot not supported yet");
        }

    }

    void FIKServer::initialiseRos()
    {
        this->fk_service = this->n.advertiseService("ur_forward_k", &FIKServer::fk_cb,this);
        this->ik_service = this->n.advertiseService("ur_inverse_k", &FIKServer::ik_cb,this);

        std::cout<<"Kinematics Service Started"<<std::endl;
    }

  
    //      FORWARD KINEMATICS

	M4f FIKServer::AH(float alpha, float a, float d, float theta)
	{
		M4f A;
		A <<    std::cos(theta),    -std::sin(theta)*std::cos(alpha),     std::sin(theta)*std::sin(alpha),      a*std::cos(theta),
                std::sin(theta),     std::cos(theta)*std::cos(alpha),     -std::cos(theta)*std::sin(alpha),     a*std::sin(theta),
                0,                   std::sin(alpha),                      std::cos(alpha),                     d,
                0,                   0,                                     0,                                  1;

		return A;
	}

    
	bool FIKServer::HTrans()
	{

        M4f T_0_1, T_1_2, T_2_3, T_3_4, T_4_5, T_5_6;
        
        T_0_1 = AH(this->alph1,    0,          this->d1,   this->q[0]);
        T_1_2 = AH(0,              this->a2,   0,          this->q[1]);
        T_2_3 = AH(0,              this->a3,   0,          this->q[2]);
        T_3_4 = AH(this->alph4,    0,          this->d4,   this->q[3]);
        T_4_5 = AH(this->alph5,    0,          this->d5,   this->q[4]);
        T_5_6 = AH(0,              0,          this->d6,   this->q[5]);

        this->T_0_6 = T_0_1*T_1_2*T_2_3*T_3_4*T_4_5*T_5_6;

        //correzione per il sistema di riferimento

        //matrice di rotazione di 90 gradi attorno all'asse x
        Eigen::AngleAxisf rot_x(M_PI/2, Eigen::Vector3f::UnitX());
        Eigen::Matrix3f Rx = rot_x.toRotationMatrix();

        //matrice di rotazione di 90 gradi attorno all'asse z
        Eigen::AngleAxisf rot_z(M_PI/2, Eigen::Vector3f::UnitZ());
        Eigen::Matrix3f Rz = rot_z.toRotationMatrix();

        this->T_0_6.block<3,3>(0,0) = this->T_0_6.block<3,3>(0,0)*Rx;
        this->T_0_6.block<3,3>(0,0) = this->T_0_6.block<3,3>(0,0)*Rz;

        return true;
	}


    bool FIKServer::fk_cb(ur_kinematics::UrForwardKinematics::Request &req, ur_kinematics::UrForwardKinematics::Response &res)
    {
        this->ur_type = req.ur_type;
        this->coefficients();

        res.reference_pose.resize(req.reference_joints.size());

        for (unsigned int i = 0; i < req.reference_joints.size(); i++)
        {
            //create input vector from request
            this->q << req.reference_joints[i].data[0], req.reference_joints[i].data[1], req.reference_joints[i].data[2], req.reference_joints[i].data[3], req.reference_joints[i].data[4], req.reference_joints[i].data[5];
            
            //compute solution
            this->HTrans();
            
            //generate response message

            res.reference_pose[i].position.x = this->T_0_6(0,3);
            res.reference_pose[i].position.y = this->T_0_6(1,3);
            res.reference_pose[i].position.z = this->T_0_6(2,3);
            
            Eigen::Matrix3f rot = this->T_0_6.block<3,3>(0,0);

            Eigen::Quaternionf q(rot);

            res.reference_pose[i].orientation.x = q.x();
            res.reference_pose[i].orientation.y = q.y();
            res.reference_pose[i].orientation.z = q.z();
            res.reference_pose[i].orientation.w = q.w();
        }

        return true;         
    }

    //      INVERSE KINEMATICS
    bool FIKServer::invKine(M4f desired_pos)
    {   
        //calcolo theta1

        for (int i = 0; i < 4; i++)
        {
            this->ik_solutions(i,0) = std::atan2(this->d4, std::sqrt(std::pow((this->d6*desired_pos(1,2) - desired_pos(1,3)),2) + std::pow((desired_pos(0,3)-this->d6*desired_pos(0,2)),2) - std::pow(this->d4,2))) - std::atan2(this->d6*desired_pos(1,2) - desired_pos(1,3), desired_pos(0,3)-this->d6*desired_pos(0,2));
            if(std::isnan(this->ik_solutions(i,0)))
            {
                this->ik_solutions(i,0) = std::atan2(this->d4, 0) - std::atan2(this->d6*desired_pos(1,2) - desired_pos(1,3), desired_pos(0,3)-this->d6*desired_pos(0,2));
            }
            this->ik_solutions(i+4,0) = std::atan2(this->d4, - std::sqrt(std::pow((this->d6*desired_pos(1,2) - desired_pos(1,3)),2) + std::pow((desired_pos(0,3)-this->d6*desired_pos(0,2)),2) - std::pow(this->d4,2))) - std::atan2(this->d6*desired_pos(1,2) - desired_pos(1,3), desired_pos(0,3)-this->d6*desired_pos(0,2));
            if(std::isnan(this->ik_solutions(i+4,0)))
            {
                this->ik_solutions(i+4,0) = std::atan2(this->d4, 0) - std::atan2(this->d6*desired_pos(1,2) - desired_pos(1,3), desired_pos(0,3)-this->d6*desired_pos(0,2));
            }
        }

        //calcolo theta5

        for (int i = 0; i < 2; i++)
        {
            this->ik_solutions(i,4) = std::atan2(std::sqrt(std::pow((desired_pos(0,0)*std::sin(this->ik_solutions(i,0)) - desired_pos(1,0)*std::cos(this->ik_solutions(i,0))),2) + std::pow(desired_pos(0,1)*std::sin(this->ik_solutions(i,0)) - desired_pos(1,1)*std::cos(this->ik_solutions(i,0)),2)), desired_pos(0,2)*std::sin(this->ik_solutions(i,0)) - desired_pos(1,2)*std::cos(this->ik_solutions(i,0)));
            // if(std::isnan(this->ik_solutions(i,4)))
            // {
            //     this->ik_solutions(i,4) = std::atan2(0, desired_pos(0,2)*std::sin(this->ik_solutions(i,0)) - desired_pos(1,2)*std::cos(this->ik_solutions(i,0)));
            // }
            
            this->ik_solutions(i+2,4) = std::atan2( - std::sqrt(std::pow((desired_pos(0,0)*std::sin(this->ik_solutions(i+2,0)) - desired_pos(1,0)*std::cos(this->ik_solutions(i+2,0))),2) + std::pow(desired_pos(0,1)*std::sin(this->ik_solutions(i+2,0)) - desired_pos(1,1)*std::cos(this->ik_solutions(i+2,0)),2)), desired_pos(0,2)*std::sin(this->ik_solutions(i+2,0)) - desired_pos(1,2)*std::cos(this->ik_solutions(i+2,0)));
            // if(std::isnan(this->ik_solutions(i+2,4)))
            // {
            //     this->ik_solutions(i+2,4) = std::atan2(0, desired_pos(0,2)*std::sin(this->ik_solutions(i+2,0)) - desired_pos(1,2)*std::cos(this->ik_solutions(i+2,0)));
            // }
            
            this->ik_solutions(i+4,4) = std::atan2(std::sqrt(std::pow((desired_pos(0,0)*std::sin(this->ik_solutions(i+4,0)) - desired_pos(1,0)*std::cos(this->ik_solutions(i+4,0))),2) + std::pow(desired_pos(0,1)*std::sin(this->ik_solutions(i+4,0)) - desired_pos(1,1)*std::cos(this->ik_solutions(i+4,0)),2)), desired_pos(0,2)*std::sin(this->ik_solutions(i+4,0)) - desired_pos(1,2)*std::cos(this->ik_solutions(i+4,0)));
            // if(std::isnan(this->ik_solutions(i+4,4)))
            // {
            //     this->ik_solutions(i+4,4) = std::atan2(0, desired_pos(0,2)*std::sin(this->ik_solutions(i+4,0)) - desired_pos(1,2)*std::cos(this->ik_solutions(i+4,0)));
            // }
            
            this->ik_solutions(i+4+2,4) = std::atan2( - std::sqrt(std::pow((desired_pos(0,0)*std::sin(this->ik_solutions(i+2+4,0)) - desired_pos(1,0)*std::cos(this->ik_solutions(i+2+4,0))),2) + std::pow(desired_pos(0,1)*std::sin(this->ik_solutions(i+2+4,0)) - desired_pos(1,1)*std::cos(this->ik_solutions(i+2+4,0)),2)), desired_pos(0,2)*std::sin(this->ik_solutions(i+2+4,0)) - desired_pos(1,2)*std::cos(this->ik_solutions(i+2+4,0)));
            // if(std::isnan(this->ik_solutions(i+2+4,4)))
            // {
            //     this->ik_solutions(i+2+4,4) = std::atan2(0, desired_pos(0,2)*std::sin(this->ik_solutions(i+2+4,0)) - desired_pos(1,2)*std::cos(this->ik_solutions(i+2+4,0)));
            // }
        }

        //calcolo theta6

        for (int i = 0; i < 8; i++)
        {   
            if ((this->ik_solutions(i,4)) != 0 && (this->ik_solutions(i,4)) != M_PI)
            {
                this->ik_solutions(i,5) = std::atan2(-(desired_pos(0,1)*std::sin(this->ik_solutions(i,0)) - desired_pos(1,1)*std::cos(this->ik_solutions(i,0)))/std::sin(this->ik_solutions(i,4)), (desired_pos(0,0)*std::sin(this->ik_solutions(i,0)) - desired_pos(1,0)*std::cos(this->ik_solutions(i,0)))/std::sin(this->ik_solutions(i,4)));
            }
            else
            {
                std::cout << "Theta 6 not defined" << std::endl;
                std::cout << "Singular Configuration " << std::endl;
                return false;
            }
            
        }

        //calcolo theta2, theta3 e theta4

        for (int i = 0; i < 8; i++)
        {   
            if ((this->ik_solutions(i,4)) != 0 && (this->ik_solutions(i,4)) != M_PI)
            {
                //calcolo theta234
                float theta234 = std::atan2(- desired_pos(2,2)/std::sin(this->ik_solutions(i,4)), - (desired_pos(0,2)*std::cos(this->ik_solutions(i,0)) + desired_pos(1,2)*std::sin(this->ik_solutions(i,0)))/std::sin(this->ik_solutions(i,4)));
                // if(std::isnan(theta234))
                // {
                //     std::cout << "Theta 234 not defined" << std::endl;
                //     std::cout << "Singular Configuration " << std::endl;
                //     return false;
                // }

                //calcolo theta2
                float A = desired_pos(0,3)*std::cos(this->ik_solutions(i,0)) + desired_pos(1,3)*std::sin(this->ik_solutions(i,0)) - this->d5*std::sin(theta234) + this->d6*std::sin(this->ik_solutions(i,4))*std::cos(theta234);
                
                float B = desired_pos(2,3) - this->d1 + this->d5*std::cos(theta234) + this->d6*std::sin(this->ik_solutions(i,4))*std::sin(theta234);

                float C = 2*this->a2*std::sqrt(std::pow(A,2) + std::pow(B,2));

                if (i%2 == 0)
                {
                    this->ik_solutions(i,1) = std::atan2( (std::pow(A,2) + std::pow(B,2) + std::pow(this->a2,2) - std::pow(this->a3,2))/C, std::sqrt(1-std::pow((std::pow(A,2) + std::pow(B,2) + std::pow(this->a2,2) - std::pow(this->a3,2))/C ,2))) - std::atan2(A,B);
 
                    if (std::isnan(this->ik_solutions(i,1)) && !std::isnan(theta234))
                    {
                        this->ik_solutions(i,1) = std::atan2( (std::pow(A,2) + std::pow(B,2) + std::pow(this->a2,2) - std::pow(this->a3,2))/C, 0) - std::atan2(A,B);
                    }       
                }
                else
                {
                    this->ik_solutions(i,1) = std::atan2((std::pow(A,2) + std::pow(B,2) + std::pow(this->a2,2) - std::pow(this->a3,2))/C, - std::sqrt(1-std::pow((std::pow(A,2) + std::pow(B,2) + std::pow(this->a2,2) - std::pow(this->a3,2))/C ,2))) - std::atan2(A,B);
                    
                    if (std::isnan(this->ik_solutions(i,1)) && !std::isnan(theta234))
                    {
                        this->ik_solutions(i,1) = std::atan2( (std::pow(A,2) + std::pow(B,2) + std::pow(this->a2,2) - std::pow(this->a3,2))/C, 0) - std::atan2(A,B);
                    }
                    
                }

                //calcolo theta23
                float theta23 = std::atan2((desired_pos(2,3)-this->d1+this->d5*std::cos(theta234)-this->a2*std::sin(this->ik_solutions(i,1))+this->d6*std::sin(this->ik_solutions(i,4))*std::sin(theta234))/this->a3, (desired_pos(0,3)*std::cos(this->ik_solutions(i,0))+desired_pos(1,3)*std::sin(this->ik_solutions(i,0))-this->d5*std::sin(theta234)-this->a2*std::cos(this->ik_solutions(i,1))+this->d6*std::sin(this->ik_solutions(i,4))*std::cos(theta234))/this->a3);
                
                //calcolo theta3
                this->ik_solutions(i,2) = theta23 - this->ik_solutions(i,1);

                //calcolo theta4
                this->ik_solutions(i,3) = theta234 - this->ik_solutions(i,2) - this->ik_solutions(i,1);
                
            }
            else
            {
                std::cout << "Theta 2, 3 and 4 not defined" << std::endl;
                std::cout << "Singular Configuration " << std::endl;
                return false;
            }
        }

        //riporto i risulatati nell'intervallo [-pi,pi]
        for (int i = 0; i < 8; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                if (this->ik_solutions(i,j) > M_PI)
                {
                    this->ik_solutions(i,j) = this->ik_solutions(i,j) - 2*M_PI;
                }
                else if (this->ik_solutions(i,j) < -M_PI)
                {
                    this->ik_solutions(i,j) = this->ik_solutions(i,j) + 2*M_PI;
                }
            }
        }

        // std::cout << "IK solutions: " << std::endl;
        // std::cout << this->ik_solutions << std::endl;

        return true;
    }


    bool FIKServer::ik_cb(ur_kinematics::UrInverseKinematics::Request &req, ur_kinematics::UrInverseKinematics::Response &res)
    {   

        std::vector<int> desired_config = req.desired_config;
        // ordina il vettore desired config in ordine crescente
        std::sort(desired_config.begin(), desired_config.end());

        // DH parameters
        this->ur_type = req.ur_type;
        this->coefficients();

        // correzioni per il mio sistema di riferimento

        //matrice di rotazione di -90 gradi attorno all'asse x
        Eigen::AngleAxisf rot_x(-M_PI/2, Eigen::Vector3f::UnitX());
        Eigen::Matrix3f Rx = rot_x.toRotationMatrix();

        //matrice di rotazione di -90 gradi attorno all'asse z
        Eigen::AngleAxisf rot_z(-M_PI/2, Eigen::Vector3f::UnitZ());
        Eigen::Matrix3f Rz = rot_z.toRotationMatrix();

        // vettore di quaternioni
        std::vector<Eigen::Quaternionf> q;
        
        // fill the vector
        for (int i = 0; i < req.reference_pose.size(); i++)
        {
            Eigen::Quaternionf temp;
            temp.x() = req.reference_pose[i].orientation.x;
            temp.y() = req.reference_pose[i].orientation.y;
            temp.z() = req.reference_pose[i].orientation.z;
            temp.w() = req.reference_pose[i].orientation.w;
            q.push_back(temp);
        }

        // vettore di matrici di rotazione
        std::vector<Eigen::Matrix3f> R;

        // fill the vector
        for (int i = 0; i < q.size(); i++)
        {
            Eigen::Matrix3f temp = q[i].toRotationMatrix();
            R.push_back(temp);
        }

        // vettore di omonogenee di riferimento
        std::vector<M4f> reference;

        // fill the vector
        for (int i = 0; i < R.size(); i++)
        {
            M4f temp = Eigen::Matrix4f::Identity();
            for(int j = 0; j<3 ;j++){
                for(int k = 0; k<3 ;k++){
                    temp(j,k) = R[i](j,k);
                }
            }
            temp(0,3) = req.reference_pose[i].position.x;
            temp(1,3) = req.reference_pose[i].position.y;
            temp(2,3) = req.reference_pose[i].position.z;
            reference.push_back(temp);
        }

        //inizializzo la matrice di soluzioni
        this->res_new.complete_solution.resize(reference.size());
        this->res_new.solution.resize(reference.size());

        unsigned int iteratore = 0;
        
        for (int i = 0; i < reference.size(); i++)
        {
            //correzione per il mio sistema di riferimento1
            reference[i].block<3,3>(0,0) = reference[i].block<3,3>(0,0)*Rz;
            reference[i].block<3,3>(0,0) = reference[i].block<3,3>(0,0)*Rx;

            //calcolo la IK
            bool ik_ok = this->invKine(reference[i]);

            if (!ik_ok)
            {
                // service call failed
                res.success = false;
                ROS_ERROR("IK service call failed");
                ROS_ERROR("Position: %f %f %f", req.reference_pose[i].position.x, req.reference_pose[i].position.y, req.reference_pose[i].position.z);
                ROS_ERROR("Orientation: %f %f %f %f", req.reference_pose[i].orientation.x, req.reference_pose[i].orientation.y, req.reference_pose[i].orientation.z, req.reference_pose[i].orientation.w);
                return res.success;
            }

            //resize
            this->res_new.complete_solution[i].joint_matrix.resize(8);
            this->res_new.solution[i].joint_matrix.resize(desired_config.size());

            //riempio la matrice di soluzioni
            for (int j = 0; j < 8; j++)
            {
                this->res_new.complete_solution[i].joint_matrix[j].data.resize(6);
                for (int k = 0; k < 6; k++)
                {
                    this->res_new.complete_solution[i].joint_matrix[j].data[k] = this->ik_solutions(j,k);
                }

                if (j == desired_config[iteratore] && iteratore < desired_config.size())
                {
                    this->res_new.solution[i].joint_matrix[iteratore].data.resize(6);
                    for (int k = 0; k < 6; k++)
                    {
                        this->res_new.solution[i].joint_matrix[iteratore].data[k] = this->ik_solutions(j,k);
                    }
                    iteratore++;
                }
            }

            //resetto l'iteratore
            iteratore = 0;
        }

        this->res_new.move_q6.resize(8);
        // set at zero
        for (int i = 0; i < this->res_new.move_q6.size(); i++)
        {
            this->res_new.move_q6[i] = 0.0;
        }
        
        // chiamo funzione di controllo per l'ultimo giunto
        if(req.check_q6)
        {
            bool result = this->checkq6();

            if (result)
            {
                iteratore = 0;

                // riempio la matrice di soluzioni
                for (int i = 0; i < this->res_new.complete_solution.size(); i++)
                {
                    for (int j = 0; j < this->res_new.complete_solution[i].joint_matrix.size(); j++)
                    {
                        if (j == desired_config[iteratore] && iteratore < desired_config.size())
                        {
                            this->res_new.solution[i].joint_matrix[iteratore].data[5] = this->res_new.complete_solution[i].joint_matrix[j].data[5];
                            iteratore++;
                        }
                    }

                    iteratore = 0;
                }
            }
        }

        // check the joint limits
        
        for (int i = 0; i < this->res_new.complete_solution.size(); i++)
        {
            for (int j = 0; j < this->res_new.complete_solution[i].joint_matrix.size(); j++)
            {
                int count = 0;
                for (int k = 0; k < this->res_new.complete_solution[i].joint_matrix[j].data.size(); k++)
                {
                    if (this->res_new.complete_solution[i].joint_matrix[j].data[k] < this->joint_limits[count] || this->res_new.complete_solution[i].joint_matrix[j].data[k] > this->joint_limits[count+1])
                    {
                        res.success = false;
                        ROS_ERROR("Joint limits exceeded!");
                        std::cout << "Joint " << k+1 << " value: " << this->res_new.complete_solution[i].joint_matrix[j].data[k] << std::endl;
                        std::cout << "Joint Limits: " << this->joint_limits[count] << " " << this->joint_limits[count+1] << std::endl;

                        return res.success;
                    }
                    count += 2;
                }
            }
        }


        // std::cout << "IK solutions: " << std::endl;
        // std::cout << res << std::endl;

        res.complete_solution = this->res_new.complete_solution;
        res.solution = this->res_new.solution;
        res.move_q6 = this->res_new.move_q6;

        res.success = true;

        return res.success;
    }


    bool FIKServer::checkq6()
    {
        bool check = false;



        for (int i =1; i < this->res_new.complete_solution.size(); i++)
        {
            for (int j = 0; j < this->res_new.complete_solution[i].joint_matrix.size(); j++)
            {
                float diff = this->res_new.complete_solution[i].joint_matrix[j].data[5] - this->res_new.complete_solution[i-1].joint_matrix[j].data[5];
                if(diff > M_PI)
                {
                    // nuovo intervallo [-pi, 3pi]
                    this->res_new.complete_solution[i].joint_matrix[j].data[5] -= 2*M_PI;
                    check = true;
                }
                else if(diff < -M_PI)
                {
                    // nuovo intervallo [-3pi, pi]
                    this->res_new.complete_solution[i].joint_matrix[j].data[5] += 2*M_PI;
                    check = true;
                }
            }
        }

        if(check)
        {
            // cerco in tutti i punti e in tutte le configurazioni se l'ultimo giunto supera i limiti [-2pi, 2pi]
            for (int i = 0; i < this->res_new.complete_solution.size(); i++)
            {
                for (int j = 0; j < this->res_new.complete_solution[i].joint_matrix.size(); j++)
                {
                    if (this->res_new.complete_solution[i].joint_matrix[j].data[5] > 2*M_PI)
                    {
                        // sottraggo 2pi a tutti i punti della configurazione attuale
                        for (int k = 0; k < this->res_new.complete_solution.size(); k++)
                        {
                            this->res_new.complete_solution[k].joint_matrix[j].data[5] -= 2*M_PI;
                        }
                        this->res_new.move_q6[j] = -2*M_PI;
                    }
                    else if (this->res_new.complete_solution[i].joint_matrix[j].data[5] < -2*M_PI)
                    {
                        // sommo 2pi a tutti i punti della configurazione attuale
                        for (int k = 0; k < this->res_new.complete_solution.size(); k++)
                        {
                            this->res_new.complete_solution[k].joint_matrix[j].data[5] += 2*M_PI;
                        }
                        this->res_new.move_q6[j] = 2*M_PI;
                    }
                }
            }
        }


        return check;
    }
}
