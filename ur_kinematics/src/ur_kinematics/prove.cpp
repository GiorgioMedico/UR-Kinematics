
// COPIA DELLE PROVE FATTE NEL COSTRUTTORE IN IK_ROBOTICS.CPP

        //prove

        // this->a2 = -0.4253630103489888;
        // this->a3 = -0.3924349641369202;

        // this->d1 =  0.162710833731481;
        // this->d4 =  0.1337873018461449;
        // this->d5 =  0.09960901169401305;
        // this->d6 =  0.09949603061275286;

        // this->alph1 = M_PI/2;
        // this->alph4 = M_PI/2;
        // this->alph5 = -M_PI/2;

        // // Forward Kinematics
        // this->q << -1.80072096, -1.96838061, -1.45026769, -1.89632231, 0, 0.93083217;
        // // this->q << 0, 0, 0, 0, 0, 0;
        // std::cout<< "Configurazione in giunto" <<std::endl;
        // std::cout<<this->q<<std::endl;
        // this->HTrans();
        // M4f T_6 = T_0_6;

        // std::cout<<std::endl;
        // std::cout<< "Omogenea" <<std::endl;
        // std::cout<<T_6<<std::endl;

        // // da matrice a posizione e orientamento
        // Eigen::Matrix3f R;
        // for(int i = 0; i<3 ;i++){
        //     for(int j = 0; j<3 ;j++){
        //         R(i,j) = T_6(i,j);
        //     }
        // }
        // Eigen::Quaternionf q1(R);

        // std::cout<<std::endl;

        // std::cout<< "Posizione e orientamneto (in quaternioni)" <<std::endl;

        // std::cout<<T_6(0,3)<<","<<T_6(1,3)<<","<<T_6(2,3)<<","<<q1.x()<<","<<q1.y()<<","<<q1.z()<<","<<q1.w()<<std::endl;
        

        
        // Prova IK con T_6

        // Eigen::Quaternionf q;
        // q.x() = -0.9868896566015951;
        // q.y() = -0.0011259553397428867;
        // q.z() = 0.16139227636023654;
        // q.w() = 0.0002665494694868573;
        
        // Eigen::Matrix3f R = q.toRotationMatrix();
                
        // M4f T_6 = M4f::Identity();

        // for(int i = 0; i<3 ;i++){
        //     for(int j = 0; j<3 ;j++){
        //         T_6(i,j) = R(i,j);
        //     }
        // }
        // T_6(0,3) = -0.17078473023916108;
        // T_6(1,3) = -0.5772072659357064;
        // T_6(2,3) = 0.29273325406756945;

        // // correzioni per il mio sistema di riferimento

        // //matrice di rotazione di 90 gradi attorno all'asse x
        // Eigen::AngleAxisf rot_x(-M_PI/2, Eigen::Vector3f::UnitX());
        // Eigen::Matrix3f Rx = rot_x.toRotationMatrix();

        // //matrice di rotazione di 90 gradi attorno all'asse z
        // Eigen::AngleAxisf rot_z(-M_PI/2, Eigen::Vector3f::UnitZ());
        // Eigen::Matrix3f Rz = rot_z.toRotationMatrix();

        // T_6.block<3,3>(0,0) = T_6.block<3,3>(0,0)*Rz;
        // T_6.block<3,3>(0,0) = T_6.block<3,3>(0,0)*Rx;

        // this->invKine(T_6);

        // // vado a capo
        // std::cout<<std::endl;
        // std::cout<< "Configurazioni in giunto della IK" <<std::endl;

        // std::cout<<this->ik_solutions<<std::endl;