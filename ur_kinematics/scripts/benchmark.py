#! /usr/bin/env python3

import rospy
import numpy as np
from ur_kinematics.srv import *
from geometry_msgs.msg import Pose
import copy
from std_msgs.msg import Float64MultiArray
from termcolor import colored, cprint
import pandas as pd
import rospkg
import os
# sopprimo future warning
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)
from tqdm import tqdm

class Benchmark:

    def __init__(self):

        self.save_data = False

        self.tolerance = 1e-3

        self.check_q6 = True
        self.ik_move_q6 = 0

        self.duration = 0
        self.duration_f = 0

        self.num_of_tests = 1000

        self.average_error = 0
        self.max_error = 0

        self.poses = [Pose() for i in range(self.num_of_tests)]

        rospack = rospkg.RosPack()
        rospack.list() 
        self.dir_path = rospack.get_path('ur_kinematics')
        # cerco nella cartella
        file_list = os.listdir(self.dir_path+'/ik_error')

        if len(file_list) == 0:
            self.df_pose_not_found = pd.DataFrame(columns=['Px','Py','Pz','Qx','Qy','Qz','Qw'])
        else:
            self.df_pose_not_found = pd.read_csv(self.dir_path+'/ik_error/pose_not_found.csv')

        self.forward_kinematics_service = rospy.ServiceProxy('ur_forward_k', UrForwardKinematics)
        self.inverse_kinematics_service = rospy.ServiceProxy('ur_inverse_k', UrInverseKinematics)

        self.benchmark(self.num_of_tests)

        print('')
        cprint('Benchmark done', 'green', attrs=['bold'])
        print('')
        print('Number of tests: ', self.num_of_tests)
        print('')
        print('Average error: ', self.average_error, 'rad')
        print('Max error: ', self.max_error, 'rad')
        print('')
        print('Duration IK: ', self.duration, 's')
        print('Average time IK: ', self.duration/self.num_of_tests, 's')
        print('')
        print('Duration FK: ', self.duration_f, 's')
        print('Average time FK: ', self.duration_f/self.num_of_tests, 's')

        if self.check_q6:
            print('')
            print('Q6 moved: ', self.ik_move_q6, 'Rads')
            print('')


    def benchmark(self, num_of_tests):



        if self.check_q6:

            # creo un array di array di 6 elementi
            joint_angles = np.zeros((int(self.num_of_tests), 6))
            # il primo elemento per tutti gli array è 0, il secondo è -1.57, il terzo è 0, il quarto è 0, il quinto è pi/2 e l'ultimo è crecente da 0 a 3pi
            joint_angles[:, 1] = -1.57
            joint_angles[:, 2] = 0
            joint_angles[:, 3] = 0
            joint_angles[:, 4] = np.pi/2
            joint_angles[:, 5] = np.linspace(np.deg2rad(10), np.deg2rad(360+10), int(num_of_tests))

            # joint_angles2 = np.zeros((int(self.num_of_tests/2), 6))
            # joint_angles2[:, 1] = -1.57
            # joint_angles2[:, 2] = 0
            # joint_angles2[:, 3] = 0
            # joint_angles2[:, 4] = np.pi/2
            # joint_angles2[:, 5] = np.linspace(3/2*np.pi, np.pi/2, int(self.num_of_tests/2))

            # # concateno i due array
            # joint_angles = np.concatenate((joint_angles, joint_angles2))

        else:
            # generate random joint angles [-pi, pi]
            joint_angles = np.random.uniform(-np.pi, np.pi, (num_of_tests, 6))

        buffer = [Float64MultiArray() for i in range(num_of_tests)]

        # fill buffer with joint angles
        for i in range(num_of_tests):
            buffer[i].data = copy.deepcopy(joint_angles[i])

        # call forward kinematics service
        req = UrForwardKinematicsRequest()

        cprint('Starting FK...', 'magenta')
        
        start_f = rospy.get_time()

        req.ur_type = 'UR5e'
        
        req.reference_joints = copy.deepcopy(buffer)

        res = self.forward_kinematics_service(req)
        self.poses = copy.deepcopy(res.reference_pose)

        end_f = rospy.get_time()

        self.duration_f = end_f - start_f

        cprint('FK done', 'green', attrs=['bold'])

        # call inverse kinematics service
        req_ik = UrInverseKinematicsRequest()

        cprint('Starting IK...', 'magenta')

        req_ik.ur_type = 'UR5e'
        req_ik.desired_config = [2,4,7]

        req_ik.check_q6 = self.check_q6

        req_ik.reference_pose = copy.deepcopy(self.poses)

        # misuro tempo di risposta del servizio
        start = rospy.get_time()

        ik_res = self.inverse_kinematics_service(req_ik)

        self.ik_move_q6 = ik_res.move_q6[0]

        if ik_res.success == False:
            cprint('IK failed', 'red', attrs=['bold'])
            return

        end = rospy.get_time()

        if(self.num_of_tests == 10):
            # metto la complete solution in un array np
            complete_solution = np.zeros((self.num_of_tests, 8, 6))
            for i in range(self.num_of_tests):
                for j in range(8):
                    for k in range(6):
                        complete_solution[i][j][k] = ik_res.complete_solution[i].joint_matrix[j].data[k]
            # metto la solution in un array np
            solution = np.zeros((self.num_of_tests,len(req_ik.desired_config), 6))
            for i in range(len(ik_res.solution)):
                for j in range(len(ik_res.solution[i].joint_matrix)):
                    for k in range(6):
                        solution[i][j][k] = ik_res.solution[i].joint_matrix[j].data[k]


            print('IK complete solution: ', complete_solution)
            print('IK solution: ', solution)
        
        self.duration = end - start

        cprint('IK done', 'green', attrs=['bold'])
     
        buffer = ik_res.complete_solution

        cprint('Starting error calculation...', 'magenta')

        # chiama funzione per calcolare errore
        self.error(joint_angles, buffer)


    def error(self, joint_angles, ik_res):

        # cerca in ik_res la configurazione che corrisponde a quella di joint_angles e calcola l'errore

        # nan counter
        nan_counter = 0

        conf_not_found = 0

        for i in range(self.num_of_tests):

            # cerca la configurazione che corrisponde a joint_angles[i]
            for j in range(len(ik_res[i].joint_matrix)):

                if np.allclose(ik_res[i].joint_matrix[j].data, joint_angles[i] + self.ik_move_q6, atol=self.tolerance):
                    # calcola errore
                    error = np.linalg.norm(joint_angles[i] - ik_res[i].joint_matrix[j].data)
                    self.average_error += error
                    if error > self.max_error:
                        self.max_error = error
                    break

                elif np.isnan(ik_res[i].joint_matrix[j].data).any():
                    nan_counter += 1
                    conf_not_found += 1
                    break

                elif j == len(ik_res[i].joint_matrix) - 1:
                    
                    self.df_pose_not_found = self.df_pose_not_found.append({'Px': self.poses[i].position.x, 'Py': self.poses[i].position.y, 'Pz': self.poses[i].position.z, 'Qx': self.poses[i].orientation.x, 'Qy': self.poses[i].orientation.y, 'Qz': self.poses[i].orientation.z, 'Qw': self.poses[i].orientation.w}, ignore_index=True)                   
                            
                    # print('')
                    # cprint('Configuration not found', 'red', attrs=['bold'])
                    # print(joint_angles[i])
                    conf_not_found += 1

        if not self.check_q6 and self.save_data:
            cprint('Saving data...', 'magenta')
            # salvo in un csv il dataframe
            self.df_pose_not_found.to_csv(self.dir_path + '/ik_error/pose_not_found.csv', index=False)

        if nan_counter > 0:
            print('')
            cprint('NaN found', 'red', attrs=['bold'])
            print('Number of NaN: ', nan_counter)
        else:
            print('')
            cprint('No NaN found', 'green', attrs=['bold'])
        
        if conf_not_found > 0:
            print('')
            cprint('Configurations not found', 'red', attrs=['bold'])
            print('Number of configurations not found (NaN excluded): ', conf_not_found-nan_counter)
            print('Tollerance: ', self.tolerance)
            print('Number of configurations not found (NaN included): ', conf_not_found)
            print('Number of configurations found: ', self.num_of_tests - conf_not_found)
            # percentuale di errore
            print('')
            print('Error percentage: ', conf_not_found/self.num_of_tests*100, '%')
        else:
            print('')
            cprint('All configurations found', 'green', attrs=['bold'])
            print('Number of configurations found: ', self.num_of_tests - conf_not_found)
        
        self.average_error /= self.num_of_tests
            


if __name__ == '__main__':

    rospy.init_node('benchmark')
    
    for i in tqdm(range(1)):

        if rospy.is_shutdown():
            break
        else:
            benchmark = Benchmark()
