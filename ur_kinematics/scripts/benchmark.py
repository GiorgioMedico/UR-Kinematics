#! /usr/bin/env python3

import rospy
import numpy as np
from ur_kinematics.srv import *
from geometry_msgs.msg import Pose
import copy
from std_msgs.msg import Float64MultiArray
from termcolor import colored, cprint

class Benchmark:

    def __init__(self):

        self.duration = 0
        self.duration_f = 0

        self.num_of_tests = 1

        self.average_error = 0
        self.max_error = 0

        self.poses = [Pose() for i in range(self.num_of_tests)]

        self.forward_kinematics_service = rospy.ServiceProxy('ur_forward_k', UrForwardKinematics)
        self.inverse_kinematics_service = rospy.ServiceProxy('ur_inverse_k', UrInverseKinematics)

        self.benchmark(self.num_of_tests)

        cprint('Benchmark done', 'green', attrs=['bold'])
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


    def benchmark(self, num_of_tests):

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
      
        self.poses[0].position.x = -0.17078473023916108
        self.poses[0].position.y = -0.5772072659357064
        self.poses[0].position.z = 0.29273325406756945
        self.poses[0].orientation.x = -0.9868896566015951
        self.poses[0].orientation.y = -0.0011259553397428867
        self.poses[0].orientation.z = 0.16139227636023654
        self.poses[0].orientation.w = 0.0002665494694868573

        req_ik.reference_pose = copy.deepcopy(self.poses)

        # misuro tempo di risposta del servizio
        start = rospy.get_time()

        ik_res = self.inverse_kinematics_service(req_ik)

        end = rospy.get_time()

        if(self.num_of_tests == 1):
            # metto la complete solution in un array np
            complete_solution = np.zeros((self.num_of_tests, 8, 6))
            for i in range(self.num_of_tests):
                for j in range(8):
                    for k in range(6):
                        complete_solution[i][j][k] = ik_res.complete_solution[i].joint_matrix[j].data[k]
            # metto la solution in un array np
            solution = np.zeros((self.num_of_tests,len(req_ik.desired_config), 6))
            for i in range(self.num_of_tests):
                for j in range(len(req_ik.desired_config)):
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

        for i in range(self.num_of_tests):

            # cerca la configurazione che corrisponde a joint_angles[i]
            for j in range(len(ik_res[i].joint_matrix)):

                if np.allclose(ik_res[i].joint_matrix[j].data, joint_angles[i]):
                    # calcola errore
                    error = np.linalg.norm(joint_angles[i] - ik_res[i].joint_matrix[j].data)
                    self.average_error += error
                    if error > self.max_error:
                        self.max_error = error

        self.average_error /= self.num_of_tests
            


if __name__ == '__main__':

    rospy.init_node('benchmark')
    benchmark = Benchmark()
