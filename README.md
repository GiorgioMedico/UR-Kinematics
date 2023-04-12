# UR-Kinematics
UR Inverse and Forward Kinematics. The inverse kinematics is closed-form, so you get 8 solution.

Is a ROS service, you send a request with your point or trajectory and responde with the partial soluzion(res.solution) that you ask and the complete one(res.complete_solution).

The solution are defined in [-pi,pi] but for the 6 joint is [-2pi, 2pi].

![images](https://user-images.githubusercontent.com/104858347/231451074-66817aef-8ece-42a6-b8c0-33df89e40641.jpg)

The frame are arranged almost in the same way but only the last one is different. 
The end-effector frame has the x-azis in approach.

There is a benchmark file in python that test the performance of the solution given.

I tried to send 1M joints configuration to the FK and then send the solution to the IK.

The error between the joint configuration sent and the result are in order of e-7 in average and the max error is in order of e-5.

The FK give the solution in 40s and the IK in 70.
