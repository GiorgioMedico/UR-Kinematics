# UR-Kinematics
UR Inverse and Forward Kinematics. The inverse kinematics is closed-form, so you get 8 solution.

Is a ROS service, you send a request with your point or trajectory and responde with the partial soluzion(res.solution) that you ask and the complete one(res.complete_solution).

The solution are defined in [-pi,pi] but for the 6 joint is [-2pi, 2pi].

![images](https://user-images.githubusercontent.com/104858347/231451074-66817aef-8ece-42a6-b8c0-33df89e40641.jpg)
