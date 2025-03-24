# Copeliaslim-Project-


The project involves implementing a control framework for a mobile manipulator to perform a 
pick-and-place task. The approach consists of three main components: trajectory generation, 
kinematics simulation, and feedback control. Each component plays a crucial role in ensuring 
accurate and stable execution of the task, with a focus on smooth motion and precise trajectory 
tracking. 


The trajectory generation is handled by the TrajectoryGenerator function, which constructs an 
eight-segment reference trajectory for the end-effector. The motion is interpolated using either 
Cartesian or Screw-based interpolation, with a quintic time-scaling method applied to ensure 
smooth transitions between segments. The trajectory defines key configurations such as the 
initial and final positions of the cube, intermediate standoff positions, and gripper actions (open 
or closed). By specifying the number of reference configurations per timestep, the function 
ensures a well-defined trajectory that the controller can follow precisely. 
The kinematics simulation is implemented through the NextState function, which updates the 
robot’s configuration at each timestep. It follows a first-order Euler integration approach to 
propagate the robot’s state based on given control inputs. The function accounts for velocity 
constraints to prevent excessive actuation, ensuring that both joint and wheel velocities remain 
within predefined limits. The chassis position is updated using a kinematic model derived from 
the wheel velocities, while the arm and wheel states are adjusted based on their respective 
velocities. This function serves as the foundation for predicting and simulating the robot’s motion 
under different control commands. 


The feedback control mechanism is designed using the FeedbackControl function, which 
employs a feedforward-plus-PI control strategy. The function computes the error between the 
current and desired end-effector configurations using which captures both rotational and 
translational discrepancies. Additionally, the integral of this error is maintained to enhance 
long-term accuracy. The commanded twist V is determined by combining the feedforward 
reference twist with proportional and integral corrections, ensuring effective trajectory tracking. 
The computed twist is then transformed into joint and wheel velocities using the Jacobian 
pseudoinverse, enabling precise execution of the desired motion. 


The main script integrates these components by initializing the cube and robot configurations, 
setting proportional and integral gains (, and executing the control loop. The system is tested in 
CoppeliaSim by generating and running CSV files that dictate the robot’s motion. By adjusting 
the control gains and trajectory parameters, different performance scenarios are analyzed, 
ensuring robust and efficient execution of the pick-and-place task.
