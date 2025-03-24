function J = computeJacobian(jointState)
    % Example computation of the Jacobian for a 5 DOF manipulator
    % jointState is a vector of joint angles [theta1, theta2, ..., theta5]
    
    % Define the robot's link lengths (example values)
    l1 = 1;  % Link 1 length
    l2 = 1;  % Link 2 length
    l3 = 1;  % Link 3 length
    l4 = 1;  % Link 4 length
    l5 = 1;  % Link 5 length
    
    % Joint angles (example: 5 DOF robot arm)
    theta1 = jointState(1);
    theta2 = jointState(2);
    theta3 = jointState(3);
    theta4 = jointState(4);
    theta5 = jointState(5);
    
    % Compute the Jacobian matrix for a 5-DOF manipulator (this is simplified)
    % You need to use the actual kinematic model of your robot here
    
    % Placeholder Jacobian computation for a 5-DOF robot (modify for your robot)
    J = [ -l1*sin(theta1) - l2*sin(theta1 + theta2) - l3*sin(theta1 + theta2 + theta3), -l2*sin(theta1 + theta2) - l3*sin(theta1 + theta2 + theta3), -l3*sin(theta1 + theta2 + theta3), 0, 0;
           l1*cos(theta1) + l2*cos(theta1 + theta2) + l3*cos(theta1 + theta2 + theta3), l2*cos(theta1 + theta2) + l3*cos(theta1 + theta2 + theta3), l3*cos(theta1 + theta2 + theta3), 0, 0;
           0, 0, 0, 0, 0;
           0, 0, 0, cos(theta4), 0;
           0, 0, 0, sin(theta5), 1];
end