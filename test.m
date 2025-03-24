
Tsci = [ 1, 0, 0, 1; 
         0, 1, 0, 1.2;
         0, 0, 1, .0248;
         0, 0, 0, 1];


Tscf = [ 0, -1, 0, .05;
         1, 0, 0,-1.047;
         0, 0, 1, .075;
         0, 0, 0, 1];

 current_robot_config = [0.5238,-.3,0,0,0,0,0,0,0,0,0,0,0];



% Robot configuration without deviation (uncomment to use)
 %current_robot_config = [ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ];

Tse = [0,0,1,0;
              0,1,0,0;
              -1,0,0,.5 ;
              0,0,0,1];

% Control gains (Proportional and Integral)
Kp = diag([7,7,7,7,7,7]);  % A balanced starting point
 Ki = diag([0,0,0,0,0,0]);  % Integral gains (can be tuned)


main(Tsci, Tscf, current_robot_config, Tse, Kp, Ki);


