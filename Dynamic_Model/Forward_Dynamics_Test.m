% %This script takes inputs of the joint angles, voint velocities and joint
% %accelerations and outputs the accelerations for each joint and displays it in
% %the terminal.
% %
% %work on this was discontinued because it was meant to be for the torque
% %controller, however we are not using the torque controller on the arm
% %because the arm is not fast enough to implement torque control.
close all 
clear all 
clc 
% 
% %mass of the links 
% m1 = 9.884; 
% m2 = 7.02; 
% m3 = 1.1902; 
% 
% % %mass of the links 
% % m1 = 0; 
% % m2 = 0; 
% % m3 = 1; 
% 
% %link lengths 
% a1 = -0.2; 
% a2 = 0.037; 
% a3 = 0.4; 
% a4 = 0.4; 
% 
% % %link lengths 
% % a1 = 0; 
% % a2 = 0; 
% % a3 = 0; 
% % a4 = 0; 
% 
% %lengths from the motor to the centre of mass(COM)
% %motor 1 to COM of link 1
% l1z = 0.089736;
% l1y = 0.001676;
% l1x = 0.009189;
% %hyponenuse of x and y (distance from base to mass 1)
% l1h = 0.00965;
% 
% %motor 2 to COM of link 2
% l2z = 0;
% l2y = 0.026217;
% l2x = 0.180954;
% %hyponenuse of x and y (distance from motor 1 to mass 2)
% l2h = 0.1828;
% 
% %motor 3 to COM of link 3
% l3z = 0;
% l3y = 0;
% l3x = 0.200000;
% 
% %acceleration due to gravity 
% g = 9.81; 
% 
% %joint angles 
% t1 = 0; 
% t2 = 0; 
% t3 = 0; 
% 
% %Joint velocities 
% t1d = 0; 
% t2d = 0; 
% t3d = 0; 
% 
% %Joint accelerations
% t1dd = 0; 
% t2dd = 0; 
% t3dd = 0; 
% 
% %Joint accelerations
% T1 = 0; 
% T2 = 0; 
% T3 = 0;
% 
% %the angle of mass 1 and mass 2
% m1a = 0.174533;
% m2a = 0.139626;
% Define variables
m1 = 0;
m2 = 0;
m3 = 1;
a1 = 1;
a2 = 1;
a3 = 1;
a4 = 1;
g = 9.81;
T1 = 0;
T2 = 100;
T3 = 0;
t1 = 0;
t2 = 0;
t3 = 0;
t1d = 0;
t2d = 0;
t3d = 0;
t3dd = 0;
t2dd = 0;
t1dd = 0;

t3dd = ((T3 - (981*a4*m3*cos(t2 + t3))/200 - (a4^2*m3*t2dd)/4 - (a4^2*m3*t3dd)/4 - a3*a4*m3*cos(t3)*t2dd - (a3*a4*m3*sin(2*t2 + t3)*t1d^2)/2 + a3*a4*m3*sin(t3)*t2d^2)/(a4^2*m3/4 + a4^2*m3/4 + 2*a2*a4*m3*cos(t2 + t3) + 2*a3*a4*m3*cos(t3))) - a3*a4*m3*sin(t3)*t2d/(a4^2*m3/4 + a4^2*m3/4 + 2*a2*a4*m3*cos(t2 + t3) + 2*a3*a4*m3*cos(t3))