%This script takes inputs of the joint angles, joint velocities and joint 
%accelerations and outputs the torques for each joint and displays it in 
%the terminal. 

close all 
clear all 
clc 

%mass of the links 
syms m1; 
syms m2; 
syms m3; 

%link lengths 
syms a1; 
syms a2; 
syms a3; 
syms a4; 

%lengths from the motor to the centre of mass(COM)
%motor 1 to COM of link 1
l1z = 0.089736;
l1y = 0.001676;
l1x = 0.009189;
%hyponenuse of x and y (distance from base to mass 1)
l1h = 0.00965;

%motor 2 to COM of link 2
l2z = 0;
l2y = 0.026217;
l2x = 0.180954;
%hyponenuse of x and y (distance from base to mass 2)
l2h = 0.1828;

%motor 3 to COM of link 3
l3z = 0;
l3y = 0;
l3x = 0.200000;

%acceleration due to gravity 
g = 9.81; 

%joint angles 
syms t1(t); 
syms t2(t); 
syms t3(t); 

%Joint velocities 
syms t1d(t); 
syms t2d(t); 
syms t3d(t); 

%the angle of mass 1 and mass 2
m1a = 0.174533;
m2a = 0.139626;

%Velocities of frame 1, 2 and 3 in the x, y and z directions 
x1d = -l1h*sin(t1(t) + m1a)*t1d(t); 
y1d = l1h*cos(t1(t) + m1a)*t1d(t); 
z1d = 0; 

x2d = (-sin(t1(t))*l2h*cos(t2(t) + m2a) - a2*sin(t1(t)))*t1d(t) + (-cos(t1(t))*l2h*sin(t2(t) + m2a))*t2d(t); 
y2d = (cos(t1(t))*l2h*cos(t2(t) + m2a) + a2*cos(t1(t)))*t1d(t) - sin(t1(t))*l2h*sin(t2(t) + m2a)*t2d(t); 
z2d = l2h*cos(t2(t) + m2a)*t2d(t); 

x3d = - a2*sin(t1(t))*t1d - a3*cos(t2(t))*sin(t1(t))*t1d - a3*cos(t1(t))*sin(t2(t))*t2d - 0.5*a4*sin(t1(t))*cos(t2(t) + t3(t))*t1d - 0.5*a4*cos(t1(t))*sin(t2(t) + t3(t))*(t2d + t3d);
y3d = a2*cos(t1(t))*t1d + a3*cos(t1(t))*cos(t2(t))*t1d - a3*sin(t1(t))*sin(t2(t))*t2d + 0.5*a4*cos(t1(t))*cos(t2(t) + t3(t))*t1d - 0.5*a4*sin(t1(t))*sin(t2(t) + t3(t))*(t2d + t3d);
z3d = 0.5*a4*cos(t2(t) + t3(t))*(t2d + t3d) + a3*cos(t2(t))*t2d;

%finding the square of the velocities 
v1 = x1d*x1d + y1d*y1d + z1d*z1d; 
v2 = x2d*x2d + y2d*y2d + z2d*z2d; 
v3 = x3d*x3d + y3d*y3d + z3d*z3d; 

%kinetic energy of the 3 links 
k1 = 0.5*m1*v1; 
k2 = 0.5*m2*v2; 
k3 = 0.5*m3*v3; 

%portential energy of the 3 links. The potential energy is measured from 
%the bottom a1. P.E = mgh
p1 = 0; 
p2 = m2*g*l2h*sin(t2(t));
p3 = m3*g*(a3*sin(t2(t)) + 0.5*a4*sin(t3(t) + t2(t)));

%finding the lagrangian
L = k1 + k2 + k3 - p1 - p2 - p3;

%Torques for each joint
T1 = diff(diff(L,t1d),t) - diff(L,t1);
T2 = diff(diff(L,t2d),t) - diff(L,t2);
T3 = diff(diff(L,t3d),t) - diff(L,t3);

%Simplify expressions
SimpT1 = simplify(T1);
SimpT2 = simplify(T2);
SimpT3 = simplify(T3);