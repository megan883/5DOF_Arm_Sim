function ANGULAR_TORQUES(T, L)

%% Inverse dynamics plot test
TESTING_INVERSE_DYNAMICS = 1;

%% Plotting the torques of each angular movement of the arm.
PLOT_TORQUES = 1;

%% Get the predicted circle path.
THETA_CIRC = IK_NEWTON_RAPHSON_CIRCLE_TRAJ(T, L);

%% Deliver a load.
LOAD = [0; 0; ((-1 * 9.81) * 0.5)];

%% Calculating the torques.
for i = 1:length(THETA_CIRC(1, :))
    [J_PINV, J] = GETTING_JACOBIAN(THETA_CIRC(:, i), L);
    
    TORQUES = J' * LOAD;
    
    STORE_TORQUES(:, i) = TORQUES;
end

%% Plot the torques at each circle point.
if (PLOT_TORQUES == 1)
    figure;
    
    hold on
    
    title("Joint Torque for 0.5KGF vertical and Z-axis load");
    
    plot(STORE_TORQUES(1, :), 'b.-');
    plot(STORE_TORQUES(2, :), 'r.-');
    plot(STORE_TORQUES(3, :), 'g.-');
    plot(STORE_TORQUES(4, :), 'm.-');
    plot(STORE_TORQUES(5, :), 'k.-');
    
    legend("TORQUE JOINT 1", "TORQUE JOINT 2", "TORQUE JOINT 3", "TORQUE JOINT 4", "TORQUE JOINT 5");
    xlabel("CIRCLE SAMPLE");
    ylabel("TORQUE [Nm]");
    
    hold off;
end

if( TESTING_INVERSE_DYNAMICS == 1)
    
    
    %% Calculating the torques.
    for i = 1:length(THETA_CIRC(1, :))
        
        ANGLE_ONE = THETA_CIRC(1, i);
        ANGLE_TWO = THETA_CIRC(2, i);
        ANGLE_THREE = THETA_CIRC(3, i);
        
        %mass of the links
        m1 = 9.884;
        m2 = 7.02;
        m3 = 1.1902;
        
        %link lengths
        a1 = -0.2;
        a2 = 0.037;
        a3 = 0.4;
        a4 = 0.4;
        
        %acceleration due to gravity
        g = 9.81;
        
        %joint angles
        t1 = ANGLE_ONE;
        t2 = ANGLE_TWO;
        t3 = ANGLE_THREE;
        
        %Joint velocities
        t1d = 0;
        t2d = 0;
        t3d = 0;
        
        %Joint accelerations
        t1dd = 0;
        t2dd = 0;
        t3dd = 0;
        
        T1 = (37249*m1*t1dd)/400000000 + (208849*m2*t1dd)/12500000 + a2^2*m2*t1dd + a2^2*m3*t1dd + (a3^2*m3*t1dd)/2 + (a4^2*m3*t1dd)/8 + (208849*m2*cos(2*t2 + 5030556812569863/18014398509481984)*t1dd)/12500000 - (208849*m2*sin(2*t2 + 5030556812569863/18014398509481984)*t1d*t2d)/6250000 + (457*a2*m2*cos(t2 + 5030556812569863/36028797018963968)*t1dd)/1250 + (a4^2*m3*cos(2*t2 + 2*t3)*t1dd)/8 + (a3^2*m3*cos(2*t2)*t1dd)/2 + (a3*a4*m3*cos(2*t2 + t3)*t1dd)/2 - (457*a2*m2*sin(t2 + 5030556812569863/36028797018963968)*t1d*t2d)/1250 - (a4^2*m3*sin(2*t2 + 2*t3)*t1d*t2d)/4 - (a4^2*m3*sin(2*t2 + 2*t3)*t1d*t3d)/4 + 2*a2*a3*m3*cos(t2)*t1dd + (a3*a4*m3*cos(t3)*t1dd)/2 + a2*a4*m3*cos(t2 + t3)*t1dd - a3^2*m3*sin(2*t2)*t1d*t2d - a3*a4*m3*sin(2*t2 + t3)*t1d*t2d - (a3*a4*m3*sin(2*t2 + t3)*t1d*t3d)/2 - 2*a2*a3*m3*sin(t2)*t1d*t2d - (a3*a4*m3*sin(t3)*t1d*t3d)/2 - a2*a4*m3*sin(t2 + t3)*t1d*t2d - a2*a4*m3*sin(t2 + t3)*t1d*t3d;
        T2 = (448317*m2*cos(t2))/250000 + (208849*m2*t2dd)/6250000 + a3^2*m3*t2dd + (a4^2*m3*t2dd)/4 + (a4^2*m3*t3dd)/4 + (981*a3*m3*cos(t2))/100 + (981*a4*m3*cos(t2 + t3))/200 + (208849*m2*sin(2*t2 + 5030556812569863/18014398509481984)*t1d^2)/12500000 + (a4^2*m3*sin(2*t2 + 2*t3)*t1d^2)/8 + (a3^2*m3*sin(2*t2)*t1d^2)/2 + (457*a2*m2*sin(t2 + 5030556812569863/36028797018963968)*t1d^2)/2500 + a2*a3*m3*sin(t2)*t1d^2 + (a2*a4*m3*sin(t2 + t3)*t1d^2)/2 + a3*a4*m3*cos(t3)*t2dd + (a3*a4*m3*cos(t3)*t3dd)/2 + (a3*a4*m3*sin(2*t2 + t3)*t1d^2)/2 - a3*a4*m3*sin(t3)*t2d*t3d - (a3*a4*m3*sin(t3)*t3d*t3d)/2;
        T3 = (a4^2*m3*t2dd)/4 + (a4^2*m3*t3dd)/4 + (981*a4*m3*cos(t2 + t3))/200 + (a4^2*m3*sin(2*t2 + 2*t3)*t1d^2)/8 + (a3*a4*m3*sin(t3)*t1d^2)/4 + (a3*a4*m3*sin(t3)*t2d^2)/2 + (a2*a4*m3*sin(t2 + t3)*t1d^2)/2 + (a3*a4*m3*cos(t3)*t2dd)/2 + (a3*a4*m3*sin(2*t2 + t3)*t1d^2)/4 - (a3*a4*m3*sin(t3)*t2d*t3d)/2 + (a3*a4*m3*sin(t3)*t2d*t3d)/2;
        
        Torques(1, i) = T1;
        Torques(2, i) = T2;
        Torques(3, i) = T3;
        
    end
    
    figure;
    
    hold on
    
    title("Joint Torque using Dynamic Model without the load");
    
    plot(Torques(1, :), 'b.-');
    plot(Torques(2, :), 'r.-');
    plot(Torques(3, :), 'g.-');
    
    legend("TORQUE JOINT 1", "TORQUE JOINT 2", "TORQUE JOINT 3");
    xlabel("CIRCLE SAMPLE");
    ylabel("TORQUE [Nm]");
    
    hold off;
    
end

end