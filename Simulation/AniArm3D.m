function AniArm3D()
    %% Show torques if wanted.
    SHOW_TORQUES = 1;

    %% Thetas at initial position.
    T = [pi/2, pi/4, 0, 0, pi/4];
    L = [-0.2, -0.037, -0.4, -0.4, -0.1];
    
    %% Get the circle trajectory.
    CIRC_TRAJ = CALC_CIRCLE_TRAJ(T, L);
    
    %% Get the predicted circle trajectory.
    THETA = IK_NEWTON_RAPHSON_CIRCLE_TRAJ(T, L);
    
    %% Animate the arm.
    range = L(1) + L(2) + L(3) + L(4) + L(5);
    
    figure;
    
    for i = 1:length(THETA)
        clf;
        hold on;
        [HM, HMPos] = GetRRRRRRHM(THETA(:, i), L);
        
        plot3([range -range], [0 0], [0 0], 'r-');
        plot3([0 0], [range -range], [0 0], 'r-');
        p0 = plot3([HMPos.PBASE(1) HMPos.P1(1)], [HMPos.PBASE(2) HMPos.P1(2)], [HMPos.PBASE(3) HMPos.P1(3)], 'k.-');
        p1 = plot3([HMPos.P1(1) HMPos.P2(1)], [HMPos.P1(2) HMPos.P2(2)], [HMPos.P1(3) HMPos.P2(3)], 'b.-');
        p2 = plot3([HMPos.P2(1) HMPos.P3(1)], [HMPos.P2(2) HMPos.P3(2)], [HMPos.P2(3) HMPos.P3(3)], 'm.-');
        p3 = plot3([HMPos.P3(1) HMPos.P4(1)], [HMPos.P3(2) HMPos.P4(2)], [HMPos.P3(3) HMPos.P4(3)], 'g.-');
        p4 = plot3([HMPos.P4(1) HMPos.P5(1)], [HMPos.P4(2) HMPos.P5(2)], [HMPos.P4(3) HMPos.P5(3)], 'r.-');
    
        plot3(CIRC_TRAJ(1, :), CIRC_TRAJ(2, :), CIRC_TRAJ(3, :), 'b.');
    
        p0.MarkerSize = 20;
        p1.MarkerSize = 20;
        p2.MarkerSize = 20;
        p3.MarkerSize = 20;
        p4.MarkerSize = 20;
   
        p0.LineWidth = 3;
        p1.LineWidth = 3;
        p2.LineWidth = 3;
        p3.LineWidth = 3;
        p4.LineWidth = 3;   
    
        xlabel('x[m]');
        ylabel('y[m]');
        zlabel('z[m]');
        title("Animation of the Arm");
    
        axis square;
        axis equal;
        
        view(35, 35);
        
        pause(0.001);
    end
    hold off;

    %% Showing the torques of the arm.
    if (SHOW_TORQUES == 1)
        ANGULAR_TORQUES(T, L);
    end
end

