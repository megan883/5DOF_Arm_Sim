function REC_THETA = IK_NEWTON_RAPHSON_CIRCLE_TRAJ(START_T, L)
    %% Displaying relevant graphs when needed.
    SHOW_PREDICTED_THETAS_AND_ERROR = 0;
    SHOW_PREDICTION_VS_ORIGINAL = 0;
    %% Calculating circle trajectory.
    GROUND_TRUTH_CIRCLE_TRAJ = CALC_CIRCLE_TRAJ(START_T, L);
    %% Setting error threshold and initial parameters.
    ERR_THRESHOLD = 0.001;
    PREV_T = START_T';
    START_IDX = 1;
    
    %% Calculating the inverse kinematics with Newton Raphson method.
    for i = 1:length(GROUND_TRUTH_CIRCLE_TRAJ(1, :))
        NOW_T = PREV_T;
        LOOP = 0;
        IDX = START_IDX;
        while (LOOP == 0)
            % Getting predicted homogeneous matrix from predicted thetas.
            [HM, HMPos] = GetRRRRRRHM(NOW_T, L);
            % Getting error.
            ERR = GROUND_TRUTH_CIRCLE_TRAJ(:, i) - [HMPos.P5(1); HMPos.P5(2); HMPos.P5(3)];
            % Getting the Jacobian Psuedo-Inverse.
            [J_PINV, J] = GETTING_JACOBIAN(NOW_T, L);
            % Calculating next theta.
            DELTA_THETA = J_PINV * ERR;
            % Integrating using the Euler integration method.
            NOW_T = NOW_T + DELTA_THETA;
            % Calculate the mean error.
            MEAN_ERR = (ERR(1) + ERR(2) + ERR(3)) / 3;
            
            % IF ERROR IS LOWER THAN 0, MAKE IT 0.
            if (MEAN_ERR < 0)
                MEAN_ERR_COL(IDX) = 0;
            else
                MEAN_ERR_COL(IDX) = MEAN_ERR;
            end
            % Iteration number
            IDX = IDX + 1;

            % IF ERROR IS LOWER THAN THRESHOLD.
            if (norm(ERR) < ERR_THRESHOLD)
                LOOP = 1;
            end
        end
        %% Collect error and theta data.
        MEAN_ERROR_COL(:, i) = MEAN_ERR_COL;
        % Set the previous theta angle to be the current theta.
        PREV_T = NOW_T;
        
        REC_THETA(:, i) = NOW_T;
    end
    %% Graphing the errors and theta angle on graphs.
    if (SHOW_PREDICTED_THETAS_AND_ERROR == 1)
        figure;
    
        hold on
        title("NEWTON RAPHSON PREDICTED THETAS");
        plot(1:length(REC_THETA(1, :)), REC_THETA(1, :), 'b.-');
        plot(1:length(REC_THETA(2, :)), REC_THETA(2, :), 'r.-');
        plot(1:length(REC_THETA(3, :)), REC_THETA(3, :), 'g.-');
        plot(1:length(REC_THETA(4, :)), REC_THETA(4, :), 'k.-');
        plot(1:length(REC_THETA(5, :)), REC_THETA(5, :), 'm.-');

        xlabel("Circle Point");
        ylabel("Theta Angle");
        legend("Theta Joint 1", "Theta Joint 2", "Theta Joint 3", "Theta Joint 4", "Theta Joint 5");
        hold off;

        figure;
   
        hold on
        title("NEWTON RAPHSON ERROR");

        for J = 1:length(GROUND_TRUTH_CIRCLE_TRAJ(1, :))
            plot(1:length(MEAN_ERROR_COL(:, J)), MEAN_ERROR_COL(:, J), 'b.-');
        end

        xlabel("Iterations");
        ylabel("Error");
        hold off;
    end
    %% Showing the estimated and original circular trajectories.
    if (SHOW_PREDICTION_VS_ORIGINAL == 1)
        figure;
    
        hold on;
    
        for i = 1:length(GROUND_TRUTH_CIRCLE_TRAJ(1, :))
            [HM, HMPos] = GetRRRRRRHM(REC_THETA(:, i), L);
        
            plot3(HMPos.P5(1), HMPos.P5(2), HMPos.P5(3), 'm.');
            plot3(GROUND_TRUTH_CIRCLE_TRAJ(1, i), GROUND_TRUTH_CIRCLE_TRAJ(2, i), GROUND_TRUTH_CIRCLE_TRAJ(3, i), 'b.');
        end
    
        axis equal;
        axis square;
        xlabel('x[m]');
        ylabel('y[m]');
        zlabel('z[m]');
        legend("Predicted Trajectory", "Original Trajectory");
        title("ORIGINAL AND PREDICTED CIRCLE TRAJECTORY");
        view(40, 40);
        hold off;
    end
end

