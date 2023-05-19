function [JP_INV, J] = GETTING_JACOBIAN(T, L)
    % Changing values for simplicity.
    T1 = T(1);
    T2 = T(2);
    T3 = T(3);
    T4 = T(4);
    T5 = T(5);
    L1 = L(1);
    L2 = L(2);
    L3 = L(3);
    L4 = L(4);
    L5 = L(5);
    
    % 5-joint jacobian (REDUCES TIME CALCULATING TRAJECTORY).
    J = [L4*(cos(T2)*sin(T1)*sin(T3) + cos(T3)*sin(T1)*sin(T2)) - L2*sin(T1) + L5*sin(T5)*(cos(T2)*sin(T1)*sin(T3) + cos(T3)*sin(T1)*sin(T2)) - L3*cos(T2)*sin(T1) - L5*cos(T5)*(cos(T4)*(cos(T2)*cos(T3)*sin(T1) - sin(T1)*sin(T2)*sin(T3)) + cos(T1)*sin(T4)), - L4*(cos(T1)*cos(T2)*cos(T3) - cos(T1)*sin(T2)*sin(T3)) - L3*cos(T1)*sin(T2) - L5*sin(T5)*(cos(T1)*cos(T2)*cos(T3) - cos(T1)*sin(T2)*sin(T3)) - L5*cos(T4)*cos(T5)*(cos(T1)*cos(T2)*sin(T3) + cos(T1)*cos(T3)*sin(T2)), - L4*(cos(T1)*cos(T2)*cos(T3) - cos(T1)*sin(T2)*sin(T3)) - L5*sin(T5)*(cos(T1)*cos(T2)*cos(T3) - cos(T1)*sin(T2)*sin(T3)) - L5*cos(T4)*cos(T5)*(cos(T1)*cos(T2)*sin(T3) + cos(T1)*cos(T3)*sin(T2)), -L5*cos(T5)*(sin(T4)*(cos(T1)*cos(T2)*cos(T3) - cos(T1)*sin(T2)*sin(T3)) + cos(T4)*sin(T1)), - L5*sin(T5)*(cos(T4)*(cos(T1)*cos(T2)*cos(T3) - cos(T1)*sin(T2)*sin(T3)) - sin(T1)*sin(T4)) - L5*cos(T5)*(cos(T1)*cos(T2)*sin(T3) + cos(T1)*cos(T3)*sin(T2));
        L2*cos(T1) - L4*(cos(T1)*cos(T2)*sin(T3) + cos(T1)*cos(T3)*sin(T2)) + L3*cos(T1)*cos(T2) + L5*cos(T5)*(cos(T4)*(cos(T1)*cos(T2)*cos(T3) - cos(T1)*sin(T2)*sin(T3)) - sin(T1)*sin(T4)) - L5*sin(T5)*(cos(T1)*cos(T2)*sin(T3) + cos(T1)*cos(T3)*sin(T2)), - L4*(cos(T2)*cos(T3)*sin(T1) - sin(T1)*sin(T2)*sin(T3)) - L5*sin(T5)*(cos(T2)*cos(T3)*sin(T1) - sin(T1)*sin(T2)*sin(T3)) - L3*sin(T1)*sin(T2) - L5*cos(T4)*cos(T5)*(cos(T2)*sin(T1)*sin(T3) + cos(T3)*sin(T1)*sin(T2)), - L4*(cos(T2)*cos(T3)*sin(T1) - sin(T1)*sin(T2)*sin(T3)) - L5*sin(T5)*(cos(T2)*cos(T3)*sin(T1) - sin(T1)*sin(T2)*sin(T3)) - L5*cos(T4)*cos(T5)*(cos(T2)*sin(T1)*sin(T3) + cos(T3)*sin(T1)*sin(T2)),  L5*cos(T5)*(cos(T1)*cos(T4) - sin(T4)*(cos(T2)*cos(T3)*sin(T1) - sin(T1)*sin(T2)*sin(T3))), - L5*cos(T5)*(cos(T2)*sin(T1)*sin(T3) + cos(T3)*sin(T1)*sin(T2)) - L5*sin(T5)*(cos(T4)*(cos(T2)*cos(T3)*sin(T1) - sin(T1)*sin(T2)*sin(T3)) + cos(T1)*sin(T4));
        0, L3*cos(T2) - L4*(cos(T2)*sin(T3) + cos(T3)*sin(T2)) - L5*sin(T5)*(cos(T2)*sin(T3) + cos(T3)*sin(T2)) + L5*cos(T4)*cos(T5)*(cos(T2)*cos(T3) - sin(T2)*sin(T3)), L5*cos(T4)*cos(T5)*(cos(T2)*cos(T3) - sin(T2)*sin(T3)) - L5*sin(T5)*(cos(T2)*sin(T3) + cos(T3)*sin(T2)) - L4*(cos(T2)*sin(T3) + cos(T3)*sin(T2)), -L5*cos(T5)*sin(T4)*(cos(T2)*sin(T3) + cos(T3)*sin(T2)), L5*cos(T5)*(cos(T2)*cos(T3) - sin(T2)*sin(T3)) - L5*cos(T4)*sin(T5)*(cos(T2)*sin(T3) + cos(T3)*sin(T2))];
    
    % puesdo-inverse.
    JP_INV = pinv(J);
end

