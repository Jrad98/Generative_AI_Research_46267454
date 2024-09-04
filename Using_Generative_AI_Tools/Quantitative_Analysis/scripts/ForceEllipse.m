% METR4202 Jared Tucker
% Q4a

% Function for visualing a force elipse on a 2R robot

%Function Inputs
% L1 = arm 1 length
% L2 = arm 2 length
% theta_1_deg = joint angle in degrees of joint 1
% theta_2_deg = joint angle in degrees of joint 2

%Function Outputs
% Outputs are manipulability measure floats
% u1 = Ratio of Longest and Shortest semi-axes
% u2 = Condition Number
% u3 = Value Proportional to Volume

%Function Outputs
function [u1,u2,u3] = ForceEllipse(L1,L2,theta_1_deg,theta_2_deg)
    %convert from deg to rad
    theta_1 = theta_1_deg*pi/180;
    theta_2 = theta_2_deg*pi/180;
    
    %to not break theta make sure that if input angles are 0, change to
    %something close to zero
    if theta_1 ==0 && theta_2 == 0
        theta_1 = 0.0001
        theta_2 = 0.0001
    end

    % factor of graph axis scaling
    axis_length_scale = (L1+L2)*1.5;
    ellipse_scale = 0.9*norm(L1,L2)
    
    %forward kinematic model
    %initial coords
    pos_0 = [0,0];
    % new link 1 coords
    pos_1 = [L1*cos(theta_1),L1*sin(theta_1)];
    
    % new link 2 coords
    %pos_2 = [(pos_1(1) + L2*cos(theta_2)),(pos_1(2) + L2*sin(theta_2))]
    pos_2 = [(L1*cos(theta_1) + L2*cos(theta_1+theta_2)),(L1*sin(theta_1) + L2*sin(theta_1+theta_2))];
    
    %calcualte force elipse
    %J(theta) = [w;v]
    J = [(-L1*sin(theta_1)-L2*sin(theta_1+theta_2)) (-L2*sin(theta_1+theta_2));
         ( L1*cos(theta_1)+L2*cos(theta_1+theta_2)) ( L2*cos(theta_1+theta_2))];
    % J transpose
    J_T = transpose(J);
    
    %A for force ellipsoid
    A = inv((J*J_T));
    
    % eigenvalues and eigen vectors of A
    [e_vect,e_vals] = eig(A);
    lambda_1 = e_vals(1,1);
    lambda_2 = e_vals(2,2);
    %eigenvectors and their directions
    vect_lambda_1 = e_vect(1:2,1);
    major_axis_direction = atan2(vect_lambda_1(2),vect_lambda_1(1));
    vect_lambda_2 = e_vect(1:2,2);
    minor_axis_direction = atan2(vect_lambda_2(2),vect_lambda_2(1));

    % Force Ellipsoid Measures
    % Ratio of Longest and Shortest semi-axes
    u1 = sqrt(max(lambda_1,lambda_2)) / sqrt(min(lambda_1,lambda_2));

    % Condition Number
    u2 = max(lambda_1,lambda_2) / min(lambda_1,lambda_2);
    
    % Value Proportional to Volume
    u3 = sqrt(det(A));

    %length of axis 1
    axis_1 = sqrt(abs(lambda_1));
    %length of axis 2
    axis_2 = sqrt(abs(lambda_2));
    % determine magnitude of axis as a vector
    length = norm([axis_1,axis_2]);
    %unit length axis
    axis_1_unit = axis_1/length;
    axis_2_unit = axis_2/length;
    
    %plot ellipse in parameterised form
    a = axis_1_unit*ellipse_scale;
    b = axis_2_unit*ellipse_scale;
    x0=pos_2(1); % x0,y0 ellipse centre coordinates
    y0=pos_2(2);
    t=-pi:0.01:pi;
    x=x0 + a*cos(t)*cos(major_axis_direction) - b*sin(t)*sin(major_axis_direction);
    y=y0 + a*cos(t)*sin(major_axis_direction) + b*sin(t)*cos(major_axis_direction);
    plot(x,y)
    axis equal
    hold on
    grid on
    %set axis scale
    xlim([-axis_length_scale,axis_length_scale])
    ylim([-axis_length_scale,axis_length_scale])
    %plot lines
    plot([pos_0(1) pos_1(1)], [pos_0(2) pos_1(2)],'linewidth', 1.5, 'color' ,'r')
    plot([pos_1(1) pos_2(1)], [pos_1(2) pos_2(2)],'linewidth', 1.5, 'color' ,'b')
    %plot joints
    plot(pos_0(1),pos_0(2),'o')
    plot(pos_1(1),pos_1(2),'o')
    plot(pos_2(1),pos_2(2),'o')
    