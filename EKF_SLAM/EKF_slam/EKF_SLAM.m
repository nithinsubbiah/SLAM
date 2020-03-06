%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  16833 Robot Localization and Mapping  % 
%  Assignment #2                         %
%  EKF-SLAM                              %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;
close all;
clc;

%==== TEST: Setup uncertainity parameters (try different values!) ===
sig_x = 0.25;
sig_y = 0.1;
sig_alpha = 0.1;
sig_beta = 0.01;
sig_r = 0.08;

%==== Generate sigma^2 from sigma ===
sig_x2 = sig_x^2;
sig_y2 = sig_y^2;
sig_alpha2 = sig_alpha^2;
sig_beta2 = sig_beta^2;
sig_r2 = sig_r^2;

%==== Open data file ====
fid = fopen('../data/data.txt');

%==== Read first measurement data ====
tline = fgets(fid);
arr = str2num(tline);
measure = arr';
t = 1;
 
%==== Setup control and measurement covariances ===
control_cov = diag([sig_x2, sig_y2, sig_alpha2]);
measure_cov = diag([sig_beta2, sig_r2]);

%==== Setup initial pose vector and pose uncertainty ====
pose = [0 ; 0 ; 0];
pose_cov = diag([0.02^2, 0.02^2, 0.1^2]);

%==== TODO: Setup initial landmark vector landmark[] and covariance matrix landmark_cov[] ====
%==== (Hint: use initial pose with uncertainty and first measurement) ====

% Write your code here...
landmark = zeros(length(measure),1);
k = length(measure)/2;

for i = 1:2:k
    landmark(i) = pose(1)+measure(i+1)*cos(pose(3)+measure(i));
    landmark(i+1) = pose(2)+measure(i+1)*sin(pose(3)+measure(i));
end

%TODO: Setup landmark covariance
landmark_cov = 0.1*eye(2*k, 2*k);

%==== Setup state vector x with pose and landmark vector ====
x = [pose ; landmark];

%==== Setup covariance matrix P with pose and landmark covariances ====
P = [pose_cov zeros(3, 2*k) ; zeros(2*k, 3) landmark_cov];

%==== Plot initial state and conariance ====
last_x = x;
drawTrajAndMap(x, last_x, P, 0);

%==== Read control data ====
tline = fgets(fid);
while ischar(tline)
    arr = str2num(tline);
    d = arr(1);
    alpha = arr(2);
    
    %==== TODO: Predict Step ====
    %==== (Notice: predict state x_pre[] and covariance P_pre[] using input control data and control_cov[]) ====
    
    % Write your code here...
    F = [eye(3) zeros(3,2*k)];
    g = [d*cos(last_x(3));d*sin(last_x(3));last_x(3)+alpha];
    
    x_pre = last_x + transpose(F)*g;
    
    G_intermediate = [1 0 -d*sin(last_x(3));0 1 d*cos(last_x(3));0 0 1];
    R_robot_world = [cos(last_x(3)) sin(last_x(3)) 0;-sin(last_x(3)) cos(last_x(3)) 0;0 0 1];
    G = eye(15) + transpose(F)* G_intermediate *F;
    P_pre = G*P*transpose(G)+transpose(F)*(R_robot_world*control_cov)*F;
    
    %==== Draw predicted state x_pre[] and covariance P_pre[] ====
    drawTrajPre(x_pre, P_pre);
    
    %==== Read measurement data ====
    tline = fgets(fid);
    arr = str2num(tline);
    measure = arr';
    
    %==== TODO: Update Step ====
    %==== (Notice: update state x[] and covariance P[] using input measurement data and measure_cov[]) ====
    
    for j = 1:2:k
        
        lx = x_pre(1)+measure(j+1)*cos(x_pre(3)+measure(j));
        ly = x_pre(2)+measure(j+1)*sin(x_pre(3)+measure(j));
        
        delta = [lx-x_pre(1);ly-x_pre(2)];
        q = transpose(delta)*delta;
        z_pred = [atan2(delta(2),delta(1));sqrt(q)];
        
        F = [eye(3) zeros(3,length(measure));zeros(2,3+length(measure))];
        F(4,j+3) = 1;
        F(5,j+4) = 1;
        Q_matrix = [delta(2) -delta(1) -q -delta(2) delta(1);
                    -sqrt(q)*delta(1) -sqrt(q)*delta(2) 0 sqrt(q)*delta(1) sqrt(q)*delta(2)];
        H = (1/q)*(Q_matrix*F);
        K = (P_pre*transpose(H))*inv(H*P_pre*transpose(H)+measure_cov);
        
        x_pre = x_pre + K*([measure(j);measure(j+1)]-z_pred);
        
        P_pre = (eye(15)-K*H)*P_pre; 
        
    end
    
    x = x_pre;
    P = P_pre;
    
    %==== Plot ====   
    drawTrajAndMap(x, last_x, P, t);
    last_x = x;
    
    %==== Iteration & read next control data ===
    t = t + 1;
    tline = fgets(fid);
end


%==== EVAL: Plot ground truth landmarks ====

% Write your code here...
    

%==== Close data file ====
fclose(fid);
