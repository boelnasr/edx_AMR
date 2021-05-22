% clear all; clc;
% 
% syms alpha beta gamma real
% 
% % write down the rotation matrices using the symbolic parameters alpha, beta, gamma
R_B1 = [...
    1,          0,          0;
    0,          cos(alpha), -sin(alpha);
    0,          sin(alpha), cos(alpha)]
% 
% R_12 = [...
%     cos(beta),  0,          sin(beta);
%     0,          1,          0;
%     -sin(beta), 0,          cos(beta)]
% 
% R_23 = [...
%     cos(gamma), 0,          sin(gamma);
%     0,          1,          0;
%     -sin(gamma),0,          cos(gamma)]
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % rotational matrices calculated in previous problem set
% R_B1 = [1,0,0;0,cos(alpha),-sin(alpha);0,sin(alpha),cos(alpha)];
% R_12 = [cos(beta),0,sin(beta);0,1,0;-sin(beta),0,cos(beta)];
% R_23 = [cos(gamma),0,sin(gamma);0,1,0;-sin(gamma),0,cos(gamma)];
% 
% % write down the 3x1 relative position vectors for link length l_i=1
% r_B1_inB = [0, 1, 0]';
% r_12_in1 = [0, 0, -1]';
% r_23_in2 = [0, 0, -1]';
% r_3F_in3 = [0, 0, -1]';
% 
% % write down the homogeneous transformation matrices
% H_B1 = [...
%     R_B1,       r_B1_inB;
%     0, 0, 0,    1]
% 
% H_12 = [...
%     R_12,       r_12_in1;
%     0, 0, 0,    1]
% 
% H_23 = [...
%     R_23,       r_23_in2;
%     0, 0, 0,    1];
% 
% % create the cumulative transformation matrix
% H_B3 = H_B1 * H_12 * H_23;
% 
% % find the foot point position vector
% r_BF_inB = H_B3 * [r_3F_in3; 1];
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% q = [alpha;beta;gamma];
% 
% r_BF_inB = [...
%     - sin(beta + gamma) - sin(beta);...
%   sin(alpha)*(cos(beta + gamma) + cos(beta) + 1) + 1;...
%   -cos(alpha)*(cos(beta + gamma) + cos(beta) + 1)];
% 
% 
% % determine the foot point Jacobian J_BF_inB=d(r_BF_inB)/dq
% J_BF_inB =  jacobian(r_BF_inB, q)
% 
% % what generalized velocity dq do you have to apply in a configuration q = [0;60�;-120�]
% % to lift the foot in vertical direction with v = [0;0;-1m/s];
% 
% dr_value    = [0, 0, -1]';
% q_value     = deg2rad([0, 60, -120]'); 
% 
% J_value = eval(subs(J_BF_inB, q, q_value));
% 
% % dq = J \ dr
% dq = J_value \ dr_value;     
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % given are the functions 
% %   r_BF_inB(alpha,beta,gamma) and
% %   J_BF_inB(alpha,beta,gamma) 
% % for the foot positon respectively Jacobian
% 
% r_BF_inB = @(alpha,beta,gamma)[...
%     -sin(beta + gamma) - sin(beta);...
%   sin(alpha)*(cos(beta + gamma) + cos(beta) + 1) + 1;...
%   -cos(alpha)*(cos(beta + gamma) + cos(beta) + 1)];
%  
% J_BF_inB = @(alpha,beta,gamma)[...
%                                               0,             - cos(beta + gamma) - cos(beta),            -cos(beta + gamma);...
%  cos(alpha)*(cos(beta + gamma) + cos(beta) + 1), -sin(alpha)*(sin(beta + gamma) + sin(beta)), -sin(beta + gamma)*sin(alpha);...
%  sin(alpha)*(cos(beta + gamma) + cos(beta) + 1),  cos(alpha)*(sin(beta + gamma) + sin(beta)),  sin(beta + gamma)*cos(alpha)];
%  
% % write an algorithm for the inverse kinematics problem to
% % find the generalized coordinates q that gives the endeffector position rGoal =
% % [0.2,0.5,-2]' and store it in qGoal
% q0 = pi/180*([0,-30,60])';
% rGoal = [0.2,0.5,-2]';
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%% enter here your algorithm
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% e=1e-8
% q=q0
% i=0
% r = r_BF_inB(q(1), q(2), q(3));
% while norm(rGoal-r)>e
%     q = q + pinv(J_BF_inB(q(1), q(2), q(3))) * (rGoal - r);
%     r = r_BF_inB(q(1), q(2), q(3));
%     
%     i = i + 1;
%     disp(i)
%     
% end
% qGoal = q 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% clear all; clc;
% 
% % given are the functions 
% %   r_BF_inB(alpha,beta,gamma) and
% %   J_BF_inB(alpha,beta,gamma) 
% % for the foot positon respectively Jacobian
% 
% r_BF_inB = @(alpha,beta,gamma)[...
%     -sin(beta + gamma) - sin(beta);...
%   sin(alpha)*(cos(beta + gamma) + cos(beta) + 1) + 1;...
%   -cos(alpha)*(cos(beta + gamma) + cos(beta) + 1)];
%  
% J_BF_inB = @(alpha,beta,gamma)[...
%                                               0,             - cos(beta + gamma) - cos(beta),            -cos(beta + gamma);...
%  cos(alpha)*(cos(beta + gamma) + cos(beta) + 1), -sin(alpha)*(sin(beta + gamma) + sin(beta)), -sin(beta + gamma)*sin(alpha);...
%  sin(alpha)*(cos(beta + gamma) + cos(beta) + 1),  cos(alpha)*(sin(beta + gamma) + sin(beta)),  sin(beta + gamma)*cos(alpha)];
%  
% % write an algorithm for the inverse kinematics problem to
% % find the generalized coordinates q that gives the endeffector position rGoal =
% % [-1.5,1,-2.5]' and store it in qGoal
% q0 = pi/180*([0,-30,60])';
% rGoal = [-1.5,1,-2.5]';
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %%% enter here your algorithm
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% tolerance = 1e-8;
% 
% % cnt
% cnt = 0;
% 
% % initial position
% q = q0;
% r = r_BF_inB(q(1), q(2), q(3));
% rPrev = rGoal;
% 
% % 'Jacobi-transposed' inverse kinematics iteration 
% %
% % Note: this algorithm is for the goal position is in the singularity case
% while norm(rPrev - r) > tolerance 
%     
%     % until converges...
%     k = 0.1;
%     
%     % previous r 
%     rPrev = r;
%     
%     q = q + k * J_BF_inB(q(1), q(2), q(3))' * (rGoal - r);
%     r = r_BF_inB(q(1), q(2), q(3));
%     
%     cnt = cnt + 1;
%     disp(cnt)
%     disp(r)
% end
% 
% disp('iter terminated with q (deg): ')
% disp(rad2deg(q))
% 
% qGoal = q;        
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all; clc;

% given are the functions 
%   r_BF_inB(alpha,beta,gamma) and
%   J_BF_inB(alpha,beta,gamma) 
% for the foot positon respectively Jacobian
  
r_BF_inB = @(alpha,beta,gamma)[...
    - sin(beta + gamma) - sin(beta);...
  sin(alpha)*(cos(beta + gamma) + cos(beta) + 1) + 1;...
  -cos(alpha)*(cos(beta + gamma) + cos(beta) + 1)];

J_BF_inB = @(alpha,beta,gamma)[...
                                              0,             - cos(beta + gamma) - cos(beta),            -cos(beta + gamma);...
 cos(alpha)*(cos(beta + gamma) + cos(beta) + 1), -sin(alpha)*(sin(beta + gamma) + sin(beta)), -sin(beta + gamma)*sin(alpha);...
 sin(alpha)*(cos(beta + gamma) + cos(beta) + 1),  cos(alpha)*(sin(beta + gamma) + sin(beta)),  sin(beta + gamma)*cos(alpha)];

% write an algorithm for the inverse differential kinematics problem to
% find the generalized velocities dq to follow a circle in the body xz plane
% around the start point rCenter with a radius of r=0.5 and a 
% frequeny of 0.25Hz. The start configuration is q =  pi/180*([0,-60,120])',
% which defines the center of the circle
q0 = pi/180*([0,-60,120])';
dq0 = zeros(3,1);
rCenter = r_BF_inB(q0(1),q0(2),q0(3));
radius = 0.5;
f = 0.25;
rGoal = @(t) rCenter + radius*[sin(2*pi*f*t),0,cos(2*pi*f*t)]';
drGoal = @(t) 2*pi*f*radius*[cos(2*pi*f*t),0,-sin(2*pi*f*t)]';

% define here the time resolution
deltaT = 0.01;
timeArr = 0:deltaT:1/f;

% q, r, and rGoal are stored for every point in time in the following arrays
qArr = zeros(3,length(timeArr));
rArr = zeros(3,length(timeArr));
rGoalArr = zeros(3,length(timeArr));

q = q0;
dq = dq0;
for i=1:length(timeArr)
    t = timeArr(i);
    % data logging, don't change this!
    q = q+deltaT*dq;
    qArr(:,i) = q;
    rArr(:,i) = r_BF_inB(q(1),q(2),q(3));
    rGoalArr(:,i) = rGoal(t);
    
    % controller: 
    % step 1: create a simple p controller to determine the desired foot
    % point velocity
    K = -10;
    v = drGoal(t) - K * (rGoal(t) - r_BF_inB(q(1),q(2),q(3)));
    % step 2: perform inverse differential kinematics to calculate the
    % generalized velocities
    dq = pinv(J_BF_inB(q(1),q(2),q(3))) * v;
    
end
     
% plotting
plot(rArr(1, 1), rArr(3, 1), 'kx')
hold on
plot(rGoalArr(1, :), rGoalArr(3, :), 'r-')
plot(rArr(1, :), rArr(3, :), 'b-')
hold off
xlabel('x')
ylabel('z')
legend('start', 'rGoal', 'r')

