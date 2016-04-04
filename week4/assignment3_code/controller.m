function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yaw_dot  --des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% =================== Your code goes here ===================

% Thurst
F = 0;
%F = params.mass*(params.gravity + des_state.acc(3));
%F = params.mass*des_state.acc(3);
kpz = 400;
kdz = 13;
%F = params.mass*(params.gravity + (kdz*des_state.vel(3) + kpz*(des_state.pos(3)-state.pos(3))));
%F = params.mass*(params.gravity - kdz*state.vel(3) + kpz*(des_state.pos(3)-state.pos(3)));

r3_acc_des = des_state.acc(3) + kdz*(des_state.vel(3) - state.vel(3)) + kpz*(des_state.pos(3) - state.pos(3));
F = params.mass*(params.gravity + r3_acc_des);

% Moment
M = zeros(3,1);

kpx = 6;
kdx = 6;
r1_acc_des = des_state.acc(1) + kdx*(des_state.vel(1) - state.vel(1)) + kpx*(des_state.pos(1) - state.pos(1));

kpy = 6;
kdy = 6;
r2_acc_des = des_state.acc(2) + kdy*(des_state.vel(2) - state.vel(2)) + kpy*(des_state.pos(2) - state.pos(2));

%phi_des = 1/params.gravity*(des_state.acc(1)*sin(state.rot(3)) - des_state.acc(2)*cos(state.rot(3)));
%theta_des = 1/params.gravity*(des_state.acc(1)*cos(state.rot(3)) + des_state.acc(2)*sin(state.rot(3)));

phi_des = 1/params.gravity*(r1_acc_des*sin(des_state.yaw) - r2_acc_des*cos(des_state.yaw));
theta_des = 1/params.gravity*(r1_acc_des*cos(des_state.yaw) + r2_acc_des*sin(des_state.yaw));

kpphi = 0.0203; %0.0203; %0.9; %2;%2;%1;
kdphi = 0.004; %0.05; %2; %5; %3;%4;
u_phi = kpphi*(phi_des -state.rot(1)) + kdphi*(0-state.omega(1));

kptheta = 0.0203; %0.0203; %0.9; %2; %2;%1;
kdtheta = 0.004; %0.05; %2; %5; %3;%4;
u_theta = kptheta*(theta_des-state.rot(2)) + kdtheta*(0-state.omega(2));

kppsi = 100;%13; %15;
kdpsi = 2; %2;
%u_psi = kppsi*(0) + kdpsi*(des_state.yawdot-state.omega(3));
u_psi = kppsi*(des_state.yaw-state.rot(3)) + kdpsi*(des_state.yawdot-state.omega(3));

M = [u_phi; u_theta; u_psi];


% =================== Your code ends here ===================

end
