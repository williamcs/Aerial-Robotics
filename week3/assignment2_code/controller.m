function [ F, M ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

F = 0;
M = 0;

% FILL IN YOUR CODE HERE
K_dz = 13; %2;
K_pz = 400;%30;

F = params.mass*(params.gravity + des_state.acc(2)+ K_dz*(des_state.vel(2)-state.vel(2)) + K_pz*(des_state.pos(2)-state.pos(2)));

K_dy = 6; %0.78; %0.35;
K_py = 6;%10.4; %4;

Phic = -1/params.gravity*(des_state.acc(1) + K_dy*(des_state.vel(1) - state.vel(1)) + K_py*(des_state.pos(1)-state.pos(1)));

K_dphi = 26.6; %20;
K_pphi = 1600;

M = params.Ixx*(K_dphi*(-state.omega) + K_pphi*(Phic - state.rot));
end

