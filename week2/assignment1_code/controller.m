function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

u = 0;
%u = params.gravity; %params.mass + params.gravity;

% FILL IN YOUR CODE HERE
%error = s_des - s;
%e = error(1);
%edot = error(2);

E = s_des - s;
e = E(1, 1);
edot = E(2, 1);
Kv = 11;
Kp = 100;

u = params.mass*(Kp*e + Kv*edot + params.gravity);

end

