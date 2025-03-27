function [t, p, v, a] = trajectory_generator(p0, pf, v_max, a_max, dt, end_together)
% Inputs:
%   p0 - vector of initial positions for each joint [n×1]
%   pf - vector of final positions for each joint [n×1]
%   v_max - vector of maximum velocity constraints for each joint [n×1]
%   a_max - vector of maximum acceleration constraints for each joint [n×1]
%   dt - time step for trajectory discretization
%   end_together - boolean flag to indicate if all joints should finish at the same time
%
% Outputs:
%   t - time vector
%   p - position trajectories [time_steps×n]
%   v - velocity trajectories [time_steps×n]
%   a - acceleration trajectories [time_steps×n]
%
% This function generates the trajectory for each joint, with an option
% to either make all joints end at the same time or not.

% Get number of joints
num_joints = length(p0);
if length(pf) ~= num_joints || length(v_max) ~= num_joints || length(a_max) ~= num_joints
    error('Input vectors must have the same length');
end

% Calculate individual times for each joint
joint_times = zeros(num_joints, 1);

for i = 1:num_joints
    % Calculate displacement
    displacement = abs(pf(i) - p0(i));
    
    % Skip joints with no motion
    if displacement < 1e-6
        continue;
    end
    
    % Calculate minimum time needed for acceleration and deceleration phases
    t_acc = v_max(i) / a_max(i);
    
    % Calculate distance traveled during acceleration and deceleration
    d_acc = 0.5 * a_max(i) * t_acc^2;
    d_dec = d_acc;
    
    % Check if we can reach maximum velocity
    if (2 * d_acc >= displacement)
        % No constant velocity phase (triangular profile)
        % Recalculate time for acceleration and deceleration
        t_acc = sqrt(displacement / a_max(i));
        t_dec = t_acc;
        t_const = 0;
    else
        % Can reach maximum velocity (trapezoidal profile)
        t_const = (displacement - 2 * d_acc) / v_max(i);
        t_dec = t_acc;
    end
    
    % Total time for this joint
    joint_times(i) = t_acc + t_const + t_dec;
end

% If end_together is true, find the longest execution time and scale
if end_together
    t_total = max(joint_times);
    
    % Scale velocity and acceleration for joints that would finish earlier
    for i = 1:num_joints
        displacement = abs(pf(i) - p0(i));
        
        % Skip joints with no motion
        if displacement < 1e-6
            continue;
        end
        
        if joint_times(i) < t_total
            % Calculate scaling factor
            scaling_factor = joint_times(i) / t_total;
            
            % Scale velocity and acceleration
            v_max(i) = v_max(i) * scaling_factor;
            a_max(i) = a_max(i) * scaling_factor^2;
        end
    end
else
    % If not ending together, use the original joint times
    t_total = max(joint_times);
end

% Generate time vector
t = 0:dt:t_total;
num_steps = length(t);

% Initialize trajectory arrays
p = zeros(num_steps, num_joints);
v = zeros(num_steps, num_joints);
a = zeros(num_steps, num_joints);

% Generate trajectory for each joint
for joint = 1:num_joints
    displacement = abs(pf(joint) - p0(joint));
    direction = sign(pf(joint) - p0(joint));
    
    % Skip joints with no motion
    if displacement < 1e-6
        p(:, joint) = p0(joint);
        continue;
    end
    
    % Recalculate trajectory parameters with original or scaled values
    t_acc = v_max(joint) / a_max(joint);
    d_acc = 0.5 * a_max(joint) * t_acc^2;
    
    % Check if we can reach maximum velocity with original values
    if (2 * d_acc >= displacement)
        % No constant velocity phase (triangular profile)
        t_acc = sqrt(displacement / a_max(joint));
        t_dec = t_acc;
        t_const = 0;
        v_peak = a_max(joint) * t_acc; % Peak velocity (lower than v_max)
    else
        % Can reach maximum velocity (trapezoidal profile)
        t_const = (displacement - 2 * d_acc) / v_max(joint);
        t_dec = t_acc;
        v_peak = v_max(joint);
    end
    
    % Generate trajectory
    for i = 1:num_steps
        current_t = t(i);
        
        if (current_t <= t_acc)
            % Acceleration phase
            a(i, joint) = direction * a_max(joint);
            v(i, joint) = direction * a_max(joint) * current_t;
            p(i, joint) = p0(joint) + direction * 0.5 * a_max(joint) * current_t^2;
        elseif (current_t <= t_acc + t_const)
            % Constant velocity phase
            a(i, joint) = 0;
            v(i, joint) = direction * v_peak;
            p(i, joint) = p0(joint) + direction * (d_acc + v_peak * (current_t - t_acc));
        elseif (current_t <= t_acc + t_const + t_dec)
            % Deceleration phase
            decel_t = current_t - (t_acc + t_const);
            a(i, joint) = -direction * a_max(joint);
            v(i, joint) = direction * (v_peak - a_max(joint) * decel_t);
            p(i, joint) = pf(joint) - direction * 0.5 * a_max(joint) * (t_dec - decel_t)^2;
        else
            % After motion is complete
            a(i, joint) = 0;
            v(i, joint) = 0;
            p(i, joint) = pf(joint);
        end
    end
end

end
