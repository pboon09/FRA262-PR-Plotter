p0 = [0; 0];               % Initial joint positions [rad; m]
pf = [pi/2; 2];          % Final joint positions [rad; m]
v_max = [revolute_speed; prismatic_speed/1000];     % Maximum velocities [rad/s; mm/s]
a_max = [revolute_accel; prismatic_accel/1000];     % Maximum accelerations [rad/s²; mm/s²]
dt = 0.001;                 % Time step (seconds)

[t, p, v, a] = trajectory_generator(p0, pf, v_max, a_max, dt, 0);

figure('Position', [100, 100, 1000, 800]);

joint_names = {'Revolute Joint', 'Prismatic Joint'};
units = {'rad', 'm'};
vel_units = {'rad/s', 'm/s'};
acc_units = {'rad/s²', 'm/s²'};

for i = 1:2
    subplot(3, 2, i);
    plot(t, p(:,i), 'LineWidth', 2);
    grid on;
    xlabel('Time (s)');
    ylabel(['Position (', units{i}, ')']);
    title([joint_names{i}, ' Position']);
    
    subplot(3, 2, i+2);
    plot(t, v(:,i), 'LineWidth', 2);
    grid on;
    xlabel('Time (s)');
    ylabel(['Velocity (', vel_units{i}, ')']);
    title([joint_names{i}, ' Velocity']);
    hold on;
    plot(t, ones(size(t))*v_max(i), 'r--', 'LineWidth', 1.5);
    plot(t, ones(size(t))*-v_max(i), 'r--', 'LineWidth', 1.5);
    hold off;
    legend('Velocity', 'Constraints');
    
    subplot(3, 2, i+4);
    plot(t, a(:,i), 'LineWidth', 2);
    grid on;
    xlabel('Time (s)');
    ylabel(['Acceleration (', acc_units{i}, ')']);
    title([joint_names{i}, ' Acceleration']);
    hold on;
    plot(t, ones(size(t))*a_max(i), 'r--', 'LineWidth', 1.5);
    plot(t, ones(size(t))*-a_max(i), 'r--', 'LineWidth', 1.5);
    hold off;
    legend('Acceleration', 'Constraints');
end

sgtitle('Joint Trajectory Profiles with Velocity and Acceleration Constraints');