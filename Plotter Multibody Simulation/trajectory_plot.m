% Example for a 2-joint robot (1 revolute, 1 prismatic):
p0 = [0; 0];              % Initial joint positions [rad; m]
pf = [pi/2; 0.25];         % Final joint positions [rad; m]
v_max = [revolute_speed, prismatic_speed];         % Maximum velocities [rad/s; m/s]
a_max = [revolute_accel, prismatic_accel];       % Maximum accelerations [rad/s²; m/s²]
dt = 0.001;                % Time step (seconds)

% Generate synchronized trajectory
[p, v, a] = trajectory_generator(p0, pf, v_max, a_max, dt);

% Plot results
figure;

% Plot joint positions
subplot(3,2,1);
plot(t, p(:,1));
ylabel('Position (rad)');
title('Revolute Joint Position');
grid on;

subplot(3,2,2);
plot(t, p(:,2));
ylabel('Position (m)');
title('Prismatic Joint Position');
grid on;

% Plot joint velocities
subplot(3,2,3);
plot(t, v(:,1));
ylabel('Velocity (rad/s)');
title('Revolute Joint Velocity');
grid on;

subplot(3,2,4);
plot(t, v(:,2));
ylabel('Velocity (m/s)');
title('Prismatic Joint Velocity');
grid on;

% Plot joint accelerations
subplot(3,2,5);
plot(t, a(:,1));
xlabel('Time (s)');
ylabel('Acceleration (rad/s²)');
title('Revolute Joint Acceleration');
grid on;

subplot(3,2,6);
plot(t, a(:,2));
xlabel('Time (s)');
ylabel('Acceleration (m/s²)');
title('Prismatic Joint Acceleration');
grid on;