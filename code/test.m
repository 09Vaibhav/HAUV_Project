waypoints = [0, 0;5, 2;8, 6;10, 10;];
% PD controller
Kp = 1;
Kd = 0.1;
% Initialize variables
current_pos = [0, 0];
current_heading = 0;
prev_error = 0;
threshold_distance =0.05; % Threshold distance for reaching waypoints
   % control loop
for i = 1:size(waypoints, 1)
target_pos = waypoints(i, :);
while norm(target_pos - current_pos) > threshold_distance
% heading error using the LOS algorithm
desired_heading = atan2(target_pos(2) - current_pos(2), target_pos(1) - current_pos(1));
heading_error = desired_heading - current_heading;
%PD controller using control signal(delta)
derivative = heading_error - prev_error;
delta = Kp * heading_error + Kd * derivative;
current_heading = current_heading + delta; %heading angle
delta_t =0.1; % Time step for simulation
current_pos = current_pos + delta_t * [cos(current_heading), sin(current_heading)];
% Update previous error for PD control
prev_error = heading_error;
% Plot the current position (optional, for visualization)
plot(current_pos(1), current_pos(2), 'bo', 'MarkerSize', 10);
hold on;
plot(waypoints(:, 1), waypoints(:, 2), 'rx', 'MarkerSize', 15);
axis equal;
xlabel('X-coordinate');
ylabel('Y-coordinate');
title('Waypoint path following using LOS Algorithm and PD Controller');
drawnow;
end
end

