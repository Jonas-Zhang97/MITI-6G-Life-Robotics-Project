%% Input variables
ee_position = [0, 0];
points_position = [-0.13, -0.11; 0.08, -0.03; 0.1 , 0.04];
points_vel = [12, 10; 2, 8; 7, 6];

vel_scaling = 1;

%% Initialize some varibles
points_vec = zeros(size(points_position, 1),2);
distance = zeros(size(points_position, 1),1);

abs_points_vel = zeros(size(points_vel,1), 1);
ang_points_vel = zeros(size(points_vel, 1), 1);
eff_vel = zeros(size(points_vel, 1), 1);

%% Get the essential factors
% Vector and distance between ee and key points
for i = 1:size(points_position, 1)
    points_vec(i, :) = ee_position - points_position(i, :);
    distance(i,1) = norm(points_vec(i,:)');
end

% Absolute value and angle of velocity
for i = 1:size(points_vel,1)
    abs_points_vel(i, 1) = norm(points_vel(i, :)');
    ang_points_vel(i, 1) = acosd(points_vel(i, :) * points_vec(i, :)' ./ (norm(points_vel(i, :)' * norm(points_vec(i, :)'))));
    % Because only the velocity direct facing the ee is critical, effective
    % velocity is calculated
    eff_vel(i, :) = cosd(ang_points_vel(i)).*abs_points_vel(i,:);
end

%% Get the retreat parameters
% Weight, should be in [0, 15] normally
w_dist = 15 .* exp(-abs(distance));
w_vel = 5.0017 .* log(abs(eff_vel)+1);

unit_vec = points_vec ./ distance;

weighted_vec = (w_dist + w_vel) .* unit_vec;

retreat_dir = [sum(weighted_vec(:, 1)), sum(weighted_vec(:, 2)), 0];
unit_retreat_vec = retreat_dir ./ norm(retreat_dir); 

%% Visualization
figure(1)
% Plot the ee position
plot(0, 0, "o")
% text(0, 0, "end effector")

hold on

% Plot the key points
for i = 1:size(points_position, 1)
    plot(points_position(i, 1), points_position(i, 2), "x")
%    text(points_position(i, 1), points_position(i, 2), "key point")
end

% Plot the direction of retreat
quiver(0, 0, unit_retreat_vec(1), unit_retreat_vec(2))
%text(unit_retreat_vec(1), unit_retreat_vec(2), "end effector velocity")

% Plot the direction of the movement of key points
for i = 1:size(points_position, 1)
    quiver(points_position(i, 1), points_position(i, 2), points_vel(i, 1)/15, points_vel(i, 2)/15)
%    text(points_position(i, 1) + points_vel(i, 1)/15, points_position(i, 2) + points_vel(i, 2)/15, "key point velocity")
end