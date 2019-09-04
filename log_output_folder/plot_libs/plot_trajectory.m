close all
clear
clc

fig = figure('pos',[10 10 1900 1000]); 
addpath('export_fig');
data = load('../log_output_4.txt'); hold on;
axis equal; axis([-1 10 -2 2 0 4.5]); grid on;

vert_obst1_xy   =       [ data(1,24), data(1,25) ];
vert_obst2_xy   =       [ data(1,26), data(1,27) ];
horiz_obst_xz   =       [ data(1,28), data(1,29) ];
horiz_obst2_xz   =       [ 2.5 3.2];
target_xyz      =       [ data(1,30), data(1,31), data(1,32) ];
camera_intrins  =       [ data(1,33), data(1,34) ];

dyn_obst_pos    =       [data(:,17), data(:,18), data(:,19)];
dyn_obst_vel    =       [data(:,20), data(:,21), data(:,22)];
dyn_obst_delay  =       data(:,23);

UAV_position    =       [data(:,2), data(:,3), data(:,4)];
UAV_orientation =       [data(:,5), data(:,6), data(:,7), data(:,8)];
UAV_controls =          [data(:,9), data(:,10), data(:,11), data(:,12)];
des_goal        =       [data(:,13), data(:,14), data(:,15), data(:,16)];

%% Obstacles and Target Drawings

vert_obst1_edges_start = [ vert_obst1_xy(1) - 0.15/2, vert_obst1_xy(2) - 0.15/2, 0 ];
vert_obst1_edges_lenght = [0.15, 0.15, 4];

vert_obst2_edges_start = [ vert_obst2_xy(1) - 0.15/2, vert_obst2_xy(2) - 0.15/2, 0 ];
vert_obst2_edges_lenght = [0.15, 0.15, 4];

horiz_obst_edges_start = [ horiz_obst_xz(1) - 0.15/2, -1.5, horiz_obst_xz(2) - 0.15/2 ];
horiz_obst_edges_lenght = [0.15, 3, 0.15];

horiz_obst2_edges_start = [ horiz_obst2_xz(1) - 0.15/2, -1.5, horiz_obst2_xz(2) - 0.15/2 ];
horiz_obst2_edges_lenght = [0.15, 3, 0.15];

plotcube(vert_obst1_edges_lenght, vert_obst1_edges_start, 0.7, [0 1 0]);
plotcube(vert_obst2_edges_lenght, vert_obst2_edges_start, 0.7, [0 0 1]);
plotcube(horiz_obst_edges_lenght, horiz_obst_edges_start, 0.7, [1 0 0]); 
plotcube(horiz_obst2_edges_lenght, horiz_obst2_edges_start, 0.7, [1 1 0]); 

[I] = imread('tag36h11.png');
[indexedImage, map] = rgb2ind(I, 3);
I2 = ind2rgb(indexedImage, map);

[Y,Z] = meshgrid(-.5:.5:.5,.5:.5:1.5);
X = 10 + zeros(length(Y));
warp(X,Y,Z,I2,map);

%% UAV Trajectories drawing

cmap = colormap('summer');
trajs_cell_array = {};
dyn_obst_array = {};

num_trajs = 0;
prev_goal_pose = [0,0,0,0];
curr_traj = [];
curr_dyn_obst = [];

for i = 1:length(dyn_obst_delay)
    
    curr_goal_pose = des_goal(i,:);
    
    if norm(prev_goal_pose - curr_goal_pose) > 1e-3
        prev_goal_pose = curr_goal_pose;
        num_trajs = num_trajs + 1;
        trajs_cell_array = [trajs_cell_array, curr_traj];
        dyn_obst_array = [dyn_obst_array, curr_dyn_obst];
        curr_traj = [];
        curr_dyn_obst = [];
    end
    
    if num_trajs > 0
        curr_traj = [curr_traj; UAV_position(i,:), UAV_orientation(i,:)];  
        curr_dyn_obst = [curr_dyn_obst; dyn_obst_pos(i,:), dyn_obst_vel(i,:)];
    end
    
end

trajs_cell_array = [trajs_cell_array, curr_traj];
curr_traj = [];

dyn_obst_array = [dyn_obst_array, curr_dyn_obst];
curr_dyn_obst = [];

for i = 1 : 10 : length(trajs_cell_array{1}) - 90
   v = [trajs_cell_array{1}(i, 1:3), trajs_cell_array{1}(i, 5:7)];
   T = v2t(v);
   triad('Linewidth', 3, 'Matrix', T);
end

for i = length(trajs_cell_array{1}) - 90 : 30 : length(trajs_cell_array{1})
   v = [trajs_cell_array{1}(i, 1:3), trajs_cell_array{1}(i, 5:7)];
   T = v2t(v);
   triad('Linewidth', 2, 'Matrix', T);
end

plot3(trajs_cell_array{1}(:,1), trajs_cell_array{1}(:,2), trajs_cell_array{1}(:,3),...
      'LineWidth', 2, 'Color', [0, 192/255, 1]);

r = 1;
g = 1;
b = 0;

curr_position = dyn_obst_array{1}(1,1:3);
dyn_obst_edges_start = [ curr_position(1,1) - 0.1, curr_position(1,2) - 0.1, curr_position(1,3) - 0.1 ];
dyn_obst_edges_lenght = [0.2, 0.2, 0.2];
plotcube(dyn_obst_edges_lenght, dyn_obst_edges_start, 0.7, [r g b]);

for i = 2 : 15 : length(dyn_obst_array{1})
    
    curr_position = curr_position + dyn_obst_array{1}(1,4:6) * 0.01 * 10;
    g = g - 0.04;
    b = b + 0.04;
    dyn_obst_edges_start = [ curr_position(1,1) - 0.1, curr_position(1,2) - 0.1, curr_position(1,3) - 0.1 ];
    dyn_obst_edges_lenght = [0.2, 0.2, 0.2];
    plotcube(dyn_obst_edges_lenght, dyn_obst_edges_start, 0.7, [r g b]);
    
end

xlabel('x');
ylabel('y');
zlabel('z');
set(gca,'YDir','normal')

set(gca,'FontSize',20);
v = [-3 5 3.5];
[caz,cel] = view(v)
%export_fig(['example2','.pdf'], '-pdf');