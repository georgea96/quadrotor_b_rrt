close all;
clear all;
clc;
addpath(genpath('./'));

disp('Planning ...');

%Script Used to load one of the maps in the maps file with names map_ours
map = load_map('maps/map_ours_11.txt', 0.2, 0.5, 0.25);
start = {[0.0  -4.9 0.2]};  %Set start goal in 3D
stop  = {[6.0  18.0-1 5.0]}; %Set end goal in 3D
%Max number of iteration
num_iter = 8e4;

% %maps to load map1, map2,map3
% map = load_map('maps/map3.txt', 0.2, 0.5, 0.25);
% start = {[0.0, 5, 5.0]};
% stop  = {[20, 5, 5]};
% num_iter = 1000000;

nquad = length(start);
%disp(map)
qn=1;
%tic

if collide(map,cell2mat(start))==true
    disp('Start position is on obstacle. Please change start position!');
    errordlg('Start position is on obstacle. Please change start position!','Change start error');
    error('Error');
    return
end    

if collide(map,cell2mat(stop))==true
    disp('Goal position is on obstacle. Please change goal position!');
    errordlg('Goal position is on obstacle. Please change goal position!','Change goal error');
    error('Error');
    return
end  

 path{qn}=rrt_function(map,start,stop,num_iter);
%path{qn}=rrt_goal_on_view(map,start,stop,num_iter);
%path{qn}=bi_directional_rrt_function(map,start,stop,num_iter);
% path{qn}=bi_directional_rrt_function_on_view(map,start,stop,num_iter);
%path{qn}=bi_directional_rrt_function_optimized(map,start,stop,num_iter);


if nquad == 1
    plot_path(map, path{1});
else
    % you could modify your plot_path to handle cell input for multiple robots
end

% Additional init script
init_script;  %Script use to generate the trajectory

% Run trajectory
trajectory = test_trajectory(start, stop, map, path, true); % with visualization

