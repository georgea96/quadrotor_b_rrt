function [C] = path_collision_checker(map, points)
% COLLIDE Test whether points collide with an obstacle in an environment.
%   C = collide(map, points).  points is an 2-by-3 matrix where each
%   row is an (x, y, z) point representing the two points in workspace.  
%   If the points are in ijk in the RRT algorithm they need to be
%   transformed to xyz beforehand. C in an M-by-1 logical vector;
%   C(i) = 1 if M(i, :) touches an obstacle and is 0 otherwise.ytutu

%load collision map.
map3d = map.map3d_collision;
sz = size(map3d); nx = sz(1); ny = sz(2); nz = sz(3);

n = 50; %number of intermediate points to check for collision
dx = linspace(points(1,1),points(2,1),n);
dy = linspace(points(1,2),points(2,2),n);
dz = linspace(points(1,3),points(2,3),n);

%assemble the matrix
int_points = zeros(n,3);
int_points(:,1) = dx; 
int_points(:,2) = dy; 
int_points(:,3)= dz;


% vectorized
points = points_to_idx(map, int_points);
idx = (points(:,3)-1)*nx*ny + (points(:,2)-1)*nx + points(:,1);
C = map3d(idx) ~= 255 | map3d(nx*ny*nz+idx) ~= 255 | map3d(nx*ny*nz*2+idx) ~= 255;
C = any(C); %return only one logical value

end
