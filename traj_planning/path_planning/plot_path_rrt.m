% plot_path_rrt function was adapted based on plot_path function
% of https://github.com/yrlu/quadrotor

function plot_path_rrt(map, path,edges)
% PLOT_PATH Visualize a path through an environment
%   PLOT_PATH(map, path) creates a figure showing a path through the
%   environment.  path is an N-by-3 matrix where each row corresponds to the
%   (x, y, z) coordinates of one point along the path.

figure(1);

hold on;
for i = 1:size(map.blocks,1)
    block = map.blocks(i, :);
    
    x = [ones(4,1) * block(1); ones(4,1) * block(4)];
    y = [ones(2,1) * block(5); ones(2,1) * block(2); ones(2,1) * block(5); ones(2,1) * block(2)];
    z = [block(3);block(6);block(3);block(6);block(3);block(6);block(3);block(6)];


    vert = [x, y, z];
    fac = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
    c = block(7:9)/255;
    patch('Vertices',vert,'Faces',fac,...
          'FaceVertexCData',hsv(6),'FaceColor',c);

    
    x = [ones(4,1) * block(1); ones(4,1) * block(4)];
    y = [block(2);block(5);block(2);block(5);block(2);block(5);block(2);block(5)];
    z = [ones(2,1) * block(3); ones(2,1) * block(6); ones(2,1) * block(3); ones(2,1) * block(6)];

    vert = [x, y, z];
    fac = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
    c = block(7:9)/255;
    patch('Vertices',vert,'Faces',fac,...
          'FaceVertexCData',hsv(6),'FaceColor',c);
end

if size(path,1) > 0
    pcshow(path, [0, 0 ,0],'MarkerSize', 100.1); %Plots path vertices
end

edges=cell2mat(edges);
if size(edges,1)>0
    for i=1:size(edges,1)
        hold on;
        plot3([edges(i,1) edges(i,4)],[edges(i,2) edges(i,5)], [edges(i,3) edges(i,6)]);
%         pause(0.5)
    end
end

track_length = length(path)-1;
%track=cell2mat(track)
if length(path)>0
    for i=1:track_length
        hold on;
        plot3([path(i,1) path(i+1,1)],[path(i,2) path(i+1,2)], [path(i,3) path(i+1,3)],'LineWidth',8);
%         pause(0.5)
    end
end



% axis([map.boundary(1)-1, map.boundary(4)-1, map.boundary(2)-1,map.boundary(5)+1,map.boundary(3)+1,map.boundary(6)+1])
hold off;
% view(3);
set(gca,'Color',[1 1 1]);

set(gcf,'Color',[1 1 1]);
end