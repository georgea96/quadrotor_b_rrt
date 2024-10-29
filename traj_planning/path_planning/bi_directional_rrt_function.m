function track=bi_directional_rrt_function(map,start,goal,num_iter)

tic
start1 = start;
start2 = goal;
goal1 = goal;
goal2 = start;

%flag1 raised if q_sampled1 is collision free and its edge to nearest
%neigbour is collision free
flag1 = false;
%flag1 raised if q_sampled2 is collision free and its edge to nearest
%neigbour is collision free
flag2 = false;


%flag to terminate porgram if not converged on specified iterations
flag_tree_connected = false;

tolerance_3 = 10;

%Max number of iteration
% num_iter=5e4;%9e4;
%Goal Bias
tolerance=0.3;%0.1;%0.2;
points_tolerance = 0.5;
%Matrix num_iterations x 3
vertices1=Inf(num_iter,3);
vertices2=Inf(num_iter,3);
%edges cell array 2 collumns of [0,0,0] arrays
edges1=cell(num_iter,2);
edges2=cell(num_iter,2);

%Adding start1 to the vertices1 list
vertices1(1,1:3)=cell2mat(start1);
%Adding start2 to the vertices2 list
vertices2(1,1:3)=cell2mat(start2);
for k=2:num_iter
    while 1
        
        %Calling a function used to sample a point
         if flag1==false 
             q_sampled1=sample_point(map);
         end
         if flag2==false
            q_sampled2=sample_point(map);
         end    
        
         %Check if it lies on an obstacle
         point_on_obstacle1=collide(map,q_sampled1);
         point_on_obstacle2=collide(map,q_sampled2);
         
         %Getting the index for the nearest vertice to the sampled point
         k_nearest1=dsearchn(vertices1,q_sampled1);  
         k_nearest2=dsearchn(vertices2,q_sampled2); 
         
         %Combining the nearest vertice and the sampled point as requested for path collision checker function
         path_to_check1=[vertices1(k_nearest1,1:3);q_sampled1];
         path_to_check2=[vertices2(k_nearest2,1:3);q_sampled2];
         
         %Passing the matrix consisting of the two points to the path collision checker
         path_on_obstacle1=path_collision_checker(map,path_to_check1);
         path_on_obstacle2=path_collision_checker(map,path_to_check2);
        
         %Checking if first tree sampled point doesn't lie on obstacle
         if (point_on_obstacle1==false)&&(path_on_obstacle1==false)
             flag1 = true;
             %if it doesn't lie on obstacle then all good
             %add the nearest vertice already in the path to the first column of edges (i.e. start of an edge)
             edges1{k-1,1}=vertices1(k_nearest1,1:3);
             %add the sampled point to the second column of edges (i.e. end of an edge)
             edges1{k-1,2}=q_sampled1;
             %break the while loop we found a collision free sample point (time to check for the goal)
         end
         
         %Checking if second tree sampled point doesn't lie on obstacle
         if (point_on_obstacle2==false)&&(path_on_obstacle2==false)
             flag2=true;
             %if it doesn't lie on obstacle then all good
             %add the nearest vertice already in the path to the first column of edges (i.e. start of an edge)
             edges2{k-1,1}=vertices2(k_nearest2,1:3);
             %add the sampled point to the second column of edges (i.e. end of an edge)
             edges2{k-1,2}=q_sampled2;
         end
         
         %if both points sumpled are collsion free, flags are updated to false
         if flag1==true && flag2==true
             flag1 = false;
             flag2 = false;
             break;
         end
         
    end
    
    %Adding the sampled point to the vertices1 list
    vertices1(k,1:3)=q_sampled1;
    %Adding the sampled point to the vertices2 list
    vertices2(k,1:3)=q_sampled2;
    
    % check if current sampled points of each tree are within a certain range and connect 
    if (pdist2(q_sampled1,q_sampled2)<tolerance_3)
        path_to_check_connect=[q_sampled1;q_sampled2];
        path_on_obstacle1 = path_collision_checker(map,path_to_check_connect);
        if (path_on_obstacle1==false)
            flag_tree_connected = true;
            break
        end
    end
end



disp(k);
% displays error if solution not found as number of iterations was not enough
if k==num_iter && flag_tree_connected==false
    disp('The algortihm did not converge. Increase number of iterations!');
    errordlg('The algortihm did not converge. Increase number of iterations!','Convergence Error');
    error('Error');
end

%Call the function for back tracking (i.e. getting the path from start to goal)
track1=back_tracking(start1,edges1,k-1);
track2=back_tracking(start2,edges2,k-1);
%track2 provides a feasible path from goal to the sampled point stoped, and
%need to be fliped in order to have a path from sampled point stoped to
%goal, before appending to first tree
track2=flip(track2);
track = [track1; track2];
edges = [edges1; edges2];

toc

plot_path_rrt(map,track,edges);

end

