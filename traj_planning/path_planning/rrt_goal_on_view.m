function track=rrt_goal_on_view(map,start,goal,num_iter)
%Max number of iteration
% num_iter=2;%5e4;%9e4;
%Goal Bias
tolerance=0.3;%0.1;%0.2;
%Matrix num_iterations x 3
vertices=Inf(num_iter,3);
%edges cell array 2 collumns of [0,0,0] arrays
edges=cell(num_iter,2);

%Adding start to the vertices list
vertices(1,1:3)=cell2mat(start);

tic
for k=2:num_iter
    while 1
        
        %Calling a function used to sample a point
        q_sampled=sample_point(map);
        
        %Check if it lies on an obstacle
        point_on_obstacle=collide(map,q_sampled);
        
        %Getting the index for the nearest vertice to the sampled point
        k_nearest=dsearchn(vertices,q_sampled);
        
        %Combining the nearest vertice and the sampled point as requested for path collision checker function
        path_to_check=[vertices(k_nearest,1:3);q_sampled];
        
        %Passing the matrix consisting of the two points to the path collision checker
        path_on_obstacle=path_collision_checker(map,path_to_check);
        
        %Checking if either sampled point doesn't lie on obstacle
        if (point_on_obstacle==false)&&(path_on_obstacle==false)
            
            %if it doesn't lie on obstacle then all good
            %add the nearest vertice already in the path to the first column of edges (i.e. start of an edge)
            edges{k-1,1}=vertices(k_nearest,1:3);
            %add the sampled point to the second column of edges (i.e. end of an edge)
            edges{k-1,2}=q_sampled;
            
            %check if there exists a direct connection (collision free) between sample point
            %and goal
            path_to_check_final=[q_sampled;cell2mat(goal)];
            path_on_obstacle_final=path_collision_checker(map,path_to_check_final);
            if (path_on_obstacle_final==false)
                edges{k,1}=q_sampled;
                edges{k,2}=cell2mat(goal);
                break
            end
            
            %break the while loop we found a collision free sample point (time to check for the goal)
          break
        end
    end
    
    %     %Adding the sampled point to the vertices list
        vertices(k,1:3)=q_sampled;
        
        % break if goal found 
        if isequal(edges{k,2},cell2mat(goal))==true
            break
        end        
end

disp(k);

% displays error if solution not found as number of iterations was not enough
if k==num_iter && isequal(edges{k,2},cell2mat(goal))==false
    disp('The algortihm did not converge. Increase number of iterations!');
    errordlg('The algortihm did not converge. Increase number of iterations!','Convergence Error');
    error('Error');
end
        
%Call the function for back tracking (i.e. getting the path from start → goal)
track=back_tracking(start,edges,k);
toc
plot_path_rrt(map,track,edges);

end

