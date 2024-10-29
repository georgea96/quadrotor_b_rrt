function track=back_tracking(start,edges,k)
    %Since edges was created based on the max number of iterations there are empty cells
    %Converts cell to matrix to remove empty cells
    temp=cell2mat(edges);
    %Converts the matrix back to cell with dimensions [x y z]_start | [x,y,z]_end
    edges=mat2cell(temp,ones(k,1),[3 3]);
    
    %Creating an empty list to return the back tracked path
    track={};
    
    %Adding the last edge of the path to the track
    track{end+1}=edges{k,2}; %essentially adding the goal ( last edge end)
    track{end+1}=edges{k,1}; %Essentially adding the last sampled pont before the goal (last edge start)
    
    %Repeat while the last entry in the track list is not the start verice
    while(all(cell2mat(track(end))~=cell2mat(start)))
        
        %Repeat for the number of steps that the algorithm took to converge
        for i=1:k
            
            %if the collumn which holds the ends of the edges is the same as the last entry in the track list (i.e. the end vertice of the previous node)
            if edges{i,2}==track{end}
                
                %Add the start of that edge in the track list
                track{end+1}=edges{i,1};
            end
        end
      
    end
    
    %Transposing track to make it into a column of 3D points starting from (Goal (first row) →Start (last row))
    track=track';
    
    %Converting the cell array into matrix
    track=cell2mat(track);
    
    %Flipping the order of the points i.e. changing 
    %(Goal (first row)→Start (last row)) to (Start (first row)→ Goal (last row))to
    track=flip(track);
   
    
end
