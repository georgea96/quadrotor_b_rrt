function q_sampled=sample_point(map)
     %Sampling between the map coordinates

     x_sample=map.boundary(1) + (map.boundary(4)-map.boundary(1)) .* rand(1,1);
     y_sample=map.boundary(2) + (map.boundary(5)-map.boundary(2)) .* rand(1,1);
     z_sample=map.boundary(3) + (map.boundary(6)-map.boundary(3)) .* rand(1,1);

     %Assembling the sampled q
     q_sampled= [x_sample, y_sample, z_sample];
end

