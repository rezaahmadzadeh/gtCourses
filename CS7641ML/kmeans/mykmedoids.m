function [ class, centroid ] = mykmedoids( pixels, K )
% Used distance : manhattan distance.
% Input:
%     pixels: data set. Each row contains one data point. For image
%     dataset, it contains 3 columns, each column corresponding to Red,
%     Green, and Blue component.
%
%     K: the number of desired clusters. Too high value of K may result in
%     empty cluster error. Then, you need to reduce it.
%
% Output:
%     class: the class assignment of each data point in pixels. The
%     assignment should be 1, 2, 3, etc. For K = 5, for example, each cell
%     of class should be either 1, 2, 3, 4, or 5. The output should be a
%     column vector with size(pixels, 1) elements.
%
%     centroid: the location of K centroids in your result. With images,
%     each centroid corresponds to the representative color of each
%     cluster. The output should be a matrix with size(pixels, 1) rows and
%     3 columns. The range of values should be [0, 255].

	rows=size(pixels,1);

	%initalization of K random clusters within the data points. 
	%centroid_idx contains the indices of the data points in [1,N] chosen as cluster centers. 
	centroid_idx = randperm(rows, K).';
	centroid = pixels(centroid_idx,:);

	%initialization of the class vector
	class = zeros(rows,1);
	
	limit = 0;
	maxIT = 50; %limit the numer of iterations, optional
	stop = 1; %if stop = 0, no clusters has been modified then we stop the algo
	
	while((stop == 1) &&(limit<maxIT))
		stop = 0;
		%cluster assignment: Assign each data point to the nearest cluster. 
		for i=1:rows %for each point
			A = bsxfun(@minus,centroid, pixels(i,:));   
			B = sum(abs(A),2); % compute the distance between i-th point and cluster center
			[x,class(i)] = min(B); %i-th point is in the class(i) cluster
		end
		    
		%cluster adjustement
		for k=1:K %for each cluster
			%compute the sum of pairwise dissimilarities between data points and the cluster center
			assign_temp = class==centroid_idx(k);
		    indices_temp = find(assign_temp); %set of indices of pixel in the k-th cluster
		    cluster_points = pixels(indices_temp,:); %set of pixels in the k-th cluster center
		    temp1 = bsxfun(@minus, cluster_points, centroid(k));
		    objective_temp = sum(sum(abs(temp1),2)); % sum of parwise dissimilarity between the cluster center and the rest of the cluster points
		    
		    %for each points in the k-th cluster
		    for l=1:size(indices_temp)
		        temp2 = bsxfun(@minus, cluster_points,cluster_points(l));
		        objective_temp2 = sum(sum(abs(temp2),2)); % sum of parwise dissimilarity between the l-th cluster point and the other cluster points
		        if (objective_temp2 < objective_temp) %the l-th point minimize the sum of pairwise dissimilarity in the cluster 
		            centroid_idx(k) = l; % it becomes the new cluster center
		            centroid(k) = pixels(l,:);
		            stop = 1;
		        end
		    end
		end
		limit = limit + 1;
	end
      
end
