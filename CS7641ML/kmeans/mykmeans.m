function [ class, centroid ] = mykmeans( pixels, K )
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


	rows = size(pixels,1);
	columns = size(pixels,2);

	%Initialization of K randon clusters and the class matrix
	centroid = randi([0,255],K,3);
	class = zeros(rows,1);
	maxIT = 20; %limit the number of iterations, advised but not neccesary
	limit = 0;
	%Boolean variable : if stop = 0, no clusters has been modified then we stop the algo
	stop = 1;

	while((stop == 1) &&(limit<maxIT))
		stop = 0;
		
		%cluster assignment 
		%class(i) contains the minimum distance between i-th data point and the clusters, minimized over the clusters
		for i=1:rows
		    centroid_temp = centroid;
		    A = bsxfun(@minus,centroid_temp, pixels(i,:));
		    B = sum(A.^2,2);
		    [x,class(i)] = min(B);
		end

		%cluster center adjustment
		for i=1:K
		    temp = (class==i);
		    c_temp = ((temp.')*pixels)./sum(temp);
		    if c_temp ~=centroid(i,:)
		        centroid(i,:) = c_temp;
		        stop = 1;
		    end
		end
		
	end
		limit = limit +1;
end
    


    



