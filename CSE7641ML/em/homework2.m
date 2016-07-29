function [] = homework2( )
% This is a simple example.
% The data contains a 400*101 matrix call X, in which the last
% column is the true label of the assignment, to help you
% evaluate your algorithm. 

	load('data');
	T = X(:,1:100);
	label = X(:,101);

	IDX = mycluster(T,4);

	acc=AccMeasure(label,IDX)

end
