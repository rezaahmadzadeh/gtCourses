function [ class ] = mycluster( bow, K )
% Input:
%     bow: data set. Bag of words representation of text document as
%     described in the assignment.
%
%     K: the number of desired topics/clusters.
%
% Output:
%     class: the assignment of each topic. The
%     assignment should be 1, 2, 3, etc.

	%Random Initialization of parameters
	load('data');
	nd = size(bow,1);
	nw = size(bow,2);
	mu = ones(nw,K);
	gamma = zeros(nd,K);
	for j=1:nw
		mu(j,:) = rand(1,K);
		mu(j,:) = mu(j,:)/sum(mu(j,:));
	end
	pi = rand(1,K);
	pi = pi/sum(pi);

	%Accuracy control parameter
	label = X(:,101);
	acc = 0;
	acc_min = 90; %minimum required accuracy
	it_max = 10;
	it_count = 1;
	acc_temp = -1*ones(1,it_max);

	%loop until the accuracy is above acc_min
	while (it_count <= it_max && acc<acc_min)
		%%Expectation phase
		for i=1:nd
		    tmp = zeros(1,K);
		    for c=1:K
		        tmp(c) = (pi(c)*prod( (mu(:,c))'.^bow(i,:)) );
		    end
		    tmp_sum = sum(tmp);
		    for c=1:K
		        gamma(i,c) = tmp(c)/tmp_sum;%expectation of document D_i belonging to cluster c
		    end
		end
		
		%Maximization phase
		for c=1:K
		    tmp = 0;
		    for i=1:nd
		        tmp = tmp + gamma(i,c)*sum(bow(i,:));
		    end
		    for j=1:nw
		        mu(j,c) = sum(gamma(:,c).*bow(:,j))/tmp;% probability of word j in cluster c
		    end
		    pi(c) = sum(gamma(:,c))/nd;%prior probability of cluster c
		end
		
		%Bayes decision
		class = zeros(nd,1);
		for i=1:nd
		    [x,class(i)] = max(gamma(i,:)); %decision step
		end
		
		%evaluation
		acc = AccMeasure(class,label);
		acc_temp(it_count) = acc;
		it_count = it_count + 1;
	   
	end
end


