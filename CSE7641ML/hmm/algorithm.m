function prob = algorithm(q)
	T = 39;
	load sp500;
	data = price_move;

	%data structure definition
	pi1 = [0.2 0.8];
	A = [ [0.8 0.2] ; [0.2 0.8] ];
	B = [ [q (1-q)] ; [(1-q) q] ];
	alpha = zeros(T,2);
	beta = zeros(T,2);

	%initialization
	if data(1)== 1
		alpha(1,1) = pi1(1)*B(1,1);
		alpha(1,2) = pi1(2)*B(1,2);
	else
		alpha(1,1) = pi1(1)*B(2,1);
		alpha(1,2) = pi1(2)*B(2,2);
	end
	beta(T,:) = [1 1];

	cnorm = zeros(1, T);
	cnorm(1) = sum(alpha(1, :));
	alpha(1, :) = alpha(1, :) / cnorm(1);
	cnorm2 = zeros(1, T);
	cnorm2(T) = 1;

	%recursion

	%forward
	for t=2:T
		obs = data(t);
		alpha(t,:);
		alpha(t,:) = alpha(t-1,:) *A ;    
		if(obs==1)
		    alpha(t,:) = [(alpha(t,1)*B(1,1)) (alpha(t,2)*B(1,2))];
		else
		    alpha(t,:) = [(alpha(t,1)*B(2,1)) (alpha(t,2)*B(2,2))];
		end
		cnorm(t) = sum(alpha(t, :));
		alpha(t, :) = alpha(t, :) / cnorm(t);
	end

	%backward
	for t=T-1:-1:1
		obs = data(t+1);    
		if (obs==1)
		    beta(t,:) = beta(t+1,:) * diag(B(1,:)) * A.';
		else
		    beta(t,:) = beta(t+1,:) * diag(B(2,:)) * A.';
		end
		cnorm2(t) = sum(beta(t, :));
		beta(t, :) = beta(t, :) / cnorm2(t);
	end

	%Estimation
	p = zeros(1,T);
	for t=1:T
		pX = alpha(t,1) * beta(t,1) + alpha(t,2) * beta(t,2);
		p(t) = alpha(t,1) * beta(t,1) / pX;
	end
	
	% plot and return the probability
	wanted = p(39)
	figure
	abs = linspace(1,T,T);
	plot(abs,p,'b');
	xlabel('Week')
	ylabel('Probability of the economy to be in good state');
	title(['q=',num2str(q)]);

end
