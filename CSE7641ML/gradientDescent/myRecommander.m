function [ U, V ] = myRecommender( rateMatrix, lowRank )
    name = 'Benbihi, Assia';
    disp(name); % Do not delete this line.

    % parameters
    maxIter = 50; % Choose your own.
    learningRate = 0.0007; % Choose your own.
    regularizer = 1; % Choose your own.
    
    % random initialization
    [n1, n2] = size(rateMatrix);
    U = rand(n1, lowRank) / lowRank;
    V = rand(n2, lowRank) / lowRank;

    % parameter initialization 
    m = n1;
    n = n2;
    r = lowRank;
    M = rateMatrix;
    lambda = regularizer;
    mu = learningRate;
    
    iter = 1; %iteration number
    tic;
    trainRMSE_tmp = 0.0;
    trainRMSE = 10.0; %error
    diff_train = 1; %error decrement
 	
 	%gradient descent
    while(  (trainRMSE > 1.0) && (iter < maxIter)  && ((diff_train) > 10^-5) && (toc < 240) )
        
        V = V -mu*( 2*lambda*V - 2* ( ( (M-U*(V.')).*(rateMatrix > 0) ) .') *U);
        U = U - mu*( 2*lambda*U - 2* ( (M-U*(V.')).*(rateMatrix > 0) )*V)  ;
        trainRMSE = norm((U*V' - rateMatrix) .* (rateMatrix > 0), 'fro') / sqrt(nnz(rateMatrix > 0));

        iter = iter + 1;
        diff_train = abs(trainRMSE - trainRMSE_tmp);
        trainRMSE_tmp = trainRMSE;     
        logTime = toc;    
    end  
end
