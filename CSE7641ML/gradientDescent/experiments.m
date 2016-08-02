% This code produces the values used to set the meta parameters. It calls a the estimator with the first version of the estimator : recommenderExperiment. It didn't factorize the computation so they are more time consuming.

clear;

% Use real data:
load ('movie_data');
rateMatrix = train;
testMatrix = test;

N=4;
train1 = zeros(N);
test1 = zeros(N);
train3 = zeros(N);
test3 = zeros(N);
train5 = zeros(N);
test5 = zeros(N);

xgmu = linspace(1,100,N)*10^-5; 
xglambda = linspace(1,1,1);
maxIter = 10;

% Global SVD Test:
lowRank = [1, 3, 5];
for l=1:size(lowRank, 2)
    %move the learning rate
    for i=1:N        
        %move the regularizer
        for j=1:1
            [U, V] = recommenderExperiment(rateMatrix, lowRank(l),maxIter, xgmu(i),xglambda(j));
            trainRMSE = norm((U*V' - rateMatrix) .* (rateMatrix > 0), 'fro') / sqrt(nnz(rateMatrix > 0));
            testRMSE = norm((U*V' - testMatrix) .* (testMatrix > 0), 'fro') / sqrt(nnz(testMatrix > 0));
            if (l==1)
                train1(i,j) = trainRMSE;
                test1(i,j) = testRMSE;
            elseif (l==3)
                train3(i,j) = trainRMSE;
                test3(i,j) = testRMSE;
            else
                train5(i,j) = trainRMSE;
                test5(i,j) = testRMSE;
            end
        end
        
    end
end



