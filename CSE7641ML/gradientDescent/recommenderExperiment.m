function [ U, V ] = recommenderExperiment( rateMatrix, lowRank, maxIter,learningRate,regularizer)
    % Please type your name here:
    name = 'Benbihi, Assia';
    disp(name); % Do not delete this line.
    
    % Random initialization:
    [n1, n2] = size(rateMatrix);
    U = rand(n1, lowRank) / lowRank;
    V = rand(n2, lowRank) / lowRank;

    % Gradient Descent:
    
    %one loop
    m = n1;
    n = n2;
    r = lowRank;
    M = rateMatrix;
    lambda = regularizer;
    mu = learningRate;
    
    for it=1:maxIter
        
        for v=1:m
            for k=1:r
                Mv = M(v,:);
                Uv = U(v,:);
                VUt = V*(Uv.');
                U(v,k) = U(v,k) - mu*(2 * lambda * U(v,k) - 2*( Mv - (VUt.')) * V(:,k));
            end
        end
        
        for j=1:n
            for k=1:r
                Mj = M(:,j);
                Vj = V(j,:);
                UVt = U * (Vj.');
                V(j,k) =V(j,k) - mu*( 2 * lambda * V(j,k) - 2 * ( U(:,k).') * (Mj - UVt));
            end
        end
    end
end
