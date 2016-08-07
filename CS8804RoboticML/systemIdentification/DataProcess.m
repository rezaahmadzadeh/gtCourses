clear all
close all
clc

%Initialisation des parametres
Nin=727;
Nout = 902;
in = csvread('logfileCommand.csv', 1, 1, [1,1,Nin,1]);
out = csvread('logfileStatus.csv', 1, 1, [1,1,Nout,1]);
t_in = csvread('logfileCommand.csv', 1, 0, [1,0,Nin,0]);
t_out = csvread('logfilestatus.csv', 1, 0, [1,0,Nout,0]);

%Definition des parametres du modele apres test sur le GUI
Na = 1;
Nb = 1;


%Transformation temporelle de t_in et t_out
t_in = t_in./(10^9);
t_out = t_out./(10^9);
a = min(t_in(1),t_out(1));
t_in = t_in - a;
t_out = t_out-a;

%Synchronisation des donnees
if max(t_in)<max(t_out)
    b=floor(max(t_in)*60);
    t=linspace(0,max(t_in), b);
    %out=interp1(t_out,out,t_in);
else
   % in=interp1(t_in,in,t_out);
   a = max(t_out);
    t=linspace(0,max(t_out), (floor(a*60)));
end
in=interp1(t_in,in,t);
out=interp1(t_out,out,t);

indice = find(isnan(in));
taille_ind = length(indice);
for i=1:taille_ind
    in(i)=0;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%modele ARX

%Construction of the matrix of regression vector given model complexity Na and Nb
M=max(Na,Nb);
N=length(t);
phi_ARX=zeros( N - M , Na + Nb);
for i=1:(N-M)
    A = -out(N -(i-1)-1:-1:N -(i-1)-Na);
    B = in (N -(i-1)-1:-1:N -(i-1)-Nb);
phi_ARX(i,:) = [A ; B].';
end

%Computation of the parameters
teta_ARX = phi_ARX\(out(N:-1:M+1).')

%Computation of the error of the estimator of the one step hahead prediction.
epsilon_ARX = norm(phi_ARX*teta_ARX - (out(N:-1:M+1).'))

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%modele FIR
t=N;
%Construction of the matrix of regression vector given model complexity Na and Nb
phi_FIR=zeros( (N - Nb), Nb);
for i=1:(N-(Nb+1))
    B = in(N-(i-1)-1:-1:N-(i-1)-Nb);
phi_FIR(i,:) = [B].';
end

%Computation of the parameters
teta_FIR = phi_FIR\(out(N:-1:Nb+1).');
%Computation of the error
epsilon_FIR = norm(phi_FIR*teta_FIR - (out(N:-1:Nb+1).'))


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Initialization of the validation set 
Nin=485;
Nout =600; 
inV = csvread('logfileCommandV.csv', 1, 1, [1,1,Nin,1]);
outV = csvread('logfileStatusV.csv', 1, 1, [1,1,Nout,1]);
t_inV = csvread('logfileCommandV.csv', 1, 0, [1,0,Nin,0]);
t_outV = csvread('logfilestatusV.csv', 1, 0, [1,0,Nout,0]);

%Extraction des donnees
t_inV = t_inV./(10^9);
t_outV = t_outV./(10^9);
a = min(t_inV(1),t_outV(1));
t_inV = t_inV - a;
t_outV = t_outV-a;

%Synchronisation des donnees
if max(t_inV)<max(t_outV)
    b=floor(max(t_inV)*60);
    tV=linspace(0,max(t_inV), b);
    %out=interp1(t_out,out,t_in);
else
   % in=interp1(t_in,in,t_out);
   a = max(t_outV);
    tV=linspace(0,max(t_outV), (floor(a*60)));
end
    
inV=interp1(t_inV,inV,tV);
outV=interp1(t_outV,outV,tV);


indiceV = find(isnan(inV));
taille_indV = length(indiceV);
for i=1:taille_indV
    inV(i)=0;
end


%Validation test of the parameters

%ARX MODEL : Given the validation data set u(t) and y(t) we compare B(q)u(t) (estimator of A(q)y(t)) with A(q)y(t)

%Construction phi_U_ARX that contains the input u(t-1) ... u(t-Nb) for
%t=[N;N-Nb+1)
N=length(tV);
phi_U_ARX=zeros( N -M , Nb);
for i=1:N-M
    B = inV(N-(i-1)-1:-1:N-(i-1)-Nb);
phi_U_ARX(i,:) = B.';
end

%Construction of phi_Y_ARX that contains the validation output y(t), y(t-1) ... y(t-Na) for
%t=[N;N-Na+1)

phi_Y_ARX = zeros(N-M-1,Na+1);
for i=1:N-M
    A= outV(N-(i-1)-1:-1:N-(i-1)-Na);
    phi_Y_ARX(i,:) =[outV(N) A.'];
end

%Computation of error on validation set;
A = [1; teta_ARX(1:Na)];
B = teta_ARX(Na+1:Na+Nb);

%error_ARX = norm( (  phi_ARX_V(1:Nb)*B / (1 + phi_ARX_V(1:Na)*A) ) - outV)
C=(phi_Y_ARX*A);
D=(phi_U_ARX*B );
error_ARX = norm(  (phi_U_ARX*B ) - (phi_Y_ARX*A))



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%FIR MODEL 

%Construction of the regression vector phi_FIR_V for the validation set
phi_FIR_V=zeros( (N - Nb), Nb);
for i=1:(N-(Nb+1))
    B = inV(N-(i-1)-1:-1:N-(i-1)-Nb);
phi_FIR_V(i,:) = [B].';
end

%Computation of error on the validation set
errorModel_FIR = norm(phi_FIR_V*teta_FIR - (outV(N:-1:Nb+1).'))

%}