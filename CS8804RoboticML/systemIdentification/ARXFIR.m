clear all
close all
clc

%Initialisation des parametres
N=700;
Nin=727;
Nout = 902;
%si in est plus grand

in = csvread('logfileCommand.csv', 1, 1, [1,1,Nin,1]);
out = csvread('logfileStatus.csv', 1, 1, [1,1,Nout,1]);
t_in = csvread('logfileCommand.csv', 1, 0, [1,0,Nin,0]);
t_out = csvread('logfilestatus.csv', 1, 0, [1,0,Nout,0]);
%{
in = csvread('logfileCommand.csv', 1, 1, [1,1,N,1]);
out = csvread('logfileStatus.csv', 1, 1, [1,1,N,1]);
t_in = csvread('logfileCommand.csv', 1, 0, [1,0,N,0]);
t_out = csvread('logfilestatus.csv', 1, 0, [1,0,N,0]);
%Na = 100;
%Nb = 50;
%}
%Extraction des donnees
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




%Initialisation des parametres
Nin=485;
Nout =600; 
inV = csvread('logfileCommandV.csv', 1, 1, [1,1,Nin,1]);
outV = csvread('logfileStatusV.csv', 1, 1, [1,1,Nout,1]);
t_inV = csvread('logfileCommandV.csv', 1, 0, [1,0,Nin,0]);
t_outV = csvread('logfilestatusV.csv', 1, 0, [1,0,Nout,0]);
%}\


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


%Initialisation des parametres
Nin=929;
Nout = 945;
%si in est plus grand

inNI = csvread('logfileNoiseInputCommand.csv', 1, 1, [1,1,Nin,1]);
outNI = csvread('logfileNoiseInputStatus.csv', 1, 1, [1,1,Nout,1]);
t_inNI = csvread('logfileNoiseInputCommand.csv', 1, 0, [1,0,Nin,0]);
t_outNI = csvread('logfileNoiseInputStatus.csv', 1, 0, [1,0,Nout,0]);
%Na = 100;
%Nb = 50;

%Extraction des donnees
t_inNI = t_inNI./(10^9);
t_outNI = t_outNI./(10^9);
a = min(t_inNI(1),t_outNI(1));
t_inNI = t_inNI - a;
t_outNI = t_outNI-a;

%Synchronisation des donnees
if max(t_inNI)<max(t_outNI)
    b=floor(max(t_inNI)*60);
    tNI=linspace(0,max(t_inNI), b);
    %out=interp1(t_out,out,t_in);
else
   % in=interp1(t_in,in,t_out);
   a = max(t_outNI);
    tNI=linspace(0,max(t_outNI), (floor(a*60)));
end
inNI=interp1(t_inNI,inNI,tNI);
outNI=interp1(t_outNI,outNI,tNI);

indice = find(isnan(inNI));
taille_ind = length(indice);
for i=1:taille_ind
    inNI(i)=0;
end



%Initialisation des parametres
Nin=899;
Nout = 886;
%si in est plus grand

inNO = csvread('logfileNoiseOutputCommand.csv', 1, 1, [1,1,Nin,1]);
outNO = csvread('logfileNoiseOutputStatus.csv', 1, 1, [1,1,Nout,1]);
t_inNO = csvread('logfileNoiseOutputCommand.csv', 1, 0, [1,0,Nin,0]);
t_outNO = csvread('logfileNoiseOutputStatus.csv', 1, 0, [1,0,Nout,0]);
%Na = 100;
%Nb = 50;

%Extraction des donnees
t_inNO = t_inNO./(10^9);
t_outNO = t_outNO./(10^9);
a = min(t_inNO(1),t_outNO(1));
t_inNO = t_inNO - a;
t_outNO = t_outNO-a;

%Synchronisation des donnees
if max(t_inNO)<max(t_outNO)
    b=floor(max(t_inNO)*60);
    tNO=linspace(0,max(t_inNO), b);
    %out=interp1(t_out,out,t_in);
else
   % in=interp1(t_in,in,t_out);
   a = max(t_outNO);
    tNO=linspace(0,max(t_outNO), (floor(a*60)));
end
inNO=interp1(t_inNO,inNO,tNO);
outNO=interp1(t_outNO,outNO,tNO);

indice = find(isnan(inNO));
taille_ind = length(indice);
for i=1:taille_ind
    inNO(i)=0;
end

