clear all
close all
clc
N = 95

%DATA EXTRACTION
t = csvread('hr_100_sort_LM.csv', 0, 0, [0,0,N,0]);
hr = csvread('hr_test.csv', 0, 1, [0,1,N,1]);
hr_100_sort_LM = csvread('hr_100_sort_LM.csv', 0, 1, [0,1,N,1]);
hr_100_sort_LRU = csvread('hr_100_sort_LRU.csv', 0, 1, [0,1,N,1]);
hr_100_sort_rand = csvread('hr_100_sort_rand.csv', 0, 1, [0,1,N,1]);
hr_100_shuffle_LM = csvread('hr_100_shuffle_LM.csv', 0, 1, [0,1,N,1]);
hr_100_shuffle_LRU = csvread('hr_100_shuffle_LRU.csv', 0, 1, [0,1,N,1]);
hr_100_shuffle_rand = csvread('hr_100_shuffle_rand.csv', 0, 1, [0,1,N,1]);

hr_90_shuffle_rand = csvread('hr_90_shuffle_rand.csv', 0, 1, [0,1,N,1]);
hr_80_shuffle_rand = csvread('hr_80_shuffle_rand.csv', 0, 1, [0,1,N,1]);
hr_70_shuffle_rand = csvread('hr_70_shuffle_rand.csv', 0, 1, [0,1,N,1]);
hr_60_shuffle_rand = csvread('hr_60_shuffle_rand.csv', 0, 1, [0,1,N,1]);
hr_50_shuffle_rand = csvread('hr_50_shuffle_rand.csv', 0, 1, [0,1,N,1]);

hr_90_shuffle_LRU = csvread('hr_90_shuffle_LRU.csv', 0, 1, [0,1,N,1]);
hr_80_shuffle_LRU = csvread('hr_80_shuffle_LRU.csv', 0, 1, [0,1,N,1]);
hr_70_shuffle_LRU = csvread('hr_70_shuffle_LRU.csv', 0, 1, [0,1,N,1]);
hr_60_shuffle_LRU = csvread('hr_60_shuffle_LRU.csv', 0, 1, [0,1,N,1]);
hr_50_shuffle_LRU = csvread('hr_50_shuffle_LRU.csv', 0, 1, [0,1,N,1]);

hr_90_shuffle_LM = csvread('hr_90_shuffle_LM.csv', 0, 1, [0,1,N,1]);
hr_80_shuffle_LM = csvread('hr_80_shuffle_LM.csv', 0, 1, [0,1,N,1]);
hr_70_shuffle_LM = csvread('hr_70_shuffle_LM.csv', 0, 1, [0,1,N,1]);
hr_60_shuffle_LM = csvread('hr_60_shuffle_LM.csv', 0, 1, [0,1,N,1]);
hr_50_shuffle_LM = csvread('hr_50_shuffle_LM.csv', 0, 1, [0,1,N,1]);

t_100_sort_LM = csvread('t_100_sort_LM.csv', 0, 1, [0,1,N,1]);
t_100_sort_LRU = csvread('t_100_sort_LRU.csv', 0, 1, [0,1,N,1]);
t_100_sort_rand = csvread('t_100_sort_rand.csv', 0, 1, [0,1,N,1]);
t_100_shuffle_LM = csvread('t_100_shuffle_LM.csv', 0, 1, [0,1,N,1]);
t_100_shuffle_LRU = csvread('t_100_shuffle_LRU.csv', 0, 1, [0,1,N,1]);
t_100_shuffle_rand = csvread('t_100_shuffle_rand.csv', 0, 1, [0,1,N,1]);

t_90_shuffle_rand = csvread('t_90_shuffle_rand.csv', 0, 1, [0,1,N,1]);
t_80_shuffle_rand = csvread('t_80_shuffle_rand.csv', 0, 1, [0,1,N,1]);
t_70_shuffle_rand = csvread('t_70_shuffle_rand.csv', 0, 1, [0,1,N,1]);
t_60_shuffle_rand = csvread('t_60_shuffle_rand.csv', 0, 1, [0,1,N,1]);
t_50_shuffle_rand = csvread('t_50_shuffle_rand.csv', 0, 1, [0,1,N,1]);

t_90_shuffle_LRU = csvread('t_90_shuffle_LRU.csv', 0, 1, [0,1,N,1]);
t_80_shuffle_LRU = csvread('t_80_shuffle_LRU.csv', 0, 1, [0,1,N,1]);
t_70_shuffle_LRU = csvread('t_70_shuffle_LRU.csv', 0, 1, [0,1,N,1]);
t_60_shuffle_LRU = csvread('t_60_shuffle_LRU.csv', 0, 1, [0,1,N,1]);
t_50_shuffle_LRU = csvread('t_50_shuffle_LRU.csv', 0, 1, [0,1,N,1]);

t_90_shuffle_LM = csvread('t_90_shuffle_LM.csv', 0, 1, [0,1,N,1]);
t_80_shuffle_LM = csvread('t_80_shuffle_LM.csv', 0, 1, [0,1,N,1]);
t_70_shuffle_LM = csvread('t_70_shuffle_LM.csv', 0, 1, [0,1,N,1]);
t_60_shuffle_LM = csvread('t_60_shuffle_LM.csv', 0, 1, [0,1,N,1]);
t_50_shuffle_LM = csvread('t_50_shuffle_LM.csv', 0, 1, [0,1,N,1]);

t_nocache = csvread('t_nocache.csv', 0, 1, [0,1,N,1]);

%{
%WORKLOAD 
abs = csvread('url_size.csv', 1, 3, [1,3,N,3]);
w1 = csvread('url_size.csv', 1, 5, [1,5,N,5]);
w2 = csvread('url_size.csv', 1, 8, [1,8,N,8]);

figure
plot(abs,w1,'k')
xlabel('Web page sequence')
ylabel('Web page size')
title('Workload distribution 1')

figure
plot(abs,w2,'k')
xlabel('Web page sequence')
ylabel('Web page size')
title('Workload distribution 2')
%}
%{
%FIGURE 1
%SIZE=100
%WORKLOAD = SHUFFLE
% HR(n) for the 3 policies
plot(t,hr_100_shuffle_LM,'r',t,hr_100_shuffle_LRU,'b',t,hr_100_shuffle_rand,'g')
xlabel('Discrete time')
ylabel('Hit Rate')
legend('LRU-MIN','LRU','RANDOM','Location','northoutside','Orientation','horizontal')
legend('boxoff')
title({'EVOLUTION OF HIT RATE WITH TIME';' ' ;'WORKLOAD = SHUFFLE';'CACHE SIZE = 100*1024'})

%VARIANCE1 OF THE HIT RATE 
%SIZE=100
%WORKLOAD = SHUFFLE

hr_100_shuffle_LM_mean = (sum(hr_100_shuffle_LM)/N) 
hr_100_shuffle_LRU_mean =  (sum(hr_100_shuffle_LRU)/N) 
hr_100_shuffle_rand_mean = (sum(hr_100_shuffle_rand)/N)

hr_100_shuffle_LM_var = sum( (hr_100_shuffle_LM - (sum(hr_100_shuffle_LM)/N) ).^2)/N
hr_100_shuffle_LRU_var = sum((hr_100_shuffle_LRU - (sum(hr_100_shuffle_LRU)/N) ).^2)/N
hr_100_shuffle_rand_var = sum((hr_100_shuffle_rand - (sum(hr_100_shuffle_rand)/N) ).^2)/N
%}

%{
%SIZE=100
%WORKLOAD = sort
% HR(n) for the 3 policies
figure
plot(t,hr_100_sort_LM,'r',t,hr_100_sort_LRU,'b',t,hr_100_sort_rand,'g')
xlabel('Discrete time')
ylabel('Hit Rate')
legend('LRU-MIN','LRU','RANDOM')
title({'EVOLUTION OF HIT RATE WITH TIME';' ' ;'WORKLOAD = SORTED';'CACHE SIZE = 100*1024'})
%}

%{
%SIZE=100
%POLICY = RANDOM
% HR(n) for the 2 workloads
figure
plot(t,hr_100_sort_rand,'r',t,hr_100_shuffle_rand,'b')
xlabel('Discrete time')
ylabel('Hit Rate')
legend('sort','shuffle')
title({'EVOLUTION OF HIT RATE WITH TIME';' ' ;'POLICY = RANDOM';'CACHE SIZE = 100*1024'})

%SIZE=100
%POLICY = LRU
% HR(n) for the 2 workloads
figure
plot(t,hr_100_sort_LRU,'r',t,hr_100_shuffle_LRU,'b')
xlabel('Discrete time')
ylabel('Hit Rate')
legend('sort','shuffle')
title({'EVOLUTION OF HIT RATE WITH TIME';' ' ;'POLICY = LRU';'CACHE SIZE = 100*1024'})

%SIZE=100
%POLICY = LRU-MIN
% HR(n) for the 2 workloads
figure
plot(t,hr_100_sort_LM,'r',t,hr_100_shuffle_LM,'b')
xlabel('Discrete time')
ylabel('Hit Rate')
legend('sort','shuffle')
title({'EVOLUTION OF HIT RATE WITH TIME';' ' ;'POLICY = LM';'CACHE SIZE = 100*1024'})


%AVERAGE TIME DIFFERENCE
hr_100_mean_diff_rand = sum( abs(hr_100_sort_rand - hr_100_shuffle_rand))/N
hr_100_mean_diff_LRU = sum( abs(hr_100_sort_LRU - hr_100_shuffle_LRU))/N
hr_100_mean_diff_LM = sum( abs(hr_100_sort_LM - hr_100_shuffle_LM))/N
%}



%{
%I2
%WORKLOAD = SHUFFLE
size_abs = [50 60 70 80 90 100];
%AVERAGE HIT RATE WITH SIZE
%POLICY = RANDOM
% HR(size) for the 6 cache sizes
hr_100_shuffle_rand_mean = sum(hr_100_shuffle_rand)/N;
hr_90_shuffle_rand_mean = sum(hr_90_shuffle_rand)/N;
hr_80_shuffle_rand_mean = sum(hr_80_shuffle_rand)/N;
hr_70_shuffle_rand_mean = sum(hr_70_shuffle_rand)/N;
hr_60_shuffle_rand_mean = sum(hr_60_shuffle_rand)/N;
hr_50_shuffle_rand_mean = sum(hr_50_shuffle_rand)/N;
hr_shuffle_rand_mean = [ hr_50_shuffle_rand_mean hr_60_shuffle_rand_mean hr_70_shuffle_rand_mean hr_80_shuffle_rand_mean hr_90_shuffle_rand_mean hr_100_shuffle_rand_mean];


figure
plot(size_abs,hr_shuffle_rand_mean)
xlabel('Cache size')
ylabel('Hit Rate')
title({'EVOLUTION OF AVERAGE HIT RATE WITH CACHE SIZE';' ' ;'WORKLOAD = SHUFFLE';'POLICY = RANDOM'})

%POLICY = LRU
% HR(size) for the 6 cache sizes
hr_100_shuffle_LRU_mean = sum(hr_100_shuffle_LRU)/N;
hr_90_shuffle_LRU_mean = sum(hr_90_shuffle_LRU)/N;
hr_80_shuffle_LRU_mean = sum(hr_80_shuffle_LRU)/N;
hr_70_shuffle_LRU_mean = sum(hr_70_shuffle_LRU)/N;
hr_60_shuffle_LRU_mean = sum(hr_60_shuffle_LRU)/N;
hr_50_shuffle_LRU_mean = sum(hr_50_shuffle_LRU)/N;
hr_shuffle_LRU_mean = [ hr_50_shuffle_LRU_mean hr_60_shuffle_LRU_mean hr_70_shuffle_LRU_mean hr_80_shuffle_LRU_mean hr_90_shuffle_LRU_mean hr_100_shuffle_LRU_mean];

figure
plot(size_abs,hr_shuffle_LRU_mean)
xlabel('Cache size')
ylabel('Hit Rate')
title({'EVOLUTION OF AVERAGE HIT RATE WITH CACHE SIZE';' ' ;'WORKLOAD = SHUFFLE';'POLICY = LRU'})

%POLICY = LRU-MIN
% HR(size) for the 6 cache sizes
hr_100_shuffle_LM_mean = sum(hr_100_shuffle_LM)/N;
hr_90_shuffle_LM_mean = sum(hr_90_shuffle_LM)/N;
hr_80_shuffle_LM_mean = sum(hr_80_shuffle_LM)/N;
hr_70_shuffle_LM_mean = sum(hr_70_shuffle_LM)/N;
hr_60_shuffle_LM_mean = sum(hr_60_shuffle_LM)/N;
hr_50_shuffle_LM_mean = sum(hr_50_shuffle_LM)/N;
hr_shuffle_LM_mean = [ hr_50_shuffle_LM_mean hr_60_shuffle_LM_mean hr_70_shuffle_LM_mean hr_80_shuffle_LM_mean hr_90_shuffle_LM_mean hr_100_shuffle_LM_mean];

figure
plot(size_abs,hr_shuffle_LM_mean)
xlabel('Cache size')
ylabel('Hit Rate')
title({'EVOLUTION OF AVERAGE HIT RATE WITH CACHE SIZE';' ' ;'WORKLOAD = SHUFFLE';'POLICY = LRU-MIN'})

figure
plot(size_abs, hr_shuffle_LM_mean,'r',size_abs, hr_shuffle_LRU_mean,'b',size_abs,hr_shuffle_rand_mean,'g')
legend('LRU-MIN','LRU','RANDOM','Location','northoutside','Orientation','horizontal')
xlabel('Cache size')
ylabel('Average Hit Rate')
title({'EVOLUTION OF AVERAGE HIT RATE WITH CACHE SIZE';' ' ;'WORKLOAD = SHUFFLE'})



%SIZE VARIANCE
%POLICY = RANDOM
hr_shuffle_rand_size_var = (sum((hr_shuffle_rand_mean - sum(hr_shuffle_rand_mean)/6).^2) )/6
%POLICY = LRU
hr_shuffle_LRU_size_var = sum(((hr_shuffle_LRU_mean - sum(hr_shuffle_LRU_mean)/6).^2) )/6
%POLICY = LRU-MIN
hr_shuffle_LM_size_var = sum(((hr_shuffle_LM_mean - sum(hr_shuffle_LM_mean)/6).^2) )/6


% TIME VARIANCE HIT RATE WITH SIZE
%SIZE=100
%WORKLOAD = SHUFFLE
hr_100_shuffle_LM_var = sum( (hr_100_shuffle_LM - (sum(hr_100_shuffle_LM)/N) ).^2)/N;
hr_100_shuffle_LRU_var = sum((hr_100_shuffle_LRU - (sum(hr_100_shuffle_LRU)/N) ).^2)/N;
hr_100_shuffle_rand_var = sum((hr_100_shuffle_rand - (sum(hr_100_shuffle_rand)/N) ).^2)/N;
hr_90_shuffle_LM_var = sum( (hr_90_shuffle_LM - (sum(hr_90_shuffle_LM)/N) ).^2)/N;
hr_90_shuffle_LRU_var = sum((hr_90_shuffle_LRU - (sum(hr_90_shuffle_LRU)/N) ).^2)/N;
hr_90_shuffle_rand_var = sum((hr_90_shuffle_rand - (sum(hr_90_shuffle_rand)/N) ).^2)/N;
hr_80_shuffle_LM_var = sum( (hr_80_shuffle_LM - (sum(hr_80_shuffle_LM)/N) ).^2)/N;
hr_80_shuffle_LRU_var = sum((hr_80_shuffle_LRU - (sum(hr_80_shuffle_LRU)/N) ).^2)/N;
hr_80_shuffle_rand_var = sum((hr_80_shuffle_rand - (sum(hr_80_shuffle_rand)/N) ).^2)/N;
hr_70_shuffle_LM_var = sum( (hr_70_shuffle_LM - (sum(hr_70_shuffle_LM)/N) ).^2)/N;
hr_70_shuffle_LRU_var = sum((hr_70_shuffle_LRU - (sum(hr_70_shuffle_LRU)/N) ).^2)/N;
hr_70_shuffle_rand_var = sum((hr_70_shuffle_rand - (sum(hr_70_shuffle_rand)/N) ).^2)/N;
hr_60_shuffle_LM_var = sum( (hr_60_shuffle_LM - (sum(hr_60_shuffle_LM)/N) ).^2)/N;
hr_60_shuffle_LRU_var = sum((hr_60_shuffle_LRU - (sum(hr_60_shuffle_LRU)/N) ).^2)/N;
hr_60_shuffle_rand_var = sum((hr_60_shuffle_rand - (sum(hr_60_shuffle_rand)/N) ).^2)/N;
hr_50_shuffle_LM_var = sum( (hr_50_shuffle_LM - (sum(hr_50_shuffle_LM)/N) ).^2)/N;
hr_50_shuffle_LRU_var = sum((hr_50_shuffle_LRU - (sum(hr_50_shuffle_LRU)/N) ).^2)/N;
hr_50_shuffle_rand_var = sum((hr_50_shuffle_rand - (sum(hr_50_shuffle_rand)/N) ).^2)/N;

hr_shuffle_LM_var = [hr_50_shuffle_LM_var hr_60_shuffle_LM_var hr_70_shuffle_LM_var hr_80_shuffle_LM_var hr_90_shuffle_LM_var hr_100_shuffle_LM_var];
hr_shuffle_LRU_var = [hr_50_shuffle_LRU_var hr_60_shuffle_LRU_var hr_70_shuffle_LRU_var hr_80_shuffle_LRU_var hr_90_shuffle_LRU_var hr_100_shuffle_LRU_var];
hr_shuffle_rand_var = [hr_50_shuffle_rand_var hr_60_shuffle_rand_var hr_70_shuffle_rand_var hr_80_shuffle_rand_var hr_90_shuffle_rand_var hr_100_shuffle_rand_var];

%POLICY = RANDOM
figure
plot(size_abs,hr_shuffle_rand_var)
xlabel('Cache size')
ylabel('Time var(Hit Rate)')
title({'EVOLUTION OF HIT RATE WITH CACHE SIZE';' ' ;'WORKLOAD = SHUFFLE';'POLICY = RANDOM'})

%POLICY = LRU
figure
plot(size_abs,hr_shuffle_LRU_var)
xlabel('Cache size')
ylabel('Time var(Hit Rate)')
title({'EVOLUTION OF HIT RATE WITH CACHE SIZE';' ' ;'WORKLOAD = SHUFFLE';'POLICY = LRU'})

%POLICY = LRU-MIN
figure
plot(size_abs,hr_shuffle_LM_var)
xlabel('Cache size')
ylabel('Time var(Hit Rate)')
title({'EVOLUTION OF HIT RATE WITH CACHE SIZE';' ' ;'WORKLOAD = SHUFFLE';'POLICY = LRU-MIN'})

figure
plot(size_abs, hr_shuffle_LM_var,'r',size_abs, hr_shuffle_LRU_var,'b',size_abs,hr_shuffle_rand_var,'g')
legend('LRU-MIN','LRU','RANDOM','Location','northoutside','Orientation','horizontal')
xlabel('Cache size')
ylabel('Hit Rate Time Variance')
title({'EVOLUTION OF HIT RATE TIME VARIANCE WITH CACHE SIZE';' ' ;'WORKLOAD = SHUFFLE'})

%WORKLOAD = SHUFFLE
%POLICY = RANDOM
% HR(n) for the 6 cache sizes
figure
plot(t,hr_100_shuffle_rand,'r',t,hr_90_shuffle_rand,'b',t,hr_80_shuffle_rand,'g',t,hr_70_shuffle_rand,'m*',t,hr_60_shuffle_rand,'c',t,hr_50_shuffle_rand,'y')
xlabel('Discrete time')
ylabel('Hit Rate')
legend('100','90','80','70','60','50','Location','northoutside','Orientation','horizontal')
title({'EVOLUTION OF HIT RATE WITH TIME';' ' ;'WORKLOAD = SHUFFLE';'POLICY = RANDOM'})

%WORKLOAD = SHUFFLE
%POLICY = RANDOM
% HR(n) for the 6 cache sizes
figure
plot(t,hr_100_shuffle_LRU,'r',t,hr_90_shuffle_LRU,'b',t,hr_80_shuffle_LRU,'g',t,hr_70_shuffle_LRU,'m*',t,hr_60_shuffle_LRU,'c',t,hr_50_shuffle_LRU,'y')
xlabel('Discrete time')
ylabel('Hit Rate')
legend('100','90','80','70','60','50','Location','northoutside','Orientation','horizontal')
title({'EVOLUTION OF HIT RATE WITH TIME';' ' ;'WORKLOAD = SHUFFLE';'POLICY = LRU'})

%WORKLOAD = SHUFFLE
%POLICY = RANDOM
% HR(n) for the 6 cache sizes
figure
plot(t,hr_100_shuffle_LM,'r',t,hr_90_shuffle_LM,'b',t,hr_80_shuffle_LM,'g',t,hr_70_shuffle_LM,'m*',t,hr_60_shuffle_LM,'c',t,hr_50_shuffle_LM,'y')
xlabel('Discrete time')
ylabel('Hit Rate')
legend('100','90','80','70','60','50','Location','northoutside','Orientation','horizontal')
title({'EVOLUTION OF HIT RATE WITH TIME';' ' ;'WORKLOAD = SHUFFLE';'POLICY = LRU-MIN'})

%}

%{
%EXECUTION TIME 
%SIZE=100
%WORKLOAD = SHUFFLE
% HR(n) for the 3 policies
t1 = t_nocache - t_100_shuffle_LM;
t2 = t_nocache - t_100_shuffle_LRU;
t3 = t_nocache-t_100_shuffle_rand;

plot(t,t_nocache - t_100_shuffle_LM,'r',t,t_nocache - t_100_shuffle_LRU,'b',t,t_nocache-t_100_shuffle_rand,'g')
xlabel('Discrete time')
ylabel('Service Execution Time(s)')
legend('LRU-MIN','LRU','RANDOM')
title({'EVOLUTION OF TIME SERVICE WITH TIME';' ' ;'WORKLOAD = SHUFFLE';'CACHE SIZE = 100*1024'})
%
%SIZE=100
%WORKLOAD = sort
% HR(n) for the 3 policies
figure
plot(t,t_nocache - t_100_sort_LM,'r',t,t_nocache-t_100_sort_LRU,'b',t,t_nocache-t_100_sort_rand,'g')
xlabel('Discrete time')
ylabel('Hit Rate')
legend('LRU-MIN','LRU','RANDOM')
title({'EVOLUTION OF TIME SERVICE WITH TIME';' ' ;'WORKLOAD = SORTED';'CACHE SIZE = 100*1024'})
%}


%SIZE=100
%POLICY = RANDOM
% HR(n) for the 2 workloads
figure
plot(t,t_100_sort_rand-t_nocache,'r',t,t_100_shuffle_rand-t_nocache,'b')
xlabel('Discrete time')
ylabel('Hit Rate')
legend('sort','shuffle')
title({'EVOLUTION OF TIME SERVICE WITH TIME';' ' ;'POLICY = RANDOM';'CACHE SIZE = 100*1024'})

%SIZE=100
%POLICY = LRU
% HR(n) for the 2 workloads
figure
plot(t,t_100_sort_LRU-t_nocache,'r',t,t_100_shuffle_LRU-t_nocache,'b')
xlabel('Discrete time')
ylabel('Hit Rate')
legend('sort','shuffle')
title({'EVOLUTION OF TIME SERVICE WITH TIME';' ' ;'POLICY = LRU';'CACHE SIZE = 100*1024'})

%SIZE=100
%POLICY = LRU-MIN
% HR(n) for the 2 workloads
figure
plot(t,t_100_sort_LM-t_nocache,'r',t,t_100_shuffle_LM-t_nocache,'b')
xlabel('Discrete time')
ylabel('Hit Rate')
legend('sort','shuffle')
title({'EVOLUTION OF TIME SERVICE WITH TIME';' ' ;'POLICY = LM';'CACHE SIZE = 100*1024'})

%WORKLOAD = SHUFFLE
%POLICY = RANDOM
% HR(n) for the 6 cache sizes
plot(t,t_100_shuffle_rand-t_nocache,'r',t,t_90_shuffle_rand-t_nocache,'b',t,t_80_shuffle_rand-t_nocache,'g',t,t_70_shuffle_rand-t_nocache,'m',t,t_60_shuffle_rand-t_nocache,'c',t,t_50_shuffle_rand-t_nocache,'y')
xlabel('Discrete time')
ylabel('Hit Rate')
legend('100','90','80','70','60','50')
title({'EVOLUTION OF TIME SERVICE WITH TIME';' ' ;'WORKLOAD = SHUFFLE';'POLICY = RANDOM'})

%WORKLOAD = SHUFFLE
%POLICY = RANDOM
% HR(n) for the 6 cache sizes
figure
plot(t,t_100_shuffle_LRU-t_nocache,'r',t,t_90_shuffle_LRU-t_nocache,'b',t,t_80_shuffle_LRU-t_nocache,'g',t,t_70_shuffle_LRU-t_nocache,'m',t,t_60_shuffle_LRU-t_nocache,'c',t,t_50_shuffle_LRU-t_nocache,'y')
xlabel('Discrete time')
ylabel('Hit Rate')
legend('100','90','80','70','60','50')
title({'EVOLUTION OF TIME SERVICE WITH TIME';' ' ;'WORKLOAD = SHUFFLE';'POLICY = LRU'})

%WORKLOAD = SHUFFLE
%POLICY = RANDOM
% HR(n) for the 6 cache sizes
figure
plot(t,t_100_shuffle_LM-t_nocache,'r',t,t_90_shuffle_LM-t_nocache,'b',t,t_80_shuffle_LM-t_nocache,'g',t,t_70_shuffle_LM-t_nocache,'m',t,t_60_shuffle_LM-t_nocache,'c',t,t_50_shuffle_LM-t_nocache,'y')
xlabel('Discrete time')
ylabel('Hit Rate')
legend('100','90','80','70','60','50')
title({'EVOLUTION OF TIME SERVICE WITH TIME';' ' ;'WORKLOAD = SHUFFLE';'POLICY = LRU-MIN'})
%}


