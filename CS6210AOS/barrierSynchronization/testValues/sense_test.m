close all
clear all
clc

%CENTRALIZED BARRIER
num_threads = csvread('sense_test.csv',2,0,[2 0 5 0]);
SGAT = csvread('sense_test.csv',2,1,[2 1 5 1]);
SGACT = csvread('sense_test.csv',2,2,[2 2 5 2]);
SGCCT = csvread('sense_test.csv',2,3,[2 3 5 3]);
SGSC = csvread('sense_test.csv',2,4,[2 4 5 4]);
SEBGSP = csvread('sense_test.csv',2,5,[2 5 5 5]);
SEBGAT = csvread('sense_test.csv',2,6,[2 6 5 6]);
SGBGSP = csvread('sense_test.csv',2,7,[2 7 5 7]);
SGBGAT = csvread('sense_test.csv',2,8,[2 8 5 8]);
SLBGSP = csvread('sense_test.csv',2,9,[2 9 5 9]);
SLBGAT = csvread('sense_test.csv',2,10,[2 10 5 10]);

%TREE SENSE BARRIER
num_threads_t = csvread('test_tree1.csv',2,0,[2 0 8 0]);
TGAT = csvread('test_tree1.csv',2,1,[2 1 8 1]);


%MCS BARRIER DATA
num_threads_mcs = csvread('mcs_test.csv',0,1,[0 1 2 1]);
MCSGAT = csvread('mcs_test.csv',0,2,[0 2 2 2]);
MCSAT = csvread('mcs_test.csv',0,3,[0 3 2 3]);
MCSWT = csvread('mcs_test.csv',0,4,[0 4 2 4]);

%TOURNAMENT BARRIER DATA
num_threads_to = csvread('tour_test.csv',0,1,[0 1 5 1]);
TOGAT = csvread('tour_test.csv',0,2,[0 2 5 2]);
TOAT = csvread('tour_test.csv',0,3,[0 3 5 3]);
TOWT = csvread('tour_test.csv',0,4,[0 4 5 4]);

%COMBINED BARRIER DATA
mat_data = csvread('stour_test.csv',0,0,[0 0 4 6])
for i=size(mat_data,2):-1:2
    mat_data(1,i) = mat_data(1,i-1);
end
K = size(mat_data,2);
tmp = mat_data(1,2:K);
figure
plot(tmp,mat_data(2,2:K),'b',tmp,mat_data(3,2:K),'g',tmp,mat_data(4,2:K),'r',tmp,mat_data(5,2:K),'c')
title('Global time ahchievement')
xlabel('Number of threads per process')
ylabel('Time in us')
legend('P=2','P=4','P=8','P=16')

%GAT MCS TOUR
%{
figure
tmp = linspace(1,140,140);
plot(num_threads_mcs,MCSGAT, 'b',num_threads_to,TOGAT, 'g' );
title('Global time ahchievement')
xlabel('Number of processes')
ylabel('Time in us')
legend('MCS barrier','tournament barrier')
%}
%{
%MCS
figure
plot(num_threads_mcs,MCSGAT, 'r:*', num_threads_mcs,MCSAT, 'b*:', num_threads_mcs,MCSWT,'g:*');

%TOURNAMENT 
figure
plot(num_threads_to, TOGAT, 'r:*', num_threads_to,TOAT, 'b*:', num_threads_to,TOWT,'g:*');
%}
%{
%MCS TOUR
%difference of arrival time
figure
tmp1 = linspace(1,70,70);
res1 = interp1(num_threads_mcs,MCSAT,tmp1);
res2 = interp1(num_threads_to,TOAT,tmp1);
res3 = res2-res1;
res4 = interp1(num_threads_mcs,MCSWT,tmp1);
res5 = interp1(num_threads_to,TOWT,tmp1);
res6 = res5-res4;
plot(tmp1,res3,'b',tmp1,res6,'g:');
title('diffrence arrivail/wake-up time : tournament - MCS')
legend('arrival time','wake-up time')
xlabel('number of processes')
ylabel('time us')
%}


%{
%graph global time achievement GAT for MP barriers
plot(num_threads,SGAT,'b-',num_threads_t,TGAT,'g-')
title('Global time ahchievement')
xlabel('Number of threads')
ylabel('Time in us')
legend('centralized barrier','tree barrier')
%}

%{
figure
%graph global time achievement GAT with spin backoff sense only
plot(num_threads,SGAT,'b:', num_threads, SEBGAT,':v',num_threads,SGBGAT,'g:o',num_threads,SLBGAT,'r:*')
title('Global time ahchievement : spin with backoff')
xlabel('Number of threads')
ylabel('Time (us)')
legend('no backoff','exponential','geometrical', 'linear')
axis([2,8,0,0.002])
%}
%{
figure
%graph global spin count with spin backoff sense only
plot(num_threads,SGSC,'b:', num_threads, SEBGSP,':v',num_threads,SGBGSP,'g:o',num_threads,SLBGSP,'r:*')
title('Spin count per barrier')
xlabel('Number of threads')
ylabel('Count')
legend('no backoff','exponential','geometrical', 'linear')
%}

%{
figure
%graph global spin count with spin backoff sense only zoom
plot(num_threads,SGSC,'b:', num_threads, SEBGSP,':v',num_threads,SGBGSP,'g:o',num_threads,SLBGSP,'r:*')
title('Spin count per barrier')
xlabel('Number of threads')
ylabel('Count')
legend('no backoff','exponential','geometrical', 'linear')
axis([2,8,0,15])
%}
%{
%TREE VARIANCE 
statis = csvread('test_tree5.csv',164,1,[164 1 187 8]);
average = sum(statis,1)/size(statis,1);
tmp = statis;
for j=1:size(statis,2)
    tmp(:,j) = (tmp(:,j)-average(j)).^2;
end
tmp
variance = sum(tmp,1)
variance./average
 %}   
