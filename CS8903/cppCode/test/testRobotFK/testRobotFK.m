clear all
close all
clc

startup_rvc
mdl_jaco2
deg = pi/180;

% 1 line number 
% 2-7 joint positions in degrees with respect to the base frame
% 8 9 10 end effector position with respect to the base frame
% 11 12 13 end effector orientation with respect to the base frame, in radians (euler angles)

% Specify experimental data
numSet = '3_6';
solutionDirName = '/home/abenbihi/gtCourses/CS8903/cppCode/test/testRobotFK/data/';
T_fkfastFilename = strcat(solutionDirName, 't_ikfast',numSet,'.txt'); 


% Import source data and data index
robotFilename = strcat('/home/abenbihi/gtCourses/CS8903/data/rawData/dataset',numSet,'.txt');
D = importdata(robotFilename);
q_begin = 2; % joint indices
q_end = 7;
ee_begin = 8; % end effector cartesian position indices
ee_end = 10;
theta_begin = 11; %end effector euler angles indices
theta_end = 13;

% Build arguments for matlab: transformation frame matrix
pathLength = size(D,1);
T_robot = zeros(4,4,pathLength);
q_robot = zeros(pathLength,6);
T_matlab = zeros(4,4,pathLength);
q_matlab = zeros(pathLength,6);

% Completion
for k=1:pathLength
    T_robot(:,4,k) = [D(k,ee_begin:ee_end) 1]'; % recorded end effector position
    T_robot(1:3,1:3,k) = eul2r(D(k,theta_begin:theta_end)); %recorded rotation matrix
    q_robot(k,:) = D(k,q_begin:q_end); % recorded joint position
    T_matlab(:,:,k) = jaco2.fkine(q_robot(k,:)*deg); % matlab fk on recorded joint configuration
end
T_fkfast = importdata(T_fkfastFilename); % ikfast fk on recorded joint configuration


figure
hold on
plot3(squeeze(D(:,ee_begin)), squeeze(D(:,ee_begin+1)), squeeze(D(:,ee_begin+2)), 'g+');
plot3(squeeze(T_matlab(1,4,:)), squeeze(T_matlab(2,4,:)), squeeze(T_matlab(3,4,:)), 'b.');
plot3(squeeze(T_fkfast(:,4)), squeeze(T_fkfast(:,8)), squeeze(T_fkfast(:,12)), 'r.');
xlabel('X')
ylabel('Y')
zlabel('Z')

% partial plot
begin_plot = 1;
end_plot = pathLength;

% 2d plots XY
%{
figure 
hold on
plot(squeeze(D(begin_plot:end_plot,8)), squeeze(D(begin_plot:end_plot,9)), 'g.');
plot(squeeze(T_matlab(1,4,begin_plot:end_plot)), squeeze(T_matlab(2,4,begin_plot:end_plot)), 'b.');
plot(squeeze(T_fkfast(begin_plot:end_plot,4)), squeeze(T_fkfast(begin_plot:end_plot,8)), 'r.');
xlabel('X')
ylabel('Y')

% 2d plots XZ
figure 
hold on
plot(squeeze(D(begin_plot:end_plot,8)), squeeze(D(begin_plot:end_plot,10)), 'g.');
plot(squeeze(T_matlab(1,4,begin_plot:end_plot)), squeeze(T_matlab(3,4,begin_plot:end_plot)), 'b.');
plot(squeeze(T_fkfast(begin_plot:end_plot,4)), squeeze(T_fkfast(begin_plot:end_plot,12)), 'r.');
xlabel('X')
ylabel('Z')

% 2d plots YZ
figure 
hold on
plot(squeeze(D(begin_plot:end_plot,9)), squeeze(D(begin_plot:end_plot,10)), 'g.');
plot(squeeze(T_matlab(2,4,begin_plot:end_plot)), squeeze(T_matlab(3,4,begin_plot:end_plot)), 'b.');
plot(squeeze(T_fkfast(begin_plot:end_plot,8)), squeeze(T_fkfast(begin_plot:end_plot,12)), 'r.');
xlabel('Y')
ylabel('Z')
%}

% Error analysis
X_ref = squeeze(D(begin_plot:end_plot,8));
Y_ref = squeeze(D(begin_plot:end_plot,9));
Z_ref = squeeze(D(begin_plot:end_plot,10));
X_ikf = squeeze(T_fkfast(begin_plot:end_plot,4));
Y_ikf = squeeze(T_fkfast(begin_plot:end_plot,8));
Z_ikf = squeeze(T_fkfast(begin_plot:end_plot,12));
X_m = squeeze(T_matlab(1,4,begin_plot:end_plot));
Y_m = squeeze(T_matlab(2,4,begin_plot:end_plot));
Z_m = squeeze(T_matlab(3,4,begin_plot:end_plot));
x_err = abs(X_ref-X_ikf);
y_err = abs(Y_ref-Y_ikf);
z_err = abs(Z_ref-Z_ikf);
figure 
hold on
plot([begin_plot:end_plot], x_err, [begin_plot:end_plot], y_err, [begin_plot:end_plot], z_err); 
legend('X error', 'Y error', 'Z error')  


