clear all, close all, clc

%add path to the robotic toolbox to compare with matlab model
addpath(genpath('/home/abenbihi/matlab/rvctools')); 
ROOT_DIR = '/home/abenbihi/gtCourses/CS8903/';

% Specify recorded dataset 
rawSourceData = 1; %0:false, 1:true
numSet = '2_5';

% Specify tests to run, set to 1 to run the test
testEE=0; %Plot the recorded and computed ee cartesian trajectories
testQ=0; % Plot the computed joint trajectories
compareWithMatlabModel=1; % Plot the ee cartesian trajectories of matlab fk on computed joint trajectories 
testError=1; % Plot the cartesian position errors on the x,y and z axis (between recorded data and ikfast fk on ikfast ik result)
testSimulation=0; % Plot jaco2 motion based on the robotic toolbox model

% Specify the data results to run the test on
testAll=0; %all raw sollutions found by ikfast
testJointConstraints=0; % all constrained solutions found by ikfast
testNN=0; % path planning with nearest neighbor strategy
testGraph=1; % path planning with graph strategy
testCut=0; %
testCombined=0; % executable solution after joint constrain and graph path planning

if rawSourceData==0 %x,y,z, eulerX, eulerY, eulerZ
    robotFilename = strcat(ROOT_DIR, 'data/processedData/dataset', numSet, '.txt');
    ee_begin = 1;
    ee_end = 3;
    theta_begin = 4;
    theta_end = 6;
elseif rawSourceData==1 % j1,j2,j3,j4,j5,j6,x,y,z,eulerX, eulerY, eulerZ
    robotFilename = strcat(ROOT_DIR, 'data/rawData/dataset',numSet,'.txt');
    q_begin = 2; % joint indices
    q_end = 7;
    ee_begin = 8; % end effector cartesian position indices
    ee_end = 10;
    theta_begin = 11; %end effector euler angles indices
    theta_end = 13;
end

if testAll==1
    solutionDirName = 'testAll/data/';
    eeTitle = 'Ik test on all solutions: ee trajectory';
    qTitle = 'Ik test on all solutions: q trajectory';
    errorTitle = 'Ik test on all solutions: ee error';
elseif testJointConstraints==1
    solutionDirName = 'testJointConstraints/data/';
    eeTitle = 'Ik test on joint constrained solutions: ee trajectory';
    qTitle = 'Ik test on joint constrained solutions: q trajectory';
    errorTitle = 'Ik test on joint constrained solutions: ee error';
elseif testNN==1
    solutionDirName = 'testNearestNeighbor/data/';
    eeTitle = 'Ik test nearest neighbor processing: ee trajectory';
    qTitle = 'Ik test nearest neighbor processing: q trajectory';
    errorTitle = 'Ik test nearest neighbor processing: ee error';
elseif testGraph==1
    solutionDirName = 'testGraph/data/';
    eeTitle = 'Ik test graph processing: ee trajectory';
    qTitle = 'Ik test graph processing: q trajectory';
    errorTitle = 'Ik test graph processing: ee error';
elseif testCut==1
    solutionDirName = 'testCut/data/';
    eeTitle = 'Ik test cut processing: ee trajectory';
    qTitle = 'Ik test cut processing: q trajectory';
    errorTitle = 'Ik test cut processing: ee error';
elseif testCombined==1
    solutionDirName = 'data/';
    eeTitle = 'Ik test joint constrained and graph processing: ee trajectory';
    qTitle = 'Ik test joint constrained and graph processing: q trajectory';
    errorTitle = 'Ik test joint constrained and graph processing: ee error';
end

% Import recorded data
recordedData = importdata(robotFilename); 

T_recorded = zeros(4,4,size(recordedData,1));
for k=1:size(recordedData,1)
    T_recorded(:,4,k) = [recordedData(k,ee_begin:ee_end) 1]';
    T_recorded(1:3,1:3,k) = eul2r(recordedData(k,theta_begin:theta_end));
end

% Imported computed data
q_ikfastFilename = strcat(solutionDirName, 'q_ikfast',numSet,'.txt')
T_ikfastFilename = strcat(solutionDirName, 't_ikfast',numSet,'.txt');
q_ikfast = importdata(q_ikfastFilename);
T_ikfast = importdata(T_ikfastFilename);

% Plot
pathLength = min(size(recordedData,1), size(T_ikfast,1));
begin_plot = 1;
end_plot = pathLength;

% Plot recorded ee (GREEN), ikfast-only ee (IK+FK by ikfast - RED),
% ikfast-matlab ee (IK by ikfast, FK by matlab - BLUE)
if testEE==1
    figure
    hold on
    plot3(squeeze(recordedData(begin_plot:end_plot,ee_begin)), squeeze(recordedData(begin_plot:end_plot,ee_begin+1)), squeeze(recordedData(begin_plot:end_plot,ee_begin+2)), 'g+');
    plot3(squeeze(T_ikfast(:,4)), squeeze(T_ikfast(:,8)), squeeze(T_ikfast(:,12)), 'r.');
    legend('Reference data', 'IK+FK ikfast');
    if compareWithMatlabModel==1
        startup_rvc
        mdl_jaco2
        T_matlab_on_ikfast = zeros(4,4,size(q_ikfast,1));
        for k=1:size(q_ikfast,1)
            T_matlab_on_ikfast(:,:,k) = jaco2.fkine(q_ikfast(k,:));
        end
        plot3(squeeze(T_matlab_on_ikfast(1,4,:)), squeeze(T_matlab_on_ikfast(2,4,:)), squeeze(T_matlab_on_ikfast(3,4,:)), 'b.');
        legend('Reference data', 'IK+FK ikfast', 'IK(ikfast)+FK(matlab)')
    end
    title(eeTitle);
end

% Plot the arm movement with the matlab model using the ikfast solution
if testSimulation==1
    startup_rvc
    mdl_jaco2
    for k=1:size(q_ikfast,1)
        jaco2.plot(q_ikfast(k,:));
    end 
end

% Plot the ikfast joint solution variation
if testQ==1
    figure
    hold on
    x = [1:size(q_ikfast,1)];
    plot(x, squeeze(q_ikfast(:,1)), 'y.', x, squeeze(q_ikfast(:,2)), 'c.',x, squeeze(q_ikfast(:,3)), 'm.',x, squeeze(q_ikfast(:,4)), 'r.',x, squeeze(q_ikfast(:,5)), 'b.',x, squeeze(q_ikfast(:,6)), 'g.');
    title(qTitle);
    legend('q1', 'q2', 'q3', 'q4', 'q5', 'q6')
end


if testError==1
    error = zeros(1,pathLength); 
    X_ref = squeeze(recordedData(begin_plot:end_plot,8));
    Y_ref = squeeze(recordedData(begin_plot:end_plot,9));
    Z_ref = squeeze(recordedData(begin_plot:end_plot,10));
    X_ikf = squeeze(T_ikfast(begin_plot:end_plot,4));
    Y_ikf = squeeze(T_ikfast(begin_plot:end_plot,8));
    Z_ikf = squeeze(T_ikfast(begin_plot:end_plot,12));
    
    x_err = abs(X_ref-X_ikf);
    y_err = abs(Y_ref-Y_ikf);
    z_err = abs(Z_ref-Z_ikf);
    figure 
    hold on
    plot([1:size(q_ikfast,1)], x_err, [1:size(q_ikfast,1)], y_err, [1:size(q_ikfast,1)],z_err);
    legend('X error', 'Y error', 'Z error') 
    title(strcat(errorTitle, '. Ikfast ee VS recorded ee'));
    
    if compareWithMatlabModel==1
        startup_rvc
        mdl_jaco2
        T_matlab_on_ikfast = zeros(4,4,size(q_ikfast,1));
        for k=1:size(q_ikfast,1)
            T_matlab_on_ikfast(:,:,k) = jaco2.fkine(q_ikfast(k,:));
        end
        X_m = squeeze(T_matlab_on_ikfast(1,4,begin_plot:end_plot));
        Y_m = squeeze(T_matlab_on_ikfast(2,4,begin_plot:end_plot));
        Z_m = squeeze(T_matlab_on_ikfast(3,4,begin_plot:end_plot));
        x_err = abs(X_m-X_ikf);
        y_err = abs(Y_m-Y_ikf);
        z_err = abs(Z_m-Z_ikf);
        
        figure 
        hold on
        plot([1:size(q_ikfast,1)], x_err, [1:size(q_ikfast,1)], y_err, [1:size(q_ikfast,1)],z_err);
        legend('X error', 'Y error', 'Z error') 
        title(strcat(errorTitle, '. Ikfast ee VS matlab ee')); %matab ee is computed with matlab fk on ikfast joint solution
    end
    
end


