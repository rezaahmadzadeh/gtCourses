function [smoothData,numDemos] = parseTrajectory(numSet, tol, span, numPoint, plotIt)
% -----------------------------------------------------------------------
% A function to analyze raw data and output smooth data
%
% Inputs:
%
%   numSet:     integer used to label the plot
%   endPoint:   point to which all trajectories will be shifted, false if
%               shifting not desired
%   tol:        amount of variation desired in beginning and end of trajectories
%   span:       input to the smoothing function, degree of smoothness
%   numPoint:   number of points desired in each smoothed trajectories,
%               trajectories will be resampled to achieve this
%
% Output:
%   smoothData: cell with each element point_numx3 vector of smoothed data
%
% -----------------------------------------------------------------------
% Code: Roshni Kaushik & Reza Ahmadzadeh (IRIM-2016)
% -----------------------------------------------------------------------

% ----- Get the x,y,z trajectories -----
[rawData, endPoint] = getRawTrajectories(numSet);

% ----- Cut data -----
cutData = cutTrajectories(rawData, endPoint, tol);

% ----- Resample Data -----
fitData = resampleTrajectories(cutData,numPoint);

% ----- Smooth data -----
smoothData = smoothTrajectories(fitData, span);
numDemos = numel(smoothData);
% ----- Plots -----
if plotIt
    plotData(fitData, sprintf('Raw Data for Set %i', numSet));
    plotData(smoothData, sprintf('Smoothed Data for Set %i', numSet));
end
end

function [rawData, endPoint] = getRawTrajectories(numSet)
%Load variables from the raw text-files converted in cell
if numSet == 1
    load dataset1;
    fprintf('> dataset%i loaded. A reaching skill from top\n',numSet);
    rawCells = {dataset1_1, dataset1_3, dataset1_6, dataset1_7};
    endPoint = [Inf,Inf,Inf]; % instead of [0,0,0]; to get rid of the shift to origin
elseif numSet == 2
    load dataset2;
    fprintf('> dataset%i loaded. A picking skill (needle threading)\n',numSet);
    rawCells = {dataset2_4, dataset2_5, dataset2_6, dataset2_7};
    endPoint = [Inf,Inf,Inf];
elseif numSet == 3
    load dataset3;
    fprintf('> dataset%i loaded. A reaching skill\n',numSet);
    rawCells = {dataset3_1, dataset3_4, dataset3_5, dataset3_6, dataset3_7};
    endPoint = [0,0,0];
elseif numSet == 4
    load dataset4;
    fprintf('> dataset%i loaded. A collision avoidance\n',numSet);
    %raw_vars = {dataset4_4, dataset4_5, dataset4_6};
    rawCells = {dataset4_3, dataset4_4, dataset4_5, dataset4_6, dataset4_7};
    endPoint = [Inf,Inf,Inf];
elseif numSet == 5
    load dataset4;
    fprintf('> dataset%i loaded. A collision avoidance (with less demonstrations)\n',numSet);
    rawCells = {dataset4_4, dataset4_5, dataset4_6};
    %     raw_vars = {dataset4_3, dataset4_4, dataset4_5, dataset4_6, dataset4_7};
    endPoint = [Inf,Inf,Inf];
elseif numSet == 6
    load dataset6;
    fprintf('> dataset%i loaded. A simple motion\n',numSet);
    disp('this dataset is a simple 3 demonstration set.');
    disp('this is count as the first refinement case.');
    fprintf('you should consider using dataset number%i after this.',numSet+1);
    rawCells = {dataset6_1, dataset6_2, dataset6_3};
    endPoint = [Inf,Inf,Inf];
elseif numSet == 7
    load dataset7;
    fprintf('> dataset%i loaded. A simple motion (after refinement)\n',numSet);
    fprintf('this dataset is a deformed version of dataset%i', numSet-1);
    rawCells = {dataset7_1, dataset7_2, dataset7_3};
    endPoint = [Inf,Inf,Inf];
elseif numSet == 8
    load dataset8;
    fprintf('> dataset%i loaded. first set of demos for ICRA-17\n',numSet);
    rawCells = {demo1, demo2, demo3, demo4};
    endPoint = [Inf,Inf,Inf];
elseif numSet == 9
    load dataset1_icra;
    fprintf('> dataset%i loaded. second set of demos for ICRA-17\n',numSet);
    %     raw_vars = {demo1, demo2, demo3, demo4, demo5, demo6, demo7, demo8, demo9};
    rawCells = {demo6, demo7, demo8, demo9}; % demo4, demo3, demo2, demo1,
    endPoint = [0,0,0];
elseif numSet == 10
    load dataset1_icra_refined_cut
    fprintf('> dataset%i loaded.\n',numSet);
    warning('> this just loads the refined trajectories for icra (no canal should be made)');
    rawCells = {repro5, repro5refined, refined_repro1 };
    endPoint = [0,0,0];
elseif numSet == 11
    load dataset2_icra;
    fprintf('> dataset%i loaded. third set of demos for ICRA-17\n',numSet);
    disp('> this is a simple curvy movement');
    rawCells = {D1, D2, D3};
    endPoint = [Inf,Inf,Inf];
elseif numSet == 12
    load dataset3_icra;
    fprintf('> dataset%i loaded. forth set of demos for ICRA-17\n',numSet);
    fprintf('> this is a simple curvy movement wider than dataset%i',numSet-1);
    rawCells = {D1, D2, D3};
    endPoint = [Inf,Inf,Inf];
elseif numSet == 13
    load dataset3_icra_refined; %dataset_icra3_repro.mat; %;
    fprintf('> dataset%i loaded. refined version of dataset%i\n',numSet,numSet-1);
    rawCells = {D1, D2, RD3}; % {R1,R2,R3}; %
    endPoint = [Inf,Inf,Inf];
else
    fprintf('Error Occurred - Dataset %i not available', numSet);
    return;
end

s = size(rawCells,2);
rawData = cell(1,s);                            %Initialize vectors

for ind1 = 1:s                                  %Store the x,y,z position for each trajectory
    %rawData{ind1} = rawCells{ind1}(:, 8:10);    % the convention for recorder I wrote for Jaco
    rawData{ind1} = rawCells{ind1}(:, 8:13); %Store the x,y,z position and the ee orientation euler angle (XYZ) for each trajectory
end
end

function cutData = cutTrajectories(rawData, endPoint, tol)
cutData = cell(size(rawData));
for ind1 = 1:size(rawData,2)
    traj = rawData{ind1};
    ss = size(traj,1);
    traj = traj(sum(   abs(traj(:,1:3) - repmat(traj(1,1:3), ss, 1)) > repmat(tol, ss, 3), 2  )   > 0, :);
    ss = size(traj,1);
    traj = traj(sum(abs(traj(:,1:3) - repmat(traj(end,1:3), ss, 1)) > repmat(tol, ss, 3), 2) > 0, :);
    
    % Shift data to the given endPoint
    if ~all(isinf(endPoint))            % if not (all the elements of the endPoint are Inf (i.e. endPoint ~= [Inf Inf Inf]))
        cutData{ind1}(:,1:3) = cutData{ind1}(:,1:3) - repmat(traj(end,1:3)-endPoint,size(traj,1),1);
    else
        cutData{ind1} = traj;
    end
end
end

function resampledData = resampleTrajectories(data,numPoint)
resampledData = cell(size(data));
for ind1 = 1:size(data,2)
    traj = data{1,ind1};
    n = size(traj,1);
    s = linspace(1,n,numPoint);
    resampledTraj = spline(1:n,traj.',s);
    resampledData{ind1} = resampledTraj.';
end
end


function smoothData = smoothTrajectories(data, span)
smoothData = cell(size(data));

for ind1 = 1:size(data,2)
    traj = data{ind1};
    smoothTraj = zeros(size(traj));
    for ind2 = 1:6
        smoothTraj(:,ind2) = smooth(traj(:,ind2), span);
    end
    smoothData{ind1} = smoothTraj;
end
end

function plotData(data, plot_title)
figure;
if isempty(plot_title)
    subplot(3,1,1); hold on; title('Demonstrations');
else
    subplot(3,1,1); hold on;
    title(plot_title);
end
subplot(3,1,1);hold on;ylabel('x_1');
subplot(3,1,2);hold on;ylabel('x_2');
subplot(3,1,3);hold on;ylabel('x_3');xlabel('num points');

legendVec = cell(size(data));
for ind1 = 1:size(data,2)
    s = (1:size(data{ind1},1)).';
    subplot(3,1,1);plot(s, data{ind1}(:,1),'linewidth', 1.2);
    subplot(3,1,2);plot(s, data{ind1}(:,2),'linewidth', 1.2);
    subplot(3,1,3);plot(s, data{ind1}(:,3),'linewidth', 1.2);
    legendVec{ind1} = sprintf('Demo %i', ind1);
end
legend(legendVec);
subplot(3,1,1); hold off; axis tight;
subplot(3,1,2); hold off; axis tight;
subplot(3,1,3); hold off; axis tight;

h = get(gcf,'children');
n = numel(h);
for ii=1:n
    h(ii).FontName = 'Times';
    h(ii).FontSize = 12;
end

figure;hold on;
for ind1 = 1:size(data,2)
    plot3(data{ind1}(:, 1), data{ind1}(:,2), data{ind1}(:,3),'linewidth', 1.5);
end
if isempty(plot_title)
    title('Demonstrations');
else
    title(plot_title);
end
legend(legendVec);
xlabel('x_1'); ylabel('x_2'); zlabel('x_3');
grid on;box on;
axis equal;

h = get(gcf,'children');
n = numel(h);
for ii=1:n
    h(ii).FontName = 'Times';
    h(ii).FontSize = 12;
end


end