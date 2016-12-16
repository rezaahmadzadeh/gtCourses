clc,%clear,close all
addpath(genpath('./'));             % Add path to functions and data
%% 1 - preprocessing the data (smooth, fit, cut, shift, ...)
% ------------------------------------------------------------
numSet      =  1;                   % which file to load the data from
numPoints   =  1500;                % number points desired in final trajectory
span        = 0.25;                 % affects the smoothness of the data
tol         = 0.015;                % the lowest variation desired in initial and final points
plotIt      = false;                % plot internal data (for debugging purposes)
[smoothData, numDemos]  = parseTrajectory(numSet, tol, span, numPoints, plotIt);

dataDir = '/home/abenbihi/gtCourses/CS8903/data/processedData/';
set = 4;

for i=1:numDemos
    filename = strcat(dataDir, 'dataset', num2str(numSet),'_',num2str(i),'p.txt');
    dlmwrite(filename,smoothData(i));
end
