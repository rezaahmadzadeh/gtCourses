[rawData, endPoint] = getRawTrajectories(numSet); %% ----- Get the x,y,z trajectories -----
cutData = cutTrajectories(rawData, endPoint, tol); % ----- Cut data -----
fitData = resampleTrajectories(cutData,numPoint); % ----- Resample Data -----
smoothData = smoothTrajectories(fitData, span); % ----- Smooth data -----
numDemos = numel(smoothData);

numSet      =  1;                   % which file to load the data from
numPoints   =  1500;                % number points desired in final trajectory
span        = 0.25;                 % affects the smoothness of the data
tol         = 0.015; 

[rawData, endPoint] = getRawTrajectories(1)
	Load matlab dataset containing 7 trajectories
	rawCell <-- Trajectories 1,3,6,7 (array of trajectories)
	endPoint = (0,0,0);
	rawData <-- End effector cartesian position trajectories for 1,3,6,7
	
cutData = cutTrajectories(rawData, endPoint, tol)
	for each trajectory i
		repmat(traj(1,:), ss, 1) = [traj(1,:) ... traj(1,:)]'
		B <-- abs(traj - repmat(traj(1,:), ss, 1)) = [|traj(i,:)-traj(1,:)|] 
		C <-- repmat(tol, ss, 3) = [ (tol,tol,tol) ... (tol,tol,tol)]' 
		|B-C| <-- abs(traj - repmat(traj(1,:), ss, 1))>repmat(tol, ss, 3) 
		% 1 if |traj(i,:)-traj(1,:)|<tol i.e. Keep it
		% 0 if |traj(i,:)-traj(1,:)|>tol i.e. Toss it
		NN <-- sum(|B-C|) i.e. number of points in tol radius i.e. number of points we keep
		

resampledData = resampleTrajectories(data,numPoint)



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tuto matlab c++
Note:   To run MATLAB engine on the UNIX platform, you must have the C shell csh installed at /bin/csh.
sudo apt-get install csh

Build Linux Engine Application
	1. Option 1: Inside matlab
		mex -v -client engine engdemo.c
		
	2. With a makefile (cf example)

Set Run-Time Library Path on Linux Systems
	LD_LIBRARY_PATH=/usr/local/MATLAB/R2016a/bin/glnxa64:/usr/local/MATLAB/R2016a/sys/os/glnxa64:$LD_LIBRARY_PATH
	export LD_LIBRARY_PATH
	PATH=/usr/local/MATLAB/R2016a/bin:$PATH

Error:
	1. Can't start matlab engine
	Solution: Check that you installed csh in /bin/.




	

