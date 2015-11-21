
clear all
close all
clc
clear

%addpath C:\Users\Pradeep\Documents\MATLAB\Multi-UAV_mission_Ver_4\track
addpath D:\MATLAB\UAVVer4\track

date_and_time = clock;

%diary(strcat('D:\MATLAB\UAVVer4\logs\',...
%    'simulation_logs',num2str(date_and_time(3)), num2str(date_and_time(4)), num2str(date_and_time(5)) ,'.txt'))

%diary on
num_of_UAVs = 8; % number of UAVs in the mission

time = 1200; % simulation time in seconds

%=======  initial states of the targets
initial_state(1).x = 4012+100*1/sqrt(2);
initial_state(1).y = -2550-100*1/sqrt(2);
initial_state(1).speed = 8;
initial_state(1).group = 1;

initial_state(2).x = 4012+100*2/sqrt(2);
initial_state(2).y = -2550-100*2/sqrt(2);
initial_state(2).speed = 8;
initial_state(2).group = 1;

initial_state(3).x = 4012+100*3/sqrt(2);
initial_state(3).y = -2550-100*3/sqrt(2);
initial_state(3).speed = 8;
initial_state(3).group = 1;

initial_state(4).x = 4012+100*4/sqrt(2);
initial_state(4).y = -2550-100*4/sqrt(2);
initial_state(4).speed = 8;
initial_state(4).group = 1;

initial_state(5).x = 4012+100*5/sqrt(2);
initial_state(5).y =  -2550-100*5/sqrt(2);
initial_state(5).speed = 8;
initial_state(5).group = 1;

initial_state(6).x = 4012+100*3/sqrt(2);
initial_state(6).y =  -2650-100*3/sqrt(2);
initial_state(6).speed = 8;
initial_state(6).group = 2;

initial_state(7).x = 4012+100*4/sqrt(2);
initial_state(7).y = -2650-100*4/sqrt(2);
initial_state(7).speed = 8;
initial_state(7).group = 2;

initial_state(8).x = 4012+100*5/sqrt(2);
initial_state(8).y =  -2650-100*5/sqrt(2);
initial_state(8).speed = 8;
initial_state(8).group = 2;

initial_state(9).x = 4012+100*1/sqrt(2);
initial_state(9).y = -2650-100*1/sqrt(2);
initial_state(9).speed = 8;
initial_state(9).group = 2;

initial_state(10).x = 4012+100*2/sqrt(2);
initial_state(10).y =  -2650-100*2/sqrt(2);
initial_state(10).speed = 8;
initial_state(10).group = 2;

% initial_state(11).x = 4012+100*3/sqrt(2);
% initial_state(11).y =  -2550-100*3/sqrt(2);
% initial_state(11).speed = 8;
% initial_state(11).group = 3;
% 
% initial_state(12).x = 4012+100*4/sqrt(2);
% initial_state(12).y =  -2550-100*4/sqrt(2);
% initial_state(12).speed = 8;
% initial_state(12).group = 3;
% 
% initial_state(13).x =  4012+100*5/sqrt(2);
% initial_state(13).y =  -2550-100*5/sqrt(2);
% initial_state(13).speed = 8;
% initial_state(13).group = 3;


%================  Initialisation   =============
for i = 1:1:numel(initial_state)
    initials = initial_state(1,i);
    ID = i;
    targets(i) = Target(ID ,initials ); %#ok<SAGROW>
end

sample_arena = Arena( targets);

tic

for i = 1:1:num_of_UAVs
    
    UAVs(i) = AirCraft(i); %#ok<SAGROW>
    communicators(i) = communicator(i , UAVs(i)); %#ok<SAGROW>
end

sample_mission_planner = mission_planner(UAVs ,communicators, sample_arena );

N = time/sample_arena.dt;
%% Main loop

for i = 1:1:N
    
    for k = 1:1:numel(UAVs)
        
        sample_mission_planner.plan_mission(k);
        
    end
    
    for k = 1:1:numel(UAVs)
        UAVs(k).move(sample_arena);
        
    end
    
 
end
toc
%diary off
%save('simulation_Data.mat')
