
clear all
close all
clc
clear

addpath C:\Users\Pradeep\Documents\MATLAB\Multi-UAV_mission_Ver_4\track

date_and_time = clock;

%diary(strcat('D:\MATLAB\UAVVer4\logs\',...
%    'simulation_logs',num2str(date_and_time(3)), num2str(date_and_time(4)), num2str(date_and_time(5)) ,'.txt'))

%diary on
num_of_UAVs = 7; % number of UAVs in the mission

time = 1400; % simulation time in seconds

%=======  initial states of the targets
initial_state(13).x = 4012+100*1/sqrt(2);
initial_state(13).y = -2550-100*1/sqrt(2);
initial_state(13).speed = 8;

initial_state(12).x = 4012+100*2/sqrt(2);
initial_state(12).y = -2550-100*2/sqrt(2);
initial_state(12).speed = 8;

initial_state(11).x = 4012+100*3/sqrt(2);
initial_state(11).y = -2550-100*3/sqrt(2);
initial_state(11).speed = 8;

initial_state(10).x = 4012+100*1/sqrt(2);
initial_state(10).y = -2650-100*1/sqrt(2);
initial_state(10).speed = 8;

initial_state(9).x = 4012+100*2/sqrt(2);
initial_state(9).y =  -2650-100*2/sqrt(2);
initial_state(9).speed = 8;

initial_state(8).x = 4012+100*3/sqrt(2);
initial_state(8).y =  -2650-100*3/sqrt(2);
initial_state(8).speed = 8;

initial_state(7).x = 4012+100*4/sqrt(2);
initial_state(7).y = -2650-100*4/sqrt(2);
initial_state(7).speed = 8;

initial_state(6).x = 4012+100*5/sqrt(2);
initial_state(6).y =  -2650-100*5/sqrt(2);
initial_state(6).speed = 8;

initial_state(5).x = 4012+100*1/sqrt(2);
initial_state(5).y = -2750-100*1/sqrt(2);
initial_state(5).speed = 8;

initial_state(4).x = 4012+100*2/sqrt(2);
initial_state(4).y =  -2750-100*2/sqrt(2);
initial_state(4).speed = 8;


initial_state(3).x = 4012+100*3/sqrt(2);
initial_state(3).y =  -2750-100*3/sqrt(2);
initial_state(3).speed = 8;

initial_state(2).x = 4012+100*4/sqrt(2);
initial_state(2).y =  -2750-100*4/sqrt(2);
initial_state(2).speed = 8;


initial_state(1).x =  4012+100*5/sqrt(2);
initial_state(1).y =  -2750-100*5/sqrt(2);
initial_state(1).speed = 8;

%================  Initialisation   =============
for i = 1:1:10
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
%% figure plots


figure(1) , hold on

for l = 1:1:num_of_UAVs
    X = [UAVs(l).vehical_log.x];
    Y = [UAVs(l).vehical_log.y];
    Z = [UAVs(l).vehical_log.h];
    if l == 1
        plot3(X , Y , Z)
    elseif l == 2
        
        plot3(X , Y , Z , 'g')
        
    elseif l == 3
        plot3(X , Y , Z , 'r')
    elseif l == 4
        plot3(X , Y , Z ,'c')
    elseif l ==5
        plot3(X , Y , Z ,'k')
    elseif l ==6
        plot3(X , Y , Z ,'m')
    end
    
end

map = sample_arena.road_map;

road_map1 = [map.p1;map.p2;map.p6;...
    map.p7;map.p8;map.p14];


road_map2 = [map.p1;map.p5;map.p13;map.p12;...
    map.p9;map.p10;map.p11;map.p14];

road_map3 = [map.p1;map.p3;map.p7;map.p4;map.p10;map.p8];

road_map4 = [map.p5;map.p9;map.p4;map.p1];

road_map5 = [map.p2;map.p7];

road_map6 = [map.p11;map.p12];

road_map7 = [map.p6;map.p14];

road_map8 = [map.p11;2000 -500];

road_map9 = [map.p13;4250 -500];

plot(road_map1(:,1), road_map1(:,2),'--b')
plot(road_map2(:,1), road_map2(:,2),'--r')
plot(road_map3(:,1), road_map3(:,2),'g')
plot(road_map4(:,1), road_map4(:,2),'g')
plot(road_map5(:,1), road_map5(:,2),'g')
plot(road_map6(:,1), road_map6(:,2),'g')
plot(road_map7(:,1), road_map7(:,2),'g')
plot(road_map8(:,1), road_map8(:,2),'g')
plot(road_map9(:,1), road_map9(:,2),'g')

rectangle('Position',[2000,-3000,2000,2000],'LineWidth',3,'LineStyle','--')
axis equal , grid on,hold off


% other results
for i = 1:1:num_of_UAVs
    
    speed(:,i) = [UAVs(i).vehical_log.v];
    heading(:,i) = [UAVs(i).vehical_log.psi];
    bank(:,i) = [UAVs(i).vehical_log.phi];
    height(:,i) = [UAVs(i).vehical_log.h];
    
end

figure(2)
subplot(4,1,1)
plot(speed)
xlabel('time (s)','fontsize',7)
ylabel('air spd (m/s)','fontsize',7)

subplot(4,1,2)
plot(heading)
xlabel('time (s)','fontsize',7)
ylabel('hdg (deg)','fontsize',7)


subplot(4,1,3)
plot(bank)
xlabel('time (s); solid - actual; dotted - cmd','fontsize',7)
ylabel('bank (deg)','fontsize',7)
subplot(4,1,4)
plot(height)
xlabel('time (s)','fontsize',7)
ylabel('ht (m)','fontsize',7)





