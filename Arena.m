classdef Arena<handle
    %Using Arena you can create a sample environment with some targets
    
    % Author: Pradeep Kumar Yadav , Date: 11 June 2013
    
    % Eg:
    % Syntex:
    % Targets = [2200,-1200;3800,-1841;3138,-2311;2910,-1200 ;2400,-2800 ;3600,-2500];
    % sample_arena = Arena(Targets);

    properties(Constant)
        dt = .1;                       % simulation time step (s)
        % sampling period (s) for plotting state
        rho = 1;                        % density of Air
        g = 9.8;                        %acceleration due to gravity
        disturbance =  [0;              % wind speed (m/s) in x direction (blowing Eastwards)
            0;              % wind speed (m/s) in y direction (blowing Northwards)
            0;              % wind accel (m/s^2) in x direction
            0];             % wind accel (m/s^2) in y direction
  
        grid_density = 6;  % number of probability points on a road 

    end
    
    properties
        
        targets
        
        Area_of_Operation = struct('x',[2000;4000] , 'y', [-1000;-3000])
        % The corners from top left clock wise will be  :
        % 1st : sample_arena.Area_of_Operation.x(1),sample_arena.Area_of_Operation.y(1)
        % 2nd : sample_arena.Area_of_Operation.x(2),sample_arena.Area_of_Operation.y(1)
        % 3rd : sample_arena.Area_of_Operation.x(2),sample_arena.Area_of_Operation.y(2)
        % 4th : sample_arena.Area_of_Operation.x(1),sample_arena.Area_of_Operation.y(2)
        
        AO_waypoint % defined as the geometric center or a corner of the area of operation
        % the point to which all the UAVs are approching after the takeoff mode
        
        road_connection % pair of points is a road
        road_map % all the junction points

    end
    
    
    methods
        function obj = Arena(targets_obj) % Constructer function
 
            obj.AO_waypoint= struct('x',1700 , 'y' , -600-2800 , 'z' , [100, 125 , 150 ,175, 200 , 225 , 250 , 275 , 300, 325 ]);
            % AO_waypoint Defined as the top left corner of the area of operation
            
            obj.targets = targets_obj; % initialising the targets in the arena
            % junction points on the road
            p1 = [3800 -2500];
            p2 = [3000 -2900];
            p3 = [3200 -2600];
            p4 = [3100 -2000];
            p5 = [3900 -2000];
            p6 = [2300 -2900];
            p7 = [2700 -2500];
            p8 = [2400 -2000];
            p9 = [3500 -1600];
            p10 = [2500 -1500];
            p11 = [2200 -1100];
            p12 = [3000 -1200];
            p13 = [3800 -1100];
            p14 = [0 -2000]; 
            
            % saving the above point in the class variable road_map
            obj.road_map.p1 = p1;
            obj.road_map.p2 = p2;
            obj.road_map.p3 = p3;
            obj.road_map.p4 = p4;
            obj.road_map.p5 = p5;
            obj.road_map.p6 = p6;
            obj.road_map.p7 = p7;
            obj.road_map.p8 = p8;
            obj.road_map.p9 = p9;
            obj.road_map.p10 = p10;
            obj.road_map.p11 = p11;
            obj.road_map.p12 = p12;
            obj.road_map.p13 = p13;
            obj.road_map.p14 = p14;
            
            % details of the raod is spedified by its end points , the road
            % network has 24 roads in total
            obj.road_connection(1).pt = [p1;p2];
            obj.road_connection(2).pt = [p2;p6];
            obj.road_connection(3).pt = [p3;p7];
            obj.road_connection(4).pt = [p4;p7];
            obj.road_connection(5).pt = [p5;p1];
            obj.road_connection(6).pt = [p6;p7];
            obj.road_connection(7).pt = [p7;p8];
            obj.road_connection(8).pt = [p8;p10];
            obj.road_connection(9).pt = [p9;p4];
            obj.road_connection(10).pt = [p10;p14];
            obj.road_connection(11).pt = [p11;p12];
            obj.road_connection(12).pt = [p12;p13];
            obj.road_connection(13).pt = [p13;p5];
            obj.road_connection(14).pt = [p14;p11];
            obj.road_connection(15).pt = [p14;p8];
            obj.road_connection(16).pt = [p14;p6];
            obj.road_connection(17).pt = [p12;p9];
            obj.road_connection(18).pt = [p9;p5];
            obj.road_connection(19).pt = [p4;p1];
            obj.road_connection(20).pt = [p4;p10];
            obj.road_connection(21).pt = [p10;p9];
            obj.road_connection(22).pt = [p3;p1];
            obj.road_connection(23).pt = [p10;p11];
            obj.road_connection(24).pt = [p7;p2];
            
            for i = 1:1:numel(obj.targets)
                % initial command to the target
                cmd.type = 'line';
                cmd.data = [obj.road_map.p1(1) obj.road_map.p1(2)];
                obj.targets(i).cmd = cmd;
                
            end
            
            
        end
        
        function move_targets(obj)
            % The IDEA behind the code below is same as of the Aircraft,
            % you specify the command according to which the target will
            % move , but you may notice that unlike the case of the
            % Aircraft model here you dont call the move command after
            % setting a command value , just set the value and the target
            % will move for 0.1 second , angin set the same command it will
            % move another
            
            % function to simulate the target movements 
            for i = 1:1:numel(obj.targets)
                if i>5 && i<=10
                    % making sure the one of the convoy is still for the
                    % first 1 minute
                    if numel(obj.targets(1).vehical_log)>1200
                        
                        obj.targets(i).state.speed = 8;
                    else
                        
                        obj.targets(i).state.speed = 0;
                    end
                    
                    p_temp = obj.targets(i).cmd.data;
                    
                    if  norm([obj.targets(i).state.x-p_temp(1),...
                            obj.targets(i).state.y-p_temp(2)])<=2*obj.targets(i).state.speed
                        if p_temp == obj.road_map.p1
                            
                            p_temp = obj.road_map.p2;
                            
                        elseif p_temp==obj.road_map.p2
                            
                            p_temp = obj.road_map.p6;
                            
                        elseif p_temp==obj.road_map.p6
                            
                            p_temp = obj.road_map.p7;
                            
                        elseif p_temp == obj.road_map.p7
                            
                            p_temp = obj.road_map.p8;
                            
                        elseif p_temp == obj.road_map.p8
                            
                            p_temp = obj.road_map.p14;
                            
                        end
     
                    end
                    cmd.type = 'line';
                    cmd.data = [p_temp(1) p_temp(2)];
                    obj.targets(i).cmd = cmd;
                    
                elseif i<=5
                    if numel(obj.targets(1).vehical_log)>600
                        
                        obj.targets(i).state.speed = 8;
                    else
                        
                        obj.targets(i).state.speed = 0;
                    end
                    
                    p_temp = obj.targets(i).cmd.data;
                    
                    if  norm([obj.targets(i).state.x-p_temp(1),...
                            obj.targets(i).state.y-p_temp(2)])<=2*obj.targets(i).state.speed
                        
                        if p_temp == obj.road_map.p1
                            
                            p_temp = obj.road_map.p4;
                            
                        elseif p_temp == obj.road_map.p4
                            
                            p_temp = obj.road_map.p7;
                        elseif p_temp == obj.road_map.p7
                            
                            p_temp = obj.road_map.p2;
                            
                        elseif p_temp == obj.road_map.p2
                            
                            p_temp = obj.road_map.p6;
                            
                        elseif p_temp == obj.road_map.p6
                            
                            p_temp = obj.road_map.p14;
                            
                        end
                        
                    end
                    cmd.type = 'line';
                    cmd.data = [p_temp(1) p_temp(2)];
                   
                    obj.targets(i).cmd = cmd;
                    
                elseif i>10
                  
                    
                    p_temp = obj.targets(i).cmd.data;
                    
                    if  norm([obj.targets(i).state.x-p_temp(1),...
                            obj.targets(i).state.y-p_temp(2)])<=2*obj.targets(i).state.speed
                        
                        if p_temp == obj.road_map.p1
                            
                            p_temp = obj.road_map.p5;
                            
                        elseif p_temp == obj.road_map.p5
                            
                            p_temp = obj.road_map.p13;
                        elseif p_temp == obj.road_map.p13
                            
                            p_temp = obj.road_map.p12;
                            
                        elseif p_temp == obj.road_map.p12
                            
                            p_temp = obj.road_map.p9;
                            
                        elseif p_temp == obj.road_map.p9
                            
                            p_temp = obj.road_map.p10;
                            
                        elseif p_temp == obj.road_map.p10
                            
                            p_temp = obj.road_map.p11;
                        elseif p_temp == obj.road_map.p11
                            
                            p_temp = obj.road_map.p14;
                            
                        end
                        
                    end
                    cmd.type = 'line';
                    cmd.data = [p_temp(1) p_temp(2)];
                   
                    obj.targets(i).cmd = cmd;
                end
            
            end
            
        end
    end
    
end

