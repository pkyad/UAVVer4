classdef mission_planner<handle
    %Console is the main controller of the program , the commands
    %instructions for the object AirCraft will be generated here
    
    %Author : Pradeep Kumar Yadav
    %Date : 20 th may 2013
    
    %   Object from this class will be resposible for the control , the messenger is embeded in
    %   the aircraft itself. you can think of it as the Game planner/designer of the simulation
    
    properties(SetAccess = 'private', GetAccess = 'private')
        
        Agents
        % Saves all the UAVs in the mission
        AC_communicators
        % Saves all the communicators in the mission
        Arena
        % Saves the arena
        running_on
        % running_n = is 1 if the mission planner is planning way points for the 1st UAV and 2 for 2nd UAV ans such for the rest
        takeoff_order
        % Equal to the UAV ID , i.e the first UAV will takeoff first then
        % second UAV and so on
        take_off_orbit
        
        step_number = 1
        % Stores the main itteration number
        figdensity
        
        measurement  % used in the search algo from Dr. Meng
        % Used to record the target found status for a grid
        
    end
    
    properties(Constant)
        target_search_range = 100;
 
        Delta=10000 % Used in the takeoff mode
        
        detectionP=1; % used in the search algo from Dr. Meng
        flaseP=0;
        
    end
    
    properties(SetAccess = 'private' )
        working_mode
        
        % working mode of the Aircraft :
        % 1: for the takeoff mode
        % 2: for the fly to AO mode
        % 3: for the search mode
        % 4: for the track mode
        
        target_assingment
        % target_assingment(k-th) = 1 is the target is assigner a UAV
        % otherwise 0
        current_road_number % current_road_number(k-th) = the number on which the k-th UAV is at the current time step
    end
    
    properties (SetAccess = 'public')
        
        takeoff_parameters    % struct('AC_CP_states' , 0 , 'ready_flag',1 , 'AC_current_level', 0,'jump_in_ready', 0) 
        
        % AC_current_level : aircraft's level of flight during the takeoff mode
        % takeoff flag is 1: already tookoff , else still waiting for the take off
        % AC_CP_states : Aircraft's status in the coordination process
        
        % 0: fly at the current level (not in the holding pattern)
        % 1: fly to the next level
        % 2: fly to the holding pattern
        % 3: fly in the holding pattern
        % 4: timer counts down before switching to the fly_to_AO mode
        
        search_parameters       % struct('CP_states' , 0 ,'group' , 0 , 'keepdensity' , 1 )
        % the state of each UAV's coordination process in the search mode
        % AC_search_states = 0: default value
        % AC_search_states = 1: the first founder UAV , ie. the leader
        % AC_search_states = 2: second UAV in a group
        % AC_search_states = 3: third UAV in a group
        % group = group number =1 or 2
        %  the figdensity OR keepdensity = cell{}[x , y , probability , qmap , weighting , search_success_flag]
        % search altitude is the height during the seach mode (NOTE: As of now it is same for all the UAVs)
        
        
        track_parameters        % struct('states' , 0 ,'start_flag' , 0 , 'terget_ID',0 ,'track_altitudes',[100, 150 ,200, 250])
        % track parameters : start__flag =1 just after the start of tracking the
        % target_ID = 0 if no target is assigner to the UAV and k-th , if
        % the k-th target is assigned to it
        % states  = 0 , 1 ,2  = 0 if it is the 2nd or third UAV in the
        % group , which then go near the target to come in the state 1 and
        % then track the target in the state 2
        % if it is the leader of the group then its initial state is 1 , as
        % it is already near the target convoy

        NTimer        % the waiting time after which if there is no communication recieved from
        % the other UAV in the level above it, it will
        % then move to the other
        
        ICA                         % intermidiate clib altitude used in the takeoff mode
    end
    
    methods
        
        function obj = mission_planner(UAVs ,communicators, sample_arena)
            
            obj.Agents = UAVs;
            % initialising the agents data
            obj.AC_communicators = communicators;
            % initialising the communicators data
            obj.Arena = sample_arena;
            % initialising the arena data
            obj.working_mode = ones(1,numel(UAVs)); %setting the working mode to take off mode
            % initialising all the working mode to takeoff mode
            obj.takeoff_order = transpose(linspace(1,numel(UAVs)));
            obj.NTimer = 20*ones(numel(UAVs));
            obj.target_assingment = zeros(numel(sample_arena.targets) , 1);
            obj.current_road_number = zeros(numel(UAVs) , 1);
            %===== Initilising the parameters used in the search mode
            
            grid_density = sample_arena.grid_density;
            num_of_roads = numel(sample_arena.road_connection);
            obj.figdensity = cell(grid_density , num_of_roads);
            
            for i = 1:1:num_of_roads
                
                pts = sample_arena.road_connection(i).pt;
                m = (pts(1,2)-pts(2,2))/(pts(1,1)-pts(2,1));
                
                x_mat = linspace(pts(1,1),pts(2,1),grid_density);
                y_mat = m.*(x_mat-pts(1,1))+pts(1,2);
                
                for t = 1:1:grid_density
                    obj.figdensity{t,i}  = [x_mat(t) , y_mat(t) , 0.5 , 0 , 0, 0];
                    %  the figdensity OR keepdensity = cell{}[x , y , probability , qmap , weighting , search_success]
                end
            end
            
            obj.measurement = zeros(numel(obj.Agents),1);
            
            for i = 1:1:numel(UAVs)
                % Parameters used in the takeoff mode
                obj.takeoff_parameters(i).AC_CP_states = 0;
                obj.takeoff_parameters(i).ready_flag = 0;
                obj.takeoff_parameters(i).AC_current_level = 0;
                obj.takeoff_parameters(i).jump_in_ready = 0;
                % Parameters used in the search mode
                obj.search_parameters(i).keepdensity = obj.figdensity;
                %  the figdensity OR keepdensity = cell{}[x , y , probability , qmap , weighting , search_success_flag]
                obj.search_parameters(i).CP_states=0;
                obj.search_parameters(i).group=0;

                % Parameters used in the track mode
                obj.track_parameters(i).start_flag = 0;
                obj.track_parameters(i).states = 0;
                obj.track_parameters(i).target_ID = 0;
                obj.track_parameters(i).track_altitudes = [100, 150 ,200, 250]; % altitude(meters)
                
            end
        end
        
        function output = get.take_off_orbit(obj)
            % get function for takeoff_orbit
            
            output.x=1000;
            output.y=-4000;
            output.z=obj.running_on*30*mod(obj.running_on,2) + 70 + 10*mod(obj.running_on,3);
            output.rad=300;
            
            
        end
        
        function output = get.takeoff_order(obj)
            % get function for takeoff_order
            output = obj.running_on;
            
        end
        
        function output = get.ICA(obj)
            % get function for ICA
            
                
             output(1).x=0;
             output(1).y=-4000;
             output(1).z= mod(obj.running_on,2)*100 + 100;
             output(1).rad=200;
             output(1).ready=0;         % Level 1 is not the jump-in level for UAVi
                
             output(2).x=0;
             output(2).y=-4000;
             output(2).z=mod(obj.running_on,2)*100 + 100;
             output(2).rad=200;
             output(2).ready=1;         % Level 2 is the jump-in level for UAVi
                
             output(3).x=0;
             output(3).y=-4000;
             output(3).z=mod(obj.running_on,2)*100 + 100;
             output(3).rad=200;
             output(3).ready=1;
                
           
        end
   
        function plan_mission( obj ,k )
            
            obj.running_on = k;
            
            UAVs = obj.Agents;
            
            AC=UAVs(k);
            
            arena = obj.Arena;
            
            switch obj.working_mode(k)
                case 1 % UAV is in the takeoff mode
                    AC.cmd=takeoff_mode(obj , AC , arena , UAVs);
                    
                case 2 % UAV is in the Fly_to_AO mode
                    AC.cmd=Fly_to_AO_mode(obj , arena );
                    
                case 3  % UAV is in the Searching mode
                    AC.cmd=Search_mode(obj, AC , arena );
                    
                case 4  % UAV is in the Tracking mode
                    AC.cmd=Track_mode(obj , AC , arena);
                    
            end
            
            obj.AC_communicators(k).communication_success(obj.AC_communicators)
            
            for i = 1:1:numel(obj.AC_communicators)
                % Send hearbeats to others
                to =i;
                if mod(obj.step_number,10 )== 0 && to~=k
                    
                    obj.AC_communicators(k).send_heart_beat(obj.AC_communicators , to);
                    
                end
                
            end
            obj.step_number = obj.Agents(1).step_number;
            
            % plots
            if ~isempty(find(obj.working_mode>=3 , 1)) && k ==1
                arena.move_targets % simulate the targets

            end
            
        end
        
    end
    
end


function cmd = takeoff_mode(obj , AC , arena , UAVs )

ID = obj.running_on;
ICA = obj.ICA;
state = AC.state;
level = obj.takeoff_parameters(ID).AC_current_level;
take_off_orbit = obj.take_off_orbit;
n  = obj.step_number;
switch obj.takeoff_parameters(ID).AC_CP_states
    
    case 0  %% when UAV(ii) is at the initial state, do the following operations
        % Operation 1: check whether previously launched UAVs (specified by
        % the takeoff_order) are above the targeting orbitting level of UAV(ii).
        obj.takeoff_parameters(ID).ready_flag = 1; % when Ready_flag == 0, it means some previously launched is not above the targeting orbitting level of UAV(ii)
        % Here we use the last received coordinates of each previously launched UAV for calculation. Therefore, we assume
        % that during the takeoff mode each UAV will not
        % decrease its altitude. Thus, it is valid to use its historical data.
  
        for jj=1:numel(UAVs)
            try
                if ID <= jj && (ICA(obj.takeoff_parameters(jj).AC_current_level+1).z - UAVs(jj).state.h <= 3)
                    
                    obj.takeoff_parameters(ID).ready_flag = 1;

                end
            catch
                
            end
        end
        
        
        if obj.takeoff_parameters(ID).ready_flag == 1  % Once no previously launched UAV is blocking UAV(ii), UAV(ii) starts to fly to the targetting orbitting level, which is
            % is the next intermediate level.
            obj.takeoff_parameters(ID).AC_CP_states = 1; % move to state 1
            
            level  = level  + 1;
            obj.takeoff_parameters(ID).AC_current_level = level;
            cmd.type = 'orbit';
            cmd.orbit = [ICA(level).x;ICA(level).y;ICA(level).z;12;15;ICA(level).rad;1];
        else  % If some previously launched UAV is blocking UAV(ii), then UAV(ii) maintains its current operation altitude.
            if obj.takeoff_parameters(ID).AC_current_level == 0 % if UAV(ii) is on the ground, keep it on the ground.
                cmd.type = 'onGrd';
                cmd.orbit = [];
            else
                
                cmd.type = 'orbit';  % if UAV(ii) is not on the ground, keep its current altitude
                cmd.orbit = [ICA(level).x;ICA(level).y;ICA(level).z;12;15;ICA(level).rad;1];
            end
        end
        
    case 1 % UAV(ii) is at state 1, which means it is in the ascending process towards the targetting orbitting level.
        % We use value a to determine whether UAV(ii) reaches its
        % targetting level, i.e., whether its distance from the centroid of
        % the orbit is sufficiently small.
        a = abs((state.x-ICA(level).x)^2+(state.y-ICA(level).y)^2+(state.h-ICA(level).z)^2 -(ICA(level).rad)^2);
        switch a < obj.Delta
            case 0  % The distance of UAV(ii) from the centroid of the orbit is larger than a prespecified threshold value Delta.
                % This means UAVi has not reached the designated targetting level yet. Then UAV(ii) continues climbing.
                cmd.type = 'orbit';
                cmd.orbit =  [ICA(level).x;ICA(level).y;ICA(level).z;12;15;ICA(level).rad;1];
            case 1 % UAV(ii) has reached the designated targetting level. There are two possibilities: either UAV(ii) is at the same level as its prespecified holding pattern, or is not.
                % In the first case, which is called a jump-in level described by
                % ICA(ii,UAV_current_levels(ii)).ready == 1, UAV(ii)
                % decides to merge into the holding pattern. In the
                % second case, UAV(ii) continues its climbing by
                % returning back to state 0.
                switch ICA(level).ready == 1
                    case 0 % the current level is not the jump-in level for UAV(ii)
                        obj.takeoff_parameters(ID).AC_CP_states = 0; % UAVi goes back to state 0 and will continue climbing to the next intermediate level
                        cmd.type = 'orbit';
                        cmd.orbit = [ICA(level).x;ICA(level).y;ICA(level).z;12;15;ICA(level).rad;1];
                    case 1 % the current level is the jump-in level for UAV(ii)
                        % UAVi waits for a gap to appear in the holding
                        % pattern before joining in.
                        
                        % Considering the current setup of the holding pattern we require all UAVs to jump into a holding pattern from the 1th quadron of the current intermediate orbit
                        % We first determine the jump-in x-y coordinates,
                        % denoted as (u,v), which gives the shortest
                        % distance from the current coordinates of UAV(ii)
                        % in the first quadron to the holding pattern.
                        
                        if state.x >=0 && state.y >= -4000 % To ensure that UAV(ii) is in the first quadron.
                            u = take_off_orbit.x + (take_off_orbit.rad/(sqrt((state.x - take_off_orbit.x)^2+(state.y - take_off_orbit.y)^2)))*(state.x - take_off_orbit.x);
                            v = take_off_orbit.y + (take_off_orbit.rad/(sqrt((state.x - take_off_orbit.x)^2+(state.y - take_off_orbit.y)^2)))*(state.y - take_off_orbit.y);
                            
                            %Calculate the time instant that UAVi reaches (u,v) from its current location
                            ti = sqrt((state.x - u)^2+(state.y - v)^2)/15 + n*arena.dt;  % each step_no takes simParam.dt time unit during simulation
                            
                            % determine the time instant for each UAV in the holding pattern to reach (u,v)
                            obj.takeoff_parameters(ID).jump_in_ready = 1;
                            for j=1:numel(UAVs)
                                if j==ID
                                    continue;
                                end
                                
                                if obj.take_off_orbit.x==take_off_orbit.x && obj.take_off_orbit.y==take_off_orbit.y && obj.take_off_orbit.z==take_off_orbit.z && obj.take_off_orbit.rad==take_off_orbit.rad % && hbRecdHist(2,j,ii)>=3  UAVj has the same holding pattern as UAVi and already in the holding pattern
                                    
                                    %consider which quadron UAVj is and then determine the time instant that UAVj will reach (u,v)
                                    
                                    x_axis = UAVs(j).state.x;
                                    y_axis = UAVs(j).state.y;
                                    
                                    if x_axis >= obj.take_off_orbit.x
                                        theta = atan((y_axis-obj.take_off_orbit.y)/(x_axis-obj.take_off_orbit.x)) + pi - atan((v-obj.take_off_orbit.y)/(u-obj.take_off_orbit.x));
                                    else
                                        theta = atan((y_axis-obj.take_off_orbit.y)/(x_axis-obj.take_off_orbit.x)) - atan((v-obj.take_off_orbit.y)/(u-obj.take_off_orbit.x));
                                        if theta < 0
                                            theta = theta + 2*pi;
                                        end
                                    end
                                    tj = theta*obj.take_off_orbit.rad/15 + n*arena.dt; % assume that the flying speed in the holding pattern is 15
                                    
                                    if abs(tj - ti) < 30  % If the gap is lower than 30 seconds, then UAV(ii) cannot jump in.
                                        obj.takeoff_parameters(ID).jump_in_ready = 0;
                                        break;
                                    end
                                end
                            end
                            if obj.takeoff_parameters(ID).jump_in_ready == 1  % When the gap is bigger than 30 seconds, UAV(ii) is ready to jump in the holding pattern
                                cmd.type = 'orbit';
                                cmd.orbit = [take_off_orbit.x;take_off_orbit.y;take_off_orbit.z;12;15;take_off_orbit.rad;1];
                                
                                
                                
                                obj.takeoff_parameters(ID).AC_CP_states = 2;
                                return;
                            end
                        end
                        cmd.type = 'orbit';
                        cmd.orbit = [ICA(level).x;ICA(level).y;ICA(level).z;12;15;ICA(level).rad;1];
                end
        end
    case 2 % UAV(ii) is at state 2, which means it is flying towards and orbitting its holding pattern.
        a = abs(((state.x-take_off_orbit.x)^2+(state.y-take_off_orbit.y)^2+(state.h-take_off_orbit.z)^2)-(take_off_orbit.rad)^2);
        if a < obj.Delta  % UAV(ii) has reached the holding pattern
            obj.takeoff_parameters(ID).AC_CP_states = 3;
            
        end
        cmd.type = 'orbit';
        cmd.orbit = [take_off_orbit.x;take_off_orbit.y;take_off_orbit.z;12;15;take_off_orbit.rad;1];
    case 3 % UAV(ii) is at state 3, where it is in the holding pattern and starts to check whether other UAVs are also in their holding patterns.
        flag =1;
        for i = 1:1:numel(obj.Agents)
            if obj.takeoff_parameters(i).AC_CP_states<3
               flag =flag*0;
            else
                flag = flag*1;
            end
        end
        if flag ==1
            
            obj.takeoff_parameters(ID).AC_CP_states = 4;  % Once UAV(ii) starts its countdown, it moves to the last state before switching to the fly_to_AO mode.
        end
        obj.NTimer(obj.running_on) = 500;
        cmd.type = 'orbit';
        cmd.orbit = [take_off_orbit.x;take_off_orbit.y;take_off_orbit.z;12;15;take_off_orbit.rad;1];
    case 4 % UAV(ii) is at state 4. When countdown reaches zero, UAV(ii) flies to the AO, i.e., switches to the working_mode 2.
        obj.NTimer(obj.running_on) = obj.NTimer(obj.running_on) - 1;
        if obj.NTimer(obj.running_on) == 0
            obj.working_mode(ID) = 2;
        end
        cmd.type = 'orbit';
        cmd.orbit = [take_off_orbit.x;take_off_orbit.y;take_off_orbit.z;12;15;take_off_orbit.rad;1];
end

end


function cmd = Fly_to_AO_mode(obj, arena)
AC = obj.Agents(obj.running_on);
cmd.type = 'wayPt';
cmd.wayPt = [arena.AO_waypoint.x;arena.AO_waypoint.y;arena.AO_waypoint.z(obj.running_on);14];

% check whether UAV(i_uav) has crossed the boundary of AO and entered it.
% If a boundary is crossed, then UAV(i_uav) switches to the searching mode.

if AC.state.x>arena.AO_waypoint.x && arena.AO_waypoint.y < AC.state.y 
    % Check whether UAV[UAV_no] has crossed the boundary of AO.
    
    obj.working_mode(obj.running_on) = 3;  % switch to the searching mode
    fprintf('UAV %d is now in the search mode \n' , obj.running_on)

    
end


end


function cmd = Search_mode(obj ,  AC , arena )

% get the new command after every 50 itteration
if mod(obj.step_number,50)==0
    
    NewWayPoint = GetNewWayPoint(obj, AC , arena );
    
else
    NewWayPoint.x = AC.cmd.wayPt(1);
    NewWayPoint.y = AC.cmd.wayPt(2);
end

cmd.type = 'wayPt';
cmd.wayPt = [NewWayPoint.x;NewWayPoint.y;AC.state.h;14];


end


function NewWayPoint = GetNewWayPoint(obj , AC , arena )
targetNUM = numel(arena.targets);
grid_density = arena.grid_density; % Mesh size
robotNUM = numel(obj.Agents); % number of UAVs searching the area
rc = obj.AC_communicators(1).commsRange;
detectionP = obj.detectionP; % Detection probability
mm = obj.running_on;
num_of_roads = numel(arena.road_connection);
keta = 2;

%========distance btwn the UAVs

Alldistance = zeros(robotNUM,robotNUM);
for i=1:robotNUM
    for j=i+1:robotNUM
        Alldistance(i,j) = sqrt((obj.Agents(i).state.x-obj.Agents(j).state.x )^2+(obj.Agents(i).state.y-obj.Agents(j).state.y)^2);
        % Distance between i-th and j-th UAV =  Alldistance(i,j)
    end
end

Alldistance=Alldistance+Alldistance';

%=============== Updating the search_success_flag

for j = 1:1:num_of_roads
    
    for t = 1:1:grid_density
        obj.search_parameters(mm).keepdensity{t,j}(6)  = 0;
        
    end
end


for i = 1:1:numel(arena.targets)
    flag = 0;
    a = arena.targets(i).state.x ;
    b = arena.targets(i).state.y ;
    
    for j = 1:1:num_of_roads
        
        pts = arena.road_connection(j).pt;
        m = (pts(1,2)-pts(2,2))/(pts(1,1)-pts(2,1));
        
        x_mat = linspace(pts(1,1),pts(2,1),grid_density);
        y_mat = m.*(x_mat-pts(1,1))+pts(1,2);
        
        
        for t = 1:1:grid_density
            dist = sqrt((x_mat(t)-a)^2+(y_mat(t)-b)^2);
            if flag ==0
                min = dist;
                indexes = [t , j];
                flag = 1;
            else
                if dist<min
                    min = dist;
                    indexes = [t, j];
                end
            end
        end
        
    end
    
    obj.search_parameters(mm).keepdensity{indexes(1),indexes(2)}(6)  = 1;
end

%=========== Updating the personal Q map and keepdensity matrix

temp_density_map = obj.search_parameters(mm).keepdensity;


for i = 1:1:num_of_roads
    
    for t = 1:1:grid_density
        
        a = temp_density_map{t,i}(1);
        b = temp_density_map{t,i}(2);
        
        dist = sqrt((AC.state.x-a)^2 + (AC.state.y-b)^2);
        
        
        if dist<obj.target_search_range
            
            if temp_density_map{t,i}(6) == 1
                
                detectnum = rand;
                if detectnum<=detectionP
                    obj.measurement(mm)=1;
                    
                else
                    obj.measurement(mm)=0;
                end
            else
                flasenum = rand;
                if flasenum<=obj.flaseP
                    obj.measurement(mm)=1;
                else
                    obj.measurement(mm)=0;
                end
            end
            if obj.measurement(mm)==1
                
                temp_density_map{t,i}(3)=detectionP*temp_density_map{t,i}(3)/...
                    (detectionP*temp_density_map{t,i}(3)+obj.flaseP*(1-temp_density_map{t,i}(3)));
                % target found =======================================================================================
                % doing assignment of targets and UAVs to groups

                % making sure that the target found does not belongs to any
                % other group that is already found or being tracked by
                % other group of UAVs 
                
                % making sure that different targets are assigned to
                % different first founder
                flag = 0;

                for p = 1:1:numel(arena.targets)

                    % finding the target ID which i found
                    if obj.target_assingment(p)==0 %&& isempty(find(mat==1, 1))

                        a = arena.targets(p).state.x ;
                        b = arena.targets(p).state.y ;
                        if flag ==0
                            min = sqrt((AC.state.x-a)^2+(AC.state.y-b)^2);
                            foundTarget = p;
                            flag =1;
                        else
                            dist = sqrt((AC.state.x-a)^2+(AC.state.y-b)^2);
                            if dist < min
                                min = dist;
                                foundTarget = p;

                            end
                        end
                        %fprintf('the target %d is at x = %d and y = %d from UAV which is a = %d and b = %d with diatance %d \n' , p, a , b, AC.state.x , AC.state.y , min)
                    end
                end
                fprintf('found target %d at a distance of %d \n' , foundTarget , min)
                for q = 1:1:numel(arena.targets)
                    if arena.targets(foundTarget).group == arena.targets(q).group

                        mat(1,q) =  obj.target_assingment(q);
                    end
                end
                
                if obj.search_parameters(mm).group ==0 && obj.search_parameters(mm).CP_states ==0 &&...
                        isempty(find(mat==1, 1))
                    
                    
                    % if i am not assigned any target
                    group_num =0;
                    
                    for p = 1:1:robotNUM
                        
                        if obj.search_parameters(p).group> group_num
                            group_num = obj.search_parameters(p).group ;
                        end
                    end

                    % taking care of the case when I am already tracking
                    % and found a target to I will simply call the near by
                    % free UAV and convery the message that I found a
                    % target and am already tracking another so you take
                    % care of the newly found target
                    
                    a = arena.targets(foundTarget).state.x ;
                    b = arena.targets(foundTarget).state.y ;

                    r = 1;
                    obj.target_assingment(foundTarget ,1)=1;
                    for p = 1:1:robotNUM
                       
                        % preparing the list of those having no group assigned
                        
                        if obj.search_parameters(p).group==0 && obj.search_parameters(p).CP_states ==0
                            those_are_free(r,1) = p;
                            distFromTarget = sqrt((obj.Agents(p).state.x-a)^2+(obj.Agents(p).state.y-b)^2);
                            those_are_free(r,2) = distFromTarget;
                            r = r+1;
                            fprintf('UAV %d interpreted that UAV %d is free at at distance of %d from the target \n' , mm , p , distFromTarget);
                        end
                    end
                    
                    count = 0;
                    
                    for p = 1:1:targetNUM
                        if arena.targets(p).group == arena.targets(foundTarget).group
                            count = count +1;
                        end
                    end
                    
                    if numel(those_are_free(:,2))-5 > 0
                        % we have more then enough UAVs for allocation at
                        % the ratio of 5:3
                        if count == 5
                           assignment = 5;
                        elseif count == 4
                           assignment = 3;
                        end
                    else 
                        assignment = numel(those_are_free(:,2));
                    end
                    % calling the clossest 3 of those are free
                    [~ , indexes] = sort(those_are_free(:,2), 'ascend');
                    fprintf('This simulation has %d free UAVs \n' , numel(those_are_free(:,2)))
                    for q = 1:1:assignment
                        obj.search_parameters(those_are_free(indexes(q), 1)).group=group_num +1;
                        fprintf('UAV %d belongs to group %d \n' , (those_are_free(indexes(q), 1))  , group_num +1)

                        fprintf('UAV %d is now in track mode \n' , (those_are_free(indexes(q), 1)) )
                        obj.working_mode(those_are_free(indexes(q), 1)) = 4;
                    end
                    
                    new_group = group_num +1;
                    y = 2;
                    grp_members_id(1) = those_are_free(indexes(1), 1); % the nearest free UAV to the found target should be on the top of this list
                    obj.track_parameters(grp_members_id(1)).target_ID = foundTarget;
                    fprintf('UAV %d assinged UAV %d the target no. %d  \n',mm , grp_members_id(1) , obj.track_parameters(grp_members_id(1)).target_ID)
                    for r = 1:1:numel(obj.Agents)
                        if obj.search_parameters(r).group == new_group && obj.track_parameters(r).target_ID ==0 && ...
                                obj.search_parameters(r).CP_states==0 && r ~= grp_members_id(1)
                            grp_members_id(y) =r;
                            y = y+1;
                        
                        end
                    end
                    % assinging the states , i,e the leader , 2nd follower
                    % and the last one
                    if obj.track_parameters(grp_members_id(1)).target_ID~=0 &&  obj.search_parameters(grp_members_id(1)).CP_states==0
                        % I am the leader of this group
                        y = 1;
                        for r = grp_members_id
                            if y ==1 
                                fprintf('UAV %d belongs to group %d and is the Leader \n' , r , new_group )
                            else
                                fprintf('UAV %d belongs to group %d and is on number %d in list\n' , r , new_group ,y )
                            end
                            
                            obj.search_parameters(r).CP_states = y;
                            y = y+1;
                        end
                        
                    end
                    
                end
            else
                temp_density_map{t,i}(3)=(1-detectionP)*temp_density_map{t,i}(3)/...
                    ((1-detectionP)*temp_density_map{t,i}(3)+(1-obj.flaseP)*(1-temp_density_map{t,i}(3)));
            end
            
        end
        % decay function
        temp_density_map{t,i}(3) = exp(-0.0010).*(temp_density_map{t,i}(3)-0.5)+0.5;
        temp_density_map{t,i}(4) = log(1/temp_density_map{t,i}(3)-1);
        temp_density_map{t,i}(5) = exp(-keta.*abs(temp_density_map{t,i}(4)));
    end
    
end

%===============sharing the map======================

for nn=1:robotNUM
    if Alldistance(mm,nn)<rc
        obj.search_parameters(nn).keepdensity = temp_density_map;
    end
end

%===== finding the new way pt ========

road_num = obj.current_road_number(mm); % there are 24 roads
poa_x = AC.cmd.wayPt(1);
poa_y = AC.cmd.wayPt(2);

connections = arena.road_connection;

if road_num == 0 % distribution at the first junction , top left corner one
    % if the UAV never been on any road
    approaching_x = arena.road_map.p6(1);
    approaching_y = arena.road_map.p6(2);
    
    
    if sqrt((AC.state.x-approaching_x)^2+(AC.state.y-approaching_y)^2)<7*AC.state.v % wheather i am on close 
        % enough to the junction do find the next junction
        %====================================================================================================
        % finding the available options for roads on the next junction ,
        % only those which are not occupied by other UAV
        y = 1;
        next_available_roads = [];
        
        for k = 1:1:numel(connections)
            
            
            if isempty(find(obj.current_road_number == k, 1)) && isempty(find([10 , 14 , 15 , 16]==k,1))
                
                temp_pt = connections(k).pt;
                for kk = 1:1:2
                    if abs(approaching_x-temp_pt(kk ,1))<=5 &&  abs(approaching_y-temp_pt(kk ,2))<=5
                        %fprintf('%d road is free , the UAVs are on road %d ,%d and %d \n' , k , obj.current_road_number(1),obj.current_road_number(2),obj.current_road_number(3))
                        next_available_roads(y,1) = k;
                        if kk ==1
                            next_available_roads(y,2) = 2;
                        else
                            next_available_roads(y,2) = 1; %#ok<*AGROW>
                        end
                        y = y +1;
                    end
                end
                
            end
        end

        %====================================================================================================
        if isempty(next_available_roads)
            % list of all the road options  from the next junction no matter
            % wheather there is an UAV on the road or not
            y = 1;
            for k = 1:1:numel(connections)
                
                if isempty(find([10 , 14 , 15 , 16]==k,1))
                    temp_pt = connections(k).pt;
                    
                    
                    for kk = 1:1:2

                        if abs(approaching_x-temp_pt(kk ,1))<=5 &&  abs(approaching_y-temp_pt(kk ,2))<=5
                            
                            next_available_roads(y,1) = k;
                            if kk ==1
                                next_available_roads(y,2) = 2;
                            else
                                next_available_roads(y,2) = 1; 
                            end
                            y = y +1;
                        end
                    end
                    
                end
            end

        end
        % finding the road on which the probability of having a target is maximum
        % basically it is generally the one which is not searched by the
        % other UAV for a long time
        flag = 0;
        for k = 1:1:size(next_available_roads,1)
            if flag ==0
                avg_probability = 0;
                for kk = 1:1:grid_density
                    avg_probability = avg_probability + temp_density_map{kk,next_available_roads(k,1)}(3);
                end
                which = k;
                max = avg_probability;
                flag = 1;
            else
                avg_probability = 0;
                for kk = 1:1:grid_density
                    avg_probability = avg_probability + temp_density_map{kk,next_available_roads(k,1)}(3);
                end
                if max < avg_probability
                    
                    which = k;
                    max = avg_probability;
                    
                end
            end
            
        end
       
        next_road_num = next_available_roads(which,1);
        temp_pt = connections(next_road_num).pt;
        NewWayPoint.x = temp_pt(next_available_roads(which,2) ,1);
        NewWayPoint.y = temp_pt(next_available_roads(which,2),2);
        obj.current_road_number(mm) = next_road_num ;
    else
        NewWayPoint.x = approaching_x;
        NewWayPoint.y = approaching_y;
    end
else
    
    if sqrt((AC.state.x-poa_x)^2+(AC.state.y-poa_y)^2)<7*AC.state.v
        % change the road
        
        y = 1;
        next_available_roads = [];
        for k = 1:1:numel(connections)
            
            if isempty(find(obj.current_road_number == k, 1))&& isempty(find([10 , 14 , 15 , 16]==k,1))
                
                temp_pt = connections(k).pt;
                for kk = 1:1:2
                    if abs(poa_x-temp_pt(kk ,1))<=5 &&  abs(poa_y-temp_pt(kk ,2))<=5
                        next_available_roads(y,1) = k;
                        if kk ==1
                            next_available_roads(y,2) = 2;
                        else
                            next_available_roads(y,2) = 1; %#ok<*AGROW>
                        end
                        y = y +1;
                    end
                end
                
            end
        end
        if isempty(next_available_roads)
            % list of all the road options  from the next junction no matter
            % wheather there is an UAV on the road or not
            y = 1;
            for k = 1:1:numel(connections)
                
                if isempty(find([10 , 14 , 15 , 16]==k,1))
                    temp_pt = connections(k).pt;
                    for kk = 1:1:2
                        if abs(poa_x-temp_pt(kk ,1))<=5 &&  abs(poa_y-temp_pt(kk ,2))<=5
                            connected_roads(y,1) = k;
                            if kk ==1
                                connected_roads(y,2) = 2;
                            else
                                connected_roads(y,2) = 1; %#ok<*AGROW>
                            end
                            y = y +1;
                        end
                    end
                    
                end
            end
            next_available_roads = connected_roads;
        end
        % finding the road on which the probability of having a target is maximum
        % basically it is generally the one which is not searched by the
        % other UAV for a long time
        flag = 0;
        for k = 1:1:size(next_available_roads,1)
            if flag ==0
                avg_probability = 0;
                for kk = 1:1:grid_density
                    avg_probability = avg_probability + temp_density_map{kk,next_available_roads(k,1)}(3);
                end
                which = k;
                max = avg_probability;
                flag = 1;
            else
                avg_probability = 0;
                for kk = 1:1:grid_density
                    avg_probability = avg_probability + temp_density_map{kk,next_available_roads(k,1)}(3);
                end
                if max < avg_probability
                    
                    which = k;
                    max = avg_probability;
                    
                end
            end
            
        end
        next_road_num = next_available_roads(which,1);
        temp_pt = connections(next_road_num).pt;
        NewWayPoint.x = temp_pt(next_available_roads(which,2) ,1);
        NewWayPoint.y = temp_pt(next_available_roads(which,2),2);
        obj.current_road_number(mm) = next_road_num ;
        
    else
        NewWayPoint.x = poa_x;
        NewWayPoint.y = poa_y;
        
    end
    
end

end


function cmd = Track_mode(obj , AC , arena)
mm = obj.running_on;
N = numel(obj.Agents);

temp_ID = obj.track_parameters(mm).target_ID;

if  obj.search_parameters(mm).CP_states==1 % means i am the leader
    % if i am the leader , i will look for the leader target in the convoy
    % , ie , 5th or 10th target
    target  = arena.targets(obj.track_parameters(mm).target_ID);
    if sqrt((AC.state.x-target.state.x)^2+(AC.state.y-target.state.y)^2)<50 && (target.ID==5 || target.ID==10)
        obj.track_parameters(mm).states =1;
        
    end
end


if temp_ID == 0 % either i am the 2nd or third UAV in a group
    for j = 1:1:N
        if obj.search_parameters(j).group == obj.search_parameters(mm).group &&...
                obj.track_parameters(j).target_ID ~=0
            
            targetGroupToTrack = arena.targets(obj.track_parameters(j).target_ID).group;
            l = 1;
            for k = 1:1:numel(arena.targets)
                if arena.targets(k).group == targetGroupToTrack && obj.target_assingment(k,1)==0
                   possibleTargets(l, 1) = k;
                   l = l+1;
                end
            end
            break;
        end
    end
    
    if obj.search_parameters(mm).CP_states == 2

        l = 1;
        for k = 1:1:numel(arena.targets)
            if arena.targets(k).group == targetGroupToTrack
               allTargets(l, 1) = k;
               l = l+1;
            end
        end
        for k = 1:1:numel(N)
        
           if obj.search_parameters(mm).group == obj.search_parameters(k).group &&  obj.search_parameters(k).CP_states == 1
              obj.track_parameters(k).target_ID = allTargets(1,1);
              fprintf('Traget changed for UAV %d and now will track target %d \n' , k , obj.track_parameters(k).target_ID );
           end
        end
        
        try
            obj.track_parameters(mm).target_ID = possibleTargets(2,1);
        catch
            obj.track_parameters(mm).target_ID = possibleTargets(1,1);
        end
        obj.target_assingment(obj.track_parameters(mm).target_ID)=1;
        fprintf('UAV %d will track target %d \n' , mm , obj.track_parameters(mm).target_ID );
    elseif  obj.search_parameters(mm).CP_states >= 3
        obj.track_parameters(mm).target_ID = possibleTargets(numel(possibleTargets),1);
        fprintf('UAV %d will track target %d \n' , mm , possibleTargets(numel(possibleTargets),1) );
        obj.target_assingment(obj.track_parameters(mm).target_ID)=1;
    end  
end
target  = arena.targets(obj.track_parameters(mm).target_ID);
if obj.track_parameters(mm).states == 0

    cmd.type = 'wayPt';
    cmd.wayPt = [target.state.x;target.state.y;AC.state.h;14];
    
    if sqrt((AC.state.x-target.state.x)^2+(AC.state.y-target.state.y)^2)<30
        obj.track_parameters(mm).states =1; 
    end
    
elseif obj.track_parameters(mm).states==1 && obj.track_parameters(mm).start_flag==0
    % When the UAV is near the target convoy assigned it will then come in
    % the position suitable to the code from Dr. He Zhirong then track
    try
        new_way = new_way_to_track(obj,AC,arena);
    catch
        new_way(1)= target.state.x;
        new_way(2) = target.state.y;
    end
    if sqrt((AC.state.x-target.state.x)^2+(AC.state.y-target.state.y)^2)<70
        cmd.type = 'wayPt';
        cmd.wayPt = [new_way(1);new_way(2);AC.state.h;14];
    else
        cmd.type = 'wayPt';
        cmd.wayPt = [target.state.x;target.state.y;AC.state.h;14];
    end
end

end


function new_way = new_way_to_track(obj,AC , arena)
% preparing the input data for the code from Dr. He
R=80;

V=AC.state.v;
target = arena.targets(obj.track_parameters(obj.running_on).target_ID);
v=target.state.speed;

V1=V/R; % V/Rreal
v1=v/R;

pos(1) = AC.state.x/R;
pos(2) = AC.state.y/R;

target_heading = find_heading(target);
AC_heading = find_heading(AC);


x0=[pos(1),pos(2),AC_heading,V1]; % start point of UAV
y0=[target.state.x/R,target.state.y/R,target_heading,v1];  % start point of target

if mod(obj.step_number , 10 )==0
    temp=ptls(x0,y0); % calling the Dr. He's track function
    % 2*3 just the location and direction of TAR and UAV, without V and v,
    new_way(1)=temp(1,1)*R;
    new_way(2)=temp(1,2)*R;
else
    
    new_way(1) = AC.cmd.wayPt(1);
    new_way(2) = AC.cmd.wayPt(2);
    
end

end


function heading = find_heading(object)
% heading is the angle measured from the +ve x-axis anticlockwise
prev_pos = [object.vehical_log(object.step_number-2).x , object.vehical_log(object.step_number-2).y];
current_pos = [object.vehical_log(object.step_number-1).x , object.vehical_log(object.step_number-1).y];
heading_vec = current_pos-prev_pos;
unit_vec = [1 0];
heading = acos(dot(heading_vec , unit_vec)/norm(heading_vec));
if heading_vec(2)<0
    heading = 2*pi-heading;
    
end

end

