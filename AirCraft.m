classdef AirCraft<handle
    
    % AirCraft is a class which can be used to model an AirCraft
    
    % Author : Pradeep Kumar Yadav
    
    % The Logics and algotithms is adapted from Aircraft model
    % provided by TL @ NUS (Dr. Surong, Dr. Rodney and the Team)
    
    % Date : 20th may 2013
    
    %   All the properties are described below:
    % Eg:
    %
    % Syntex : sample_Aircraft = AirCraft; %( Generate an object Aircraft )
    %
    %       Update cmd: access as sample_AirCraft.cmd
    %       sample_AirCraft.cmd.type = 'primitive' OR 'wayPt' OR 'orbit'
    %
    %       sample_AirCraft.cmd.primitive = [airspd;alt;yaw]; (m/s;m;deg) yaw is measured
    %       clockwise from North
    %
    %       sample_AirCraft.cmd.wayPt =[E;     }
    %                                   N;     }  way point (m)
    %                                   U;     }
    %                                   airspd];   (m/s)
    %       Note: UAV will fly to way ENU at airspd
    %
    %       sample_AirCraft.cmd.orbit = [E;            }
    %                                   N;            } centre of orbit (m)
    %                                   U;            }
    %                                   fly2orbAirspd; (m/s) fly to orbit speed
    %                                   orbAirspd;    (m/s) fly in orbit speed
    %                                   radius;       (m) radius of orbit
    %                                   direction];   clockwise if 1; anti-clockwise if -1.
    
    % Syntex(Method) : sample_Aircraft.move;        % (Move the aircraft according to the command)
    
    properties(SetAccess = private)
        AC_ID
        % Assigned identifiaction no.
        
        vehical_log
        % Vehical Log saves the log data
        % obj.vehical_log(step_no).x           % m (East)
        % obj.vehical_log(step_no).y           % m (North)
        % obj.vehical_log(step_no).h           % m (height)
        % obj.vehical_log(step_no).v           % m/s (airspeed) 12.4 m/s to 16.5 m/s
        % obj.vehical_log(step_no).gamma       % flight path angle (rad)
        % obj.vehical_log(step_no).psi         % yaw angle (rad) clockwise from North
        % obj.vehical_log(step_no).phi         % bank angle (rad) (+ve for a bank to the right)
        % obj.vehical_log(step_no).cmdAirspd = obj.cmd.primitive(1);
        % obj.vehical_log(step_no).cmdAlt = obj.cmd.primitive(2);
        
        step_number =1
        % access the step number as : 
        % Syntex: temp = sample_AirCraft.step_number;
    end
    
    properties(Constant , GetAccess = private)
        
        
        
        %conversion
        
        r2d = 180/pi;                          % radians to degrees
        
        % Basic parameters
        W          =   15;                     % weight (kg)
        span       =   2.5;                    % wing span (m)
        S          =   0.8;                      % wing ref area (m2)
        delta      =   0.01;                   % for the beta calculation (coefficient of Cl for Cd)
        Cdo        =   0.02;                   % parasite drag coefficient
        
        % sample_arenaspeed, throttle
        tau_T      =   45;                     % time constants for thrust response
        kd_T       =   0.1;                    % Kd for T control
        zero_T     =   -10;                    % zero position for T control
        kp_T       =   -AirCraft.zero_T*AirCraft.kd_T;	    % Kp for T control
        
        T_max      =   100;                    % max thrust
        T_min      =   -100;                   % min thrust
        v_dot_max       =   3;                 % max accel command (m/s2)
        v_dot_min       =   -3;                % min accel command (m/s2)
        v_max           =   16.5;              % max speed command (m/s) (approx. Mach 0.8 at sea level)
        v_min           =   12.4;              % min speed command (m/s)
        
        % Climb, altitude, flight path angle
        tau_Cl     =   1;                      % time constants for lift coefficient response
        kd_Cl      =   0.5;           		   % Kd for Cl control
        zero_Cl    =   -1;                     % zero position for Cl control
        kp_Cl      =   -AirCraft.zero_Cl*AirCraft.kd_Cl;   % Kp for Cl control
        k_gamma    =   0.007;                  % kp for altitude hold
        
        Cl_max     =   100;                    % max lift coefficient
        Cl_min     =   -100;                   % min lift coefficient
        gamma_dot_max   =   15/AirCraft.r2d;   % max flt path angle rate command (rad/s)
        gamma_dot_min   =   -15/AirCraft.r2d;  % min flt path angle rate command (rad/s)
        gamma_max       =   13/AirCraft.r2d;   % max flt path angle (rad)
        gamma_min       =   -15/AirCraft.r2d;  % min flt path angle (rad)
        
        % Heading, bank
        K_phi      =   0.5102;	               % gain for phi control
        tau_phi    =   0.1;                    % time constants for roll response
        k_psi      =   0.7;                    % kp for heading hold
        
        phi_max    =   12/AirCraft.r2d;        % max roll angle
        
        % Higher level modes
        k_st       =   0.8;                    % kp for ground track hold
        tau_m      =   10;                     % time constants for ground track hold
        
        % Measurement noise
        x_sd       =   5;                      % m standard deviation of x measurement
        y_sd       =   5;                      % m standard deviation of y measurement
        h_sd       =   5;                      % m standard deviation of h measurement
        v_sd       =   2;                      % m/s standard deviation of v measurement
        gamma_sd   =   3/AirCraft.r2d;         % rad/s standard deviation of gamma measurement
        psi_sd     =   3/AirCraft.r2d;         % rad standard deviation of psi measurement
        phi_sd     =   3/AirCraft.r2d;         % rad standard deviation of phi measurement
        omega_sd   =   3/AirCraft.r2d;         % rad/s standard deviation of omega measurement
        
    end
    
    
    properties (SetAccess = private , GetAccess = public)
        
        state
        
        measured_state
        % state, measured_state have the same structure as state
        
        % state.x = m (East)
        % state.y = m (North)
        % state.h = m (height)
        % state.v = m/s (sample_speed) 12.4 m/s to 16.5 m/s
        % state.gamma = flight path angle (rad)
        % state.psi = yaw angle (rad) clockwise from North
        % state.phi = bank angle (rad) (+ve for a bank to the right)
        % state.omega = roll rate (rad/s)
        % state.Cl = lift coefficient rate (/s)
        % state.T = thrust rate
        % state.vPrev = m/s (sample_speed) 12.4 m/s to 16.5 m/s
        % state.gammaPrev = flight path angle (rad)
 
        hb = struct('time',[] , 'pos',[] , 'speed',[] , 'hdg',[] , 'fp',[])
        
        % hb: Heart Beat stores the current hearbeat of the Aircraft
    end
    properties(SetAccess = public)
        
        cmd
        % cmd.type = 'primitive','wayPt','orbit'
        %
        % cmd.primitive = [airspd;alt;yaw]; (m/s;m;deg) yaw is measured
        % clockwise from North
        %
        % cmd.wayPt =        [E;     }
        %                     N;     }  way point (m)
        %                     U;     }
        %                     airspd];   (m/s)
        %   Note: UAV will fly to way ENU at airspd
        %
        % cmd.orbit =        [E;            }
        %                     N;            } centre of orbit (m)
        %                     U;            }
        %                     fly2orbAirspd; (m/s) fly to orbit speed
        %                     orbAirspd;    (m/s) fly in orbit speed
        %                     radius;       (m) radius of orbit
        %                     direction];   clockwise if 1; anti-clockwise if -1.

        
    end
    
    methods
        
        function obj = AirCraft(AC_ID_number) 
            
            % Constructer function : to initialise the AirCraft by giving it a Identification number
            
            obj.AC_ID = AC_ID_number; % giving the Aircraft an ID
            
            if mod(AC_ID_number,2)==0  % the initial state of the Aircraft when ID==1
                
                obj.state.x = -300*rand(1);
                obj.state.y = -250*rand(1);
                obj.state.h = 0;
                obj.state.v = 14.4;
                obj.state.gamma = 0;
                obj.state.psi = 45/obj.r2d;
                obj.state.phi = 0;
                obj.state.omega = 0;
                obj.state.Cl = 0;
                obj.state.T = 0;
                obj.state.vPrev = 0;
                obj.state.gammaPrev = 0;
                
            elseif mod(AC_ID_number,2)~=0 % the initial state of the Aircraft when ID==2
                
                obj.state.x = -300*rand(1);
                obj.state.y = 150*rand(1);
                obj.state.h = 0;
                obj.state.v = 12.4;
                obj.state.gamma = 0;
                obj.state.psi = 135/obj.r2d;          
                obj.state.phi = 0;          
                obj.state.omega = 0;        
                obj.state.Cl = 0;           
                obj.state.T = 0;            
                obj.state.vPrev = 0;        
                obj.state.gammaPrev = 0;             
               
           end
               
        end
        
        function move(obj,sample_arena)
           % Public function to make the Aircract move according to the date in sample_AirCraft.cmd 
            step_no = obj.step_number;
            
            switch obj.cmd.type
                case 'primitive'
                    
                    [obj.state,obj.measured_state] = DoPrimitive(obj,[],[],sample_arena);
                case 'wayPt'
                    
                    [obj.state,obj.measured_state] = DoWayPt(obj, sample_arena ,...
                        obj.cmd,sample_arena.disturbance);
                case 'orbit'
                    
                    [obj.state,obj.measured_state] = DoOrbit(obj, sample_arena);
                    
                case 'onGrd'
                    [obj.state,obj.measured_state] = DoOnGrd(obj );
            end
            %=======Logging===============================%
            
            obj.vehical_log(step_no).x = obj.state.x;          % m (East)
            obj.vehical_log(step_no).y = obj.state.y;          % m (North)
            obj.vehical_log(step_no).h = obj.state.h;          % m (height)
            obj.vehical_log(step_no).v = obj.state.v;          % m/s (airspeed) 12.4 m/s to 16.5 m/s
            obj.vehical_log(step_no).gamma = obj.state.gamma;  % flight path angle (rad)
            obj.vehical_log(step_no).psi = obj.state.psi;      % yaw angle (rad) clockwise from North
            obj.vehical_log(step_no).phi = obj.state.phi;      % bank angle (rad) (+ve for a bank to the right)
            
            switch obj.cmd.type
                case 'primitive'
                    obj.vehical_log(step_no).cmdAirspd = obj.cmd.primitive(1);
                    obj.vehical_log(step_no).cmdAlt = obj.cmd.primitive(2);
                case 'wayPt'
                    obj.vehical_log(step_no).cmdAirspd = obj.cmd.wayPt(4);
                    obj.vehical_log(step_no).cmdAlt = obj.cmd.wayPt(3);
                case 'orbit'
                    obj.vehical_log(step_no).cmdAirspd = obj.cmd.orbit(5);
                    obj.vehical_log(step_no).cmdAlt = obj.cmd.orbit(3);
            end
            
            % Updating the Heart Beat of the UAV
            
            obj.hb.time = step_no;
            obj.hb.pos.x = obj.state.x;
            obj.hb.pos.y = obj.state.y;
            obj.hb.pos.h = obj.state.y;
            obj.hb.speed = obj.state.v;
            obj.hb.hdg = obj.state.psi;
            obj.hb.fp = obj.state.gamma;
   
            obj.step_number = step_no +1;
        end
        
    end
    
    
end



function control = controller(obj, sample_arena , state,cont_outer)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function for controller                                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% here obj refers to the AirCraft
% ==================
% Saturate commands
% ==================
if cont_outer.v_dot > obj.v_dot_max
    cont_outer.v_dot=obj.v_dot_max;
elseif cont_outer.v_dot < -obj.v_dot_max
    cont_outer.v_dot=-obj.v_dot_max;
end
if cont_outer.v > obj.v_max
    cont_outer.v=obj.v_max;
elseif cont_outer.v < obj.v_min
    cont_outer.v=obj.v_min;
end
if cont_outer.gamma_dot > obj.gamma_dot_max
    cont_outer.gamma_dot=obj.gamma_dot_max;
elseif cont_outer.gamma_dot < -obj.gamma_dot_max
    cont_outer.gamma_dot=-obj.gamma_dot_max;
end
if cont_outer.gamma > obj.gamma_max
    cont_outer.gamma=obj.gamma_max;
elseif cont_outer.gamma < -obj.gamma_max
    cont_outer.gamma=-obj.gamma_max;
end
if cont_outer.phi > obj.phi_max
    cont_outer.phi=obj.phi_max;
elseif cont_outer.phi < -obj.phi_max
    cont_outer.phi=-obj.phi_max;
end

% =======================
% Compute variables
% =======================
%
m=obj.W/sample_arena.g;
AR=obj.span^2/obj.S;

beta=(1.0+obj.delta)/(pi*AR);

%q_ref=obj.rho*cont_outer.v^2/2;
q_ref=sample_arena.rho*state.v^2/2;

%Cl_ref=1/(q_ref*obj.S*cos(cont_outer.phi)) ...
%    *(m*cont_outer.v*cont_outer.gamma_dot+obj.W*cos(cont_outer.gamma));
Cl_ref=1/(q_ref*obj.S*cos(state.phi)) ...
    *(m*state.v*cont_outer.gamma_dot+obj.W*cos(state.gamma));
Cd=obj.Cdo+Cl_ref^2*beta;
%T_ref=m*cont_outer.v_dot+Cd*q_ref*obj.S+obj.W*sin(cont_outer.gamma);
T_ref=m*cont_outer.v_dot+Cd*q_ref*obj.S+obj.W*sin(state.gamma);
% if numel(obj.vehical_log)==0
%     vPrev = obj.state.v;
%     gammaPrev = obj.state.gamma;
%     
% else
%     vPrev = obj.vehical_log(numel(obj.vehical_log)).v;
%     gammaPrev = obj.vehical_log(numel(obj.vehical_log)).gamma; 
% end

v_dot=(state.v-state.vPrev)/sample_arena.dt;
gamma_dot=(state.gamma-state.gammaPrev)/sample_arena.dt;

% ==================
% Compute control
% ==================
Cl_cont=Cl_ref+obj.kp_Cl*(cont_outer.gamma-state.gamma)+obj.kd_Cl*(cont_outer.gamma_dot-gamma_dot);
T_cont=T_ref+obj.kp_T*(cont_outer.v-state.v)+obj.kd_T*(cont_outer.v_dot-v_dot);
phi_cont=cont_outer.phi;

% ==================
% Saturate control
% ==================
if Cl_cont > obj.Cl_max
    Cl_cont=obj.Cl_max;
elseif Cl_cont < obj.Cl_min
    Cl_cont=obj.Cl_min;
end
if T_cont > obj.T_max
    T_cont=obj.T_max;
elseif T_cont < obj.T_min
    T_cont=obj.T_min;
end
if phi_cont > obj.phi_max
    phi_cont=obj.phi_max;
elseif phi_cont < -obj.phi_max
    phi_cont=-obj.phi_max;
end

% ==================
% Assign control
% ==================
control=[Cl_cont;T_cont;phi_cont];
end


function [state,meas_state] = DoOnGrd(obj )
% ========================================================================
% Do onGrd function
% ======================
%

state = obj.state;

meas_state.x      =   state.x+randn(1)*obj.x_sd;          % x-position (m)
meas_state.y      =   state.y+randn(1)*obj.y_sd;         % y-position (m)
meas_state.h      =   state.h+randn(1)*obj.h_sd;          % altitude (m)
meas_state.v      =   state.v+randn(1)*obj.v_sd;          % sample_arenaspeed (m/s)
meas_state.gamma  =   state.gamma+randn(1)*obj.gamma_sd;  % flight path angle (rad)
meas_state.psi    =   state.psi+randn(1)*obj.psi_sd;      % heading wrt North (+ve clockwise) (rad)
meas_state.phi    =   state.phi+randn(1)*obj.phi_sd;      % roll angle (rad)
meas_state.omega  =   state.omega+randn(1)*obj.omega_sd;  % roll rate (rad/s)
state = meas_state;
end


function [state,meas_state] = DoOrbit(obj ,sample_arena)


statePrev = obj.state;
cmd = obj.cmd;
disturbance = sample_arena.disturbance;

orbCen = cmd.orbit(1:2);
orbRad = cmd.orbit(6);
orbSpd = cmd.orbit(5);
orbAlt = cmd.orbit(3);
orbDir = cmd.orbit(7);
fly2orbSpd = cmd.orbit(4);

% check distance from destination orbit centre

xy2centre_vector = orbCen-[statePrev.x;statePrev.y];
n_xy2centre_unit = xy2centre_vector/norm(xy2centre_vector);
xy2centre_dist=dot(xy2centre_vector,n_xy2centre_unit);

tangent_hdg=atan2(n_xy2centre_unit(1),n_xy2centre_unit(2)) ...
    -orbDir*asin(orbRad/xy2centre_dist);
tangent_unit_vector=[sin(tangent_hdg);cos(tangent_hdg)];
dist2tangent_pt=sqrt(xy2centre_dist^2-orbRad^2);
tangent_pt=[statePrev.x;statePrev.y]+dist2tangent_pt*tangent_unit_vector;

if xy2centre_dist > orbRad*2
    % too far away, so fly tangential path
    cmd.wayPt = [tangent_pt;orbAlt;fly2orbSpd];
    [state,meas_state] = DoWayPt(obj ,sample_arena,cmd,disturbance);
else
    % near enough, start orbiting
    radial_vector=[statePrev.x-orbCen(1,1);
        statePrev.y-orbCen(2,1)];
    n_radial_vector=radial_vector/norm(radial_vector);
    command.lat.mode = 2;
    orb_start_pt=orbCen+orbRad*n_radial_vector;
    orb_end_pt=orbSpd*10*[0 orbDir;-orbDir 0]*n_radial_vector+orb_start_pt;
    cmd.wayPt = [orb_end_pt;orbAlt;orbSpd];
    [state,meas_state] = DoWayPt(obj , sample_arena,cmd,disturbance);
end

end


function [state,meas_state] = DoPrimitive(obj ,flt_path_ang,roll,sample_arena)

% ========================================================================
% Do primitive function
% ======================
%
% Inputs
% =======
% obj - see assign_obj()
% speed (m/s)
% alt (m) or flt_path_ang (deg) (choose one or the other)
% roll (deg) or hdg (deg) (choose one or the other)
%
% e.g. DoPrimitive(obj,state,30,10,[],[],45,disturbance,sim_state)
% commands a sample_arenaspeed of 30 m/s, altitude of 10 m, heading of 45 deg
% clockwise from the North.
%
% disturbance = [wind_x;wind_y;wind_dot_x;wind_dot_y]; x = East, y = North
%                   in m/s and m/s^2
%
% sim_state - see InitDoPrimitive()

speed = obj.cmd.primitive(1);
alt = obj.cmd.primitive(2);
hdg  = obj.cmd.primitive(3);
disturbance = sample_arena.disturbance;
statePrev = obj.state;
command.speed=speed;
if isempty(alt)
    command.alt.gamma=flt_path_ang;
    command.alt.mode=0;
elseif isempty(flt_path_ang)
    command.alt.alt=alt;
    command.alt.mode=1;
end
if isempty(roll)
    command.lat.hdg=hdg;
    command.lat.mode=1;
elseif isempty(hdg)
    command.lat.roll=roll;
    command.lat.mode=0;
end

cont_outer=primitive_control(obj , statePrev,command);
control=controller(obj , statePrev,cont_outer);
state=VehDynInt(obj, statePrev,control,disturbance);

meas_state.x      =   state.x+randn(1)*obj.x_sd;          % x-position (m)
meas_state.y      =   state.y+randn(1)*obj.y_sd;          % y-position (m)
meas_state.h      =   state.h+randn(1)*obj.h_sd;          % altitude (m)
meas_state.v      =   state.v+randn(1)*obj.v_sd;          % sample_arenaspeed (m/s)
meas_state.gamma  =   state.gamma+randn(1)*obj.gamma_sd;  % flight path angle (rad)
meas_state.psi    =   state.psi+randn(1)*obj.psi_sd;      % heading wrt North (+ve clockwise) (rad)
meas_state.phi    =   state.phi+randn(1)*obj.phi_sd;      % roll angle (rad)
meas_state.omega  =   state.omega+randn(1)*obj.omega_sd;  % roll rate (rad/s)
end


function [state,meas_state] = DoWayPt(obj, sample_arena,cmd,disturbance)

statePrev= obj.state;


% Find hdg of way point from current UAV position
vecUAV2wayPt = cmd.wayPt(1:2)-[statePrev.x;statePrev.y];
hdg2wayPt = atan2(vecUAV2wayPt(1),vecUAV2wayPt(2))*obj.r2d; % (deg)

% Assign command
command.speed = cmd.wayPt(4);
command.alt.mode = 1;
command.alt.alt = cmd.wayPt(3);
command.lat.mode = 1;
command.lat.hdg = hdg2wayPt;

cont_outer=primitive_control(obj , statePrev,command);
control=controller(obj , sample_arena , statePrev,cont_outer);
state=VehDynInt(obj , sample_arena , statePrev,control,disturbance);

meas_state.x      =   state.x+randn(1)*obj.x_sd;          % x-position (m)
meas_state.y      =   state.y+randn(1)*obj.y_sd;          % y-position (m)
meas_state.h      =   state.h+randn(1)*obj.h_sd;          % altitude (m)
meas_state.v      =   state.v+randn(1)*obj.v_sd;          % sample_arenaspeed (m/s)
meas_state.gamma  =   state.gamma+randn(1)*obj.gamma_sd;  % flight path angle (rad)
meas_state.psi    =   state.psi+randn(1)*obj.psi_sd;      % heading wrt North (+ve clockwise) (rad)
meas_state.phi    =   state.phi+randn(1)*obj.phi_sd;      % roll angle (rad)
meas_state.omega  =   state.omega+randn(1)*obj.omega_sd;  % roll rate (rad/s)

end


function angle=place_angle(angle)

while angle > pi
    angle=angle-2*pi;
end
while angle <= -pi
    angle=angle+2*pi;
end
end


function cont_outer=primitive_control(obj, state,command)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function for outer loop control        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% command = the command input into the AirCraft
%   command.type    =   type of command
%               0 for primitive command
%               1 for way point command
%
%   command.speed       =   speed command (m/s)
%
%   command.alt.mode    =   altitude command mode
%       0 for flight path angle hold
%       1 for altitude hold
%   command.alt.gamma   =   flight path angle (deg) command if mode 0
%   command.alt.alt     =   altitude (m) command if mode 1
%
%   command.lat.mode    =   lateral dynamics command mode
%       0 for roll angle hold
%       1 for heading hold
%       2 for ground track hold
%   command.lat.roll    =   roll angle (deg) command if mode 0
%   command.lat.hdg     =   heading angle (deg) command if mode 1
%   command.lat.start_end   =   [start_x end_x; if mode 2
%                                start_y end_y] (m)
%
% =============================================================
% =========================
% Speed channel
% =========================
cont_outer.v            =   command.speed;  % speed command (m/s)

% =========================
% Altitude channel
% =========================
if command.alt.mode == 0
    % =======================
    % Flight path angle hold
    % =======================
    cont_outer.gamma=command.alt.gamma/obj.r2d;
elseif command.alt.mode == 1
    % ====================
    % Altitude hold
    % ====================
    cont_outer.gamma=obj.k_gamma*(command.alt.alt-state.h);
end

% ========================
% Roll channel
% ========================
if command.lat.mode == 0
    % ====================
    % Roll angle hold
    % ====================
    cont_outer.phi=command.lat.roll/obj.r2d;
elseif command.lat.mode == 1
    % ==================
    % Heading hold
    % ==================
    psi_error=place_angle(command.lat.hdg/obj.r2d-state.psi);
    cont_outer.phi=obj.k_psi*psi_error;
elseif command.lat.mode == 2
    % ================
    % Ground track
    % ================
    n_unit=[0 1;-1 0]*[command.lat.start_end(:,2)-command.lat.start_end(:,1)]...
        /norm([command.lat.start_end(:,2)-command.lat.start_end(:,1)]);
    dl=dot(([state.x;state.y]-command.lat.start_end(:,1)),n_unit);
    % ensure that temp is one or less so that asin is not imaginary
    temp=-dl/(state.v*obj.tau_m);
    if abs(temp) > 1
        temp=sign(temp)*1;
    end
    dpsi_ref=asin(temp);
    %    dpsi_ref=asin(-dl/(state.v*obj.tau_m));
    psi_ref=atan2(command.lat.start_end(1,2)-command.lat.start_end(1,1), ...
        command.lat.start_end(2,2)-command.lat.start_end(2,1));
    psi_error=place_angle(psi_ref+dpsi_ref-state.psi);
    cont_outer.phi=obj.k_st*psi_error;
end

cont_outer.v_dot        =   0;              % accel command (m/s2)
cont_outer.gamma_dot    =   0;              % flight path angular rate command (rad/s)

end


function stateOut=VehDynInt(obj, sample_arena , state,control,disturbance)


% function for 4th order Runga-Kutta integration routine %



stateX(1,1)       =   state.x;        % x-position (m)
stateX(2,1)       =   state.y;        % y-position (m)
stateX(3,1)       =   state.h;        % altitude (m)
stateX(4,1)       =   state.v;        % sample_arenaspeed (m/s)
stateX(5,1)       =   state.gamma;    % flight path angle (rad)
stateX(6,1)       =   state.psi;      % heading wrt North (+ve clockwise) (rad)
stateX(7,1)       =   state.phi;      % roll angle (rad)
stateX(8,1)       =   state.omega;    % roll rate (rad/s)
stateX(9,1)       =   state.Cl;       % lift coefficient rate (/s)
stateX(10,1)      =   state.T;        % thrust rate (N/s)



alpha1=1/2;alpha2=1/2;beta1=1/2;beta3=1/2;
alpha3=1;beta6=1;
beta2=0;beta4=0;beta5=0;
gamma1=1/6;gamma4=1/6;
gamma2=1/3;gamma3=1/3;

k1=sample_arena.dt*VehDyn(obj,sample_arena , stateX,control,disturbance);
k2=sample_arena.dt*VehDyn(obj,sample_arena , stateX+beta1*k1, ...
    control,disturbance);
k3=sample_arena.dt*VehDyn(obj,sample_arena ,stateX+beta2*k1+beta3*k2, ...
    control,disturbance);
k4=sample_arena.dt*VehDyn(obj,sample_arena , stateX+beta4*k1+beta5*k2+beta6*k3, ...
    control,disturbance);

stateX=stateX+gamma1*k1+gamma2*k2+gamma3*k3+gamma4*k4;

stateOut.x         =   stateX(1);      % x-position (m)
stateOut.y         =   stateX(2);      % y-position (m)
stateOut.h         =   stateX(3);      % altitude (m)
stateOut.v         =   stateX(4);      % sample_arenaspeed (m/s)
stateOut.gamma     =   stateX(5);      % flight path angle (rad)
stateOut.psi       =   stateX(6);      % heading wrt North (+ve clockwise) (rad)
stateOut.phi       =   stateX(7);      % roll angle (rad)
stateOut.omega     =   stateX(8);      % roll rate (rad/s)
stateOut.Cl        =   stateX(9);      % lift coefficient rate (/s)
stateOut.T         =   stateX(10);     % thrust rate (N/s)
stateOut.vPrev     =   state.v;        % m/s (sample_arenaspeed) 12.4 m/s to 16.5 m/s (previous)
stateOut.gammaPrev =   state.gamma;    % flight path angle (rad) (previous)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% differential equation to be integrated %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%	point mass model of AirCraft with wind
%
end


function stateX_dot=VehDyn(obj, sample_arena ,stateX,control,disturbance)



% =====================
% Assign variables
% =====================
x       =   stateX(1);   % x-position (m)
y       =   stateX(2);   % y-position (m)
h       =   stateX(3);   % altitude (m)
v       =   stateX(4);   % sample_arenaspeed (m/s)
gamma   =   stateX(5);   % flight path angle (rad)
psi     =   stateX(6);   % heading wrt North (+ve clockwise) (rad)
phi     =   stateX(7);   % roll angle (rad)
omega   =   stateX(8);   % roll rate (rad/s)
Cl      =   stateX(9);   % lift coefficient rate (/s)
T       =   stateX(10);  % thrust rate (N/s)

Cl_cont  =  control(1); % lift coefficient control input
T_cont   =  control(2); % thrust control input (N)
phi_cont =  control(3); % roll angle control input (rad)

wind_x      =   disturbance(1); % wind speed, x-direction (m/s)
wind_y      =   disturbance(2); % wind speed, y-direction (m/s)
wind_dot_x  =   disturbance(3); % wind accel, x-direction (m/s2)
wind_dot_y  =   disturbance(4); % wind accel, y-direction (m/s2)

% =====================
% Compute variables
% =====================

m=obj.W/sample_arena.g;
AR=obj.span^2/obj.S;

beta=(1.0+obj.delta)/(pi*AR);

q=sample_arena.rho*v^2/2;

L=Cl*q*obj.S;
D=(obj.Cdo+beta*Cl^2)*q*obj.S;

wind_dot_v=wind_dot_x*sin(psi)+wind_dot_y*cos(psi);
wind_dot_left=-wind_dot_x*cos(psi)+wind_dot_y*sin(psi);

% =====================
% Equations of motion
% =====================

x_dot=v*cos(gamma)*sin(psi) ...
    +wind_x;
y_dot=v*cos(gamma)*cos(psi) ...
    +wind_y;
h_dot=v*sin(gamma);
v_dot=(T-D-obj.W*sin(gamma))/m ...
    -cos(gamma)*wind_dot_v;
gamma_dot=(L*cos(phi)-obj.W*cos(gamma))/(m*v) ...
    +(sin(gamma)/v)*wind_dot_v;
psi_dot=L*sin(phi)/(m*v*cos(gamma)) ...
    +wind_dot_left/(v*cos(gamma));
phi_dot=omega;
omega_dot=(obj.K_phi*(phi_cont-phi)-omega)/obj.tau_phi;
Cl_dot=(Cl_cont-Cl)/obj.tau_Cl;
T_dot=(T_cont-T)/obj.tau_T;

% =====================
% Assign output vector
% =====================

stateX_dot=[x_dot;y_dot;h_dot;v_dot;gamma_dot;
    psi_dot;phi_dot;omega_dot;Cl_dot;T_dot];

end




