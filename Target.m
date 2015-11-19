classdef Target<handle
    %ROBOT : The robot class can create moving or static targets for
    % simulation of search and track objectives in a mission
    %
    % Author: Pradeep Kumar Yadav , Date :11 June 2013 
    
    %   Properties and methods are described as follows:
    
    properties( GetAccess = 'public')
        
        ID      % Identification number
        group
        state =struct('x' , 0 , 'y' , 0 , 'speed' , 0) % stores state variables
        vehical_log % =struct('x' , 0 , 'y' , 0 , 'speed' , 0, 'cmd_type' , [] , 'cmd_data' , [])
        step_number = 1 
    end
    
    properties (SetAccess = 'private' , GetAccess = 'private')
 
        
        time_step = 0.1
        
        circle_parameters = struct('initial_phase' , 0, 'circling_flag',0 , 'starting_time' , 0)
        line_parameters = struct('flag' , 0, 'm_sin',[],'m_cos',[],'px', [],'py',[] )
        
    end
    properties
       cmd 
    end
    
    methods
        
        function obj = Target(ID , initial_state)
            % constructor function : initialises the object with custom
            % initial states and ID
            obj.ID = ID;
            obj.group = initial_state.group;
            obj.state.x = initial_state.x;
            obj.state.y = initial_state.y;
            obj.state.speed = initial_state.speed;
            
        end
        
        function move(obj)
            %== Logging velical data ==%
            obj.vehical_log(obj.step_number).x = obj.state.x;
            obj.vehical_log(obj.step_number).y =obj.state.y;
            obj.vehical_log(obj.step_number).speed =obj.state.speed;
            
            obj.vehical_log(obj.step_number).cmd_type =obj.cmd.type;
            obj.vehical_log(obj.step_number).cmd_data =obj.cmd.data;
            
            obj.step_number = obj.step_number +1; 
            
            switch obj.cmd.type
                case 'circle'
                    circle(obj);
                    
                case 'line'
                    
                    line(obj);
                case 'halt'
                    halt(obj);
            end
  
        end
       
        
        function set.cmd(obj,newCommand)
            % set function for the cmd 
            % no need to call target(k-th).move , this function will call
            % it for you , just need to set the command
            obj.cmd.type = newCommand.type;
            obj.cmd.data = newCommand.data;
            cmd_check(obj);
            obj.move;
                       
        end
    end
    
end
function cmd_check(obj)
% it  checks the new cmd , if it is different than the previous one then it
% will allot the default value to the required parameters ,i.e if previous
% cmd is line cmd and the new one is circle then it will clears the line
% cmd parameters and set the circle parameters to its initial values
if obj.step_number >2
    if strcmp(obj.cmd.type ,'circle')
        if obj.vehical_log(obj.step_number-1).cmd_data(1)~=obj.cmd.data(1) || obj.vehical_log(obj.step_number-1).cmd_data(2)~=obj.cmd.data(2)||...
                obj.vehical_log(obj.step_number-1).cmd_data(3)~=obj.cmd.data(3)
            
            obj.circle_parameters.circling_flag=0;
            obj.circle_parameters.initial_phase = 0;
            obj.circle_parameters.starting_time = 0;
            
            obj.line_parameters.cy = [];
            obj.line_parameters.cx = [];
            obj.line_parameters.flag = 0;
            obj.line_parameters.m_sin = 0;
            obj.line_parameters.m_cos = 0;
        end
        
        
    end
end
end

function circle(obj)
% the main executer of the circle cmd 

cx = obj.cmd.data(1);
cy = obj.cmd.data(2);
radius = obj.cmd.data(3);
orientation = obj.cmd.data(4);

speed = obj.state.speed;
%=====================


if obj.circle_parameters.circling_flag==0
    if obj.line_parameters.flag ==0;
        
        a = obj.state.x-cx;
        b = obj.state.y-cy;
        
        alpha = asin(a/sqrt(a^2+b^2));
        
        p_theta = asin(radius/sqrt(a^2+b^2))-alpha;
        px_temp = cx+radius*cos(p_theta);
        py_temp = cy+radius*sin(p_theta);
        if orientation==1
            px = px_temp;
            py = py_temp;
            
        elseif orientation ==-1
            m = (obj.state.y-cy)/(obj.state.x-cx);
            px = (2*m/(1+m^2))*(py_temp-cy-m*(px_temp-cx))+px_temp;
            
            py = (-1/m)*(px-px_temp)+py_temp;
        end
        
        obj.line_parameters.flag=1;
        obj.line_parameters.px = px;
        obj.line_parameters.py = py;
        obj.line_parameters.m_sin = (py-obj.state.y)/sqrt((px-obj.state.x)^2+(py-obj.state.y)^2);
        obj.line_parameters.m_cos = (px-obj.state.x)/sqrt((px-obj.state.x)^2+(py-obj.state.y)^2);
    end

    vx = speed*obj.line_parameters.m_cos;
    vy = speed*obj.line_parameters.m_sin;
    
    x = obj.state.x+obj.time_step*vx;
    y = obj.state.y+obj.time_step*vy;
    
    if sqrt((obj.state.x-obj.line_parameters.px)^2 + (obj.state.y-obj.line_parameters.py)^2 )<obj.state.speed*obj.time_step
        obj.line_parameters.flag =0;
        
        obj.circle_parameters.initial_phase = asin((obj.state.x-cx)/radius);
        obj.circle_parameters.circling_flag=1;
        obj.circle_parameters.starting_time = obj.step_number;
    end
else
    angle = orientation*speed*obj.time_step*(obj.step_number-obj.circle_parameters.starting_time)/radius + obj.circle_parameters.initial_phase;
    x = cx+radius*sin(angle);
    y = cy+radius*cos(angle);
end


obj.state.x = x;
obj.state.y = y;

end
function line(obj)
% main executor of the line cmd
x1 = obj.cmd.data(1);
y1 = obj.cmd.data(2);

speed = obj.state.speed;
if isempty(obj.line_parameters.m_cos) || (x1-obj.state.x)/sqrt((x1-obj.state.x)^2+(y1-obj.state.y)^2)~=obj.line_parameters.m_cos...
        ||(y1-obj.state.y)/sqrt((x1-obj.state.x)^2+(y1-obj.state.y)^2)~= obj.line_parameters.m_sin
    
obj.line_parameters.m_cos = (x1-obj.state.x)/sqrt((x1-obj.state.x)^2+(y1-obj.state.y)^2);
obj.line_parameters.m_sin = (y1-obj.state.y)/sqrt((x1-obj.state.x)^2+(y1-obj.state.y)^2);

end

vx = speed*obj.line_parameters.m_cos;
vy = speed*obj.line_parameters.m_sin;

obj.state.x = obj.state.x+obj.time_step*vx;
obj.state.y = obj.state.y+obj.time_step*vy;

end

function halt(~)
% using halt cmd the object will not move from its current position but
% still log the state and inclese the step number
end