classdef communicator < handle
    %COMMUNICATOR : You can think the communicator as an Iphone , It helps
    % you to compose messeges , send message which can be sent to the
    % respective owner via a communcation channel such as Air , Use is as a
    % GPS navigator , it can tell where its owner is and also guide the
    % owner to its targets assigned
    %
    % Author : Pradeep Kumar Yadav , Date: 22 may 2013
    
    % examples will follow a sample_communicator object from the class
    % communicator which can be generated by assigning it an identification
    % number : for eg. :
    %
    % sample_communicator = communicator(12345 , ith-UAV)
    %
    %
    % Communicator is having the discribed properties and methods:
    %
    % Properties: 
    % 1. identification_number
    % 2. inbox
    % 3. draft
    % 4. sent
    % 5. map (eg. : position = [sample_communicator.map.current_location.x , sample_communicator.current_location.map.y])
    % 6. hb_inbox : stores the heat beats recieved from others
    
    %
    % methods than can be performed are :
    %
    % 1. Draft message 
    %
    % 2. read message 

    
    properties
        
        identification_number
        
        owner

        draft = struct('message', [])

        % obj.draft.message.to=             'All' OR i-th
        % obj.draft.message.from =          i-th
        % obj.draft.message.subject=        'CNET'
        % obj.draft.message.message=        'TARGET' OR 'PROPOSAL' OR 'AWARD'
        % obj.draft.message.message_body=   [TARGET: 'x|y|static'
        %                                    PROPOSAL: 'x|y|cost'
        %                                    AWARD: 'x|y|static']
        
        inbox = struct('message', [])
        sent = struct('message' , [])
        % data structure is same for inbox and sent box described as
        % for k-th message the message can be read as 
        
        % obj.sent.message(k-th).to=             'All' OR i-th
        % obj.sent.message(k-th).from =          i-th
        % obj.sent.message(k-th).subject=        'CNET'
        % obj.sent.message(k-th).message=        'TARGET' OR 'PROPOSAL' OR 'AWARD'
        % obj.sent.message(k-th).message_body=   [TARGET: 'x|y|static'
        %                                         PROPOSAL: 'x|y|cost'
        %                                         AWARD: 'x|y|static']        
        
        % An additional data type message_read_flag is present in case of a
        % message in the inbox its value is 0 when it is not processed yet
        % otherwise 1 , you can access it as : 
        % obj.draft.message(k-th).message_read_flag=   [0 or 1]
        
        
        map %= struct('destination' , struct('x' ,[] ,'y',[] , 'type' ,[]),'current_location', struct('x' , 0 ,'y', 0 , 'z' , 0))
        
        % next destination can be found by :
        % Syntex:
        % target_assigned = [sample_communicator.map.destination(i-th).x,sample_communicator.map.destination(i-th).y]
        % my_current_location = [sample_communicator.map.current_location.x,sample_communicator.map.current_location.y]
        
        hb_inbox = struct('hb' , [])
        % the k-th recieved heartbeat consist of 
        
        % sample_communicator.hb_inbox.hb(k-th).time  
        % sample_communicator.hb_inbox.hb(k-th).pos.x 
        % sample_communicator.hb_inbox.hb(k-th).pos.y  
        % sample_communicator.hb_inbox.hb(k-th).pos.h 
        % sample_communicator.hb_inbox.hb(k-th).speed  
        % sample_communicator.hb_inbox.hb(k-th).hdg  
        % sample_communicator.hb_inbox.hb(k-th).fp  
        
    end
    properties(SetAccess = 'private')
        
        communication_success_flag
        % = [1 0 1 0 0... number of registered communicators in the network]
        % if previous_communication_success_flag(k-th)==1
        %   communication successfull;
        % else
        %   communication failed;
        % end
        % i.e. the message from this communicator to k-th communicator is
        % successfull or failed
        previous_communication_success_flag
        % stores the previous communication_success_flags
        wait_for_reply=30
        % The time to wait for others to send a proposal
        proposal_read_parameters = struct('x' , [] , 'y' , [] ,'cost' , [], 'who' ,[])
        % Used in the processing of the proposals recieved
    end
    
    properties (Constant)
        pRecv_prevLost = .5
        % The probability of communication success if the previous
        % communication attempt has failed
        pRecv_prevRecd = .95
        % The probability of communication success if the previous
        % communication attempt was successful
        commsRange = 3000
        % Communication range
    end
    
    methods
        
        function obj = communicator( number ,Owner) 
            % Constructor function
            
            obj.identification_number = number;
            % Assining an identification number
            obj.owner = Owner;
            % Assining an owner : Owner is an object of class Aircraft
            obj.map.destination = struct('x' ,[] ,'y',[] , 'type' ,[]);
            % initialising destination as an empty structure
            obj.map.current_location = struct('x' , 0 ,'y', 0 , 'z' , 0);
            % initialising position
            
        end
        
        function draft_message(obj,All_communicators , target_to_annaounce)
            % Syntex :
            % target_to_annaounce.x = x_of_target_found;
            % target_to_annaounce.y = y_of_target_found;
            % k-th communicator = All_communicators(k-th)
            % sample_communicator.draft_message( All_communicators , target_to_annaounce));
            
            x = target_to_annaounce.x;
            y = target_to_annaounce.y;
            
            obj.draft.message.to='ALL';
            obj.draft.message.from = obj.identification_number;
            obj.draft.message.subject='CNET';
            obj.draft.message.message='TARGET';
            obj.draft.message.message_body=strcat(num2str(x) ,'|',num2str(y),'|' ,'static');
            
            obj.send_message(All_communicators )
            
        end

        function read_message(obj , All_communicators , arena)
            % (eg. : sample_communicator.read_message )
            
            % sample_communicator.inbox.message(k-th).to=             'All' OR i-th
            % sample_communicator.inbox.message(k-th).from =          i-th
            % sample_communicator.inbox.message(k-th).subject=        'CNET'
            % sample_communicator.inbox.message(k-th).message=        'TARGET' OR 'PROPOSAL' OR 'AWARD'
            % sample_communicator.inbox.message(k-th).message_body=   [TARGET: 'x|y|static'
            %                                                          PROPOSAL: 'x|y|cost'
            %                                                          AWARD: 'x|y|static']
            % obj.draft.message(k-th).message_read_flag=               [0 or 1]
            
            n = numel(obj.inbox.message); % number of messages in the inbox
            
            if n~=0
                for i =1:1: n
                    if obj.inbox.message(i).read_flag == 0
                        
                        sender = obj.inbox.message(i).from; % Who sent the message
                        body = obj.inbox.message(i).message_body;   % The message
                        
                        % Decomposing the message body and converting
                        % srtings data to number format
                        [x_str , remains] = strtok(body , '|');
                        x = str2double(x_str);
                        [y_str , remains] = strtok(remains , '|'); %#ok<STTOK>
                        y = str2double(y_str);
                        cost_str = strtok(remains , '|');
                        last_arg = str2double(cost_str);
                        
                        switch obj.inbox.message(i).message
                            
                            case 'TARGET'
                                
                                try
                                    k = numel(obj.map.destination); % number of target already in the destinations list
                                catch
                                    
                                    k = 0;
                                end
                                if k < 2 % It will send a proposal if and only if it has an empty slot
                                    
                                    % Preparing the list of targets for
                                    % which the cost will be evaluated
                                    targets_to_propose = [];
                                    
                                    try
                                        for p = 1:1:numel(obj.map.destination)
                                            
                                            targets_to_propose(p , 1) = obj.map.destination(p).x; %#ok<*AGROW>
                                            targets_to_propose(p , 2) = obj.map.destination(p).y;
                                            
                                        end
                                    catch
                                    end
                                    t = size(targets_to_propose , 1);
                                    targets_to_propose(t+1 ,1) = x;
                                    targets_to_propose(t+1 ,2) = y;
                                    
                                    % Calculating the cost
%                                     try
                                        if size(targets_to_propose , 1)==2
                                            
%                                             a = [2200, -1200;3138, -2311];
%                                             cost(a , 150 , 14)
                                            temp = cost(targets_to_propose , 150 , 14); % cost to track 2 targets
                                            cost_to_propose = temp(1);
                                            
                                            revisit_time = temp(2);
                                            
                                        else
                                            
                                            cost_to_propose = find_cost(obj , x , y);       % cost to track 1 target
                                            
                                        end
%                                     catch
%                                         cost_to_propose = find_cost(obj , x , y);
%                                     end
                                    
                                    variables.to =sender;   %: Wo won the tender
                                    variables.x = x;        %: x coordinates of Whom to track by the winner
                                    variables.y = y;        %: y coordinates of Whom to track by the winner
                                    variables.type = cost_to_propose;
                                    
                                    % Drafting a proposal message and
                                    % replying to the announcer
                                    obj.draft_and_reply('PROPOSAL' ,variables ,All_communicators)
                                    
                                else
                                    fprintf('UAV %d cant send a proposal as it doesnt have any empty slot \n' ,  obj.identification_number)
                                end
                                % Marking the message as read
                                obj.inbox.message(i).read_flag = 1;
                                
                            case 'AWARD'
                                
                                
                                try
                                    k = numel(obj.map.destination);
                                catch
                                    
                                    k = 0;
                                end
                                
                                if k <2 % It will add the awarded target if and only if it has 1 or 0 target already in the destination list
                                    
                                    if isempty(obj.map.destination(k).x)
                                        k =0;
                                    end
                                    for p = 1:1:numel(arena.target_positions.target)
                                        if arena.target_positions.target(p).x == x && arena.target_positions.target(p).track_flag ==0
                                            arena.target_positions.target(p).track_flag = 1;
                                        end
                                    end
                                    % Adding the target details in the destination list
                                    obj.map.destination(k+1).x =x ;
                                    obj.map.destination(k+1).y =y ;
                                    obj.map.destination(k+1).type ='static' ;
                                    fprintf('UAV %d got an award from UAV %d \n' , obj.identification_number , sender)
                                else
                                    fprintf('UAV %d got an award from %d but dont have empty slot \n' , obj.identification_number , sender)
                                    
                                end
                                % Marking the message as read
                                obj.inbox.message(i).read_flag = 1;
                            case 'PROPOSAL'
                                
                                for p = 1:1:numel(arena.target_positions.target)
                                    if arena.target_positions.target(p).x == x && arena.target_positions.target(p).track_flag ==0 &&...
                                            isempty(obj.proposal_read_parameters.x )
                                        % Reading the first proposal for a
                                        % given target and assigning its cost as minimum
                                        obj.proposal_read_parameters.cost = last_arg;
                                        obj.proposal_read_parameters.who = obj.inbox.message(i).from;
                                        obj.proposal_read_parameters.x = x;
                                        obj.proposal_read_parameters.y = y;
                                        
                                        % Starting the timer to wait for the proposals
                                        obj.wait_for_reply = 30;
                                    end
                                end
                                
                                if last_arg < obj.proposal_read_parameters.cost
                                    % changing the minimum cost if the cost
                                    % in the new message is less than the
                                    % cost in the previous mail
                                    obj.proposal_read_parameters.who = obj.inbox.message(i).from;
                                    
                                    obj.proposal_read_parameters.cost = last_arg;
                                    
                                end
                                % Marking the message as read
                                obj.inbox.message(i).read_flag = 1;
                        end
                        
                    end
                    
                    if obj.wait_for_reply <0
                        % If waiting timer is 0 or -ve declare the winner
                        variables.to = obj.proposal_read_parameters.who;      %: Wo won the tender
                        variables.x =obj.proposal_read_parameters.x;          %: x coordinates of Whom to track by the winner
                        variables.y =obj.proposal_read_parameters.y;          %: y coordinates of Whom to track by the winner
                        
                        for p = 1:1:numel(arena.target_positions.target)
                            
                            if arena.target_positions.target(p).x ==variables.x
                                fprintf('UAV %d won the contract \n' , variables.to)
                                % Draft and reply an AWARD message to the winner
                                obj.draft_and_reply('AWARD' , variables , All_communicators);
                                break
                            end
                        end
                        
                    else
                        obj.wait_for_reply = obj.wait_for_reply -1;
                    end
                end
                
            end
            
        end
        
        function draft_and_reply(obj , type_of_message ,variables , All_communicators)
            
            obj.refresh_map()
            obj.draft.message.from=obj.identification_number;
            
            switch type_of_message
                
                
                case 'PROPOSAL'
                    
                    %the anouncement message came in the inbox will be
                    %processed by the message processor which in turn tell
                    %to draft a proposal for the contract
                    
                    % the variable will then tell :
                    % variable.to :  who made the announcement
                    % variable.x    : the x position of the target
                    % variable.y     : the y position of the target
                    
                    obj.draft.message.to = variables.to;
                    
                    obj.draft.message.subject='CNET';
                    obj.draft.message.message='PROPOSAL';
                    % here type is equal to the cost
                    obj.draft.message.message_body =strcat(num2str(variables.x) ,'|' , num2str(variables.y ) ,'|' , num2str(variables.type));
                    fprintf('UAV %d wrote a proposal to UAV %d with cost %f \n', obj.identification_number , variables.to , variables.type)
                case 'AWARD'
                    
                    % this drafting will be based on the variables defined
                    % as :
                    
                    %variables.to   : Wo won the tender
                    %variables.x    : x coordinates of Whom to track by the winner
                    %variables.y    : y coordinates of Whom to track by the winner
                    
                    obj.draft.message.to = variables.to;
                    
                    obj.draft.message.subject='CNET';
                    obj.draft.message.message='AWARD';
                    fprintf('UAV %d wrote an award to UAV %d \n', obj.identification_number , variables.to)
                    obj.draft.message.message_body =strcat(num2str(variables.x) ,'|',num2str( variables.y),'|' ,'static');
                    
                    % Resetting the proposal_read_parameters to empty
                    obj.proposal_read_parameters.who =[];
                    obj.proposal_read_parameters.x = [];
                    obj.proposal_read_parameters.y = [];
                    obj.proposal_read_parameters.cost= [];
                    obj.wait_for_reply = 30;
            end
            
            obj.send_message(All_communicators)  
            % after drafting the AC is sending the message immediately in air(communication channel)
        end
        
        function send_message(obj , All_communicators)
            obj.communication_success(All_communicators);
            
            if strcmp(obj.draft.message.to,'ALL')
                % It the messsage is marked as to 'ALL'
                for i = 1:1:numel(All_communicators)
                    
                    if obj.communication_success_flag(i,1)==1 || i == obj.identification_number
                        % If the communication is successfull between the
                        % current communicator and the i-th communicator
                        
                        to = i;
                        recipient_communicator = All_communicators(to);
                        
                        size_of_the_inbox = numel(recipient_communicator.inbox.message);
                        
                        recipient_communicator.inbox.message(size_of_the_inbox+1).from = obj.draft.message.from;
                        recipient_communicator.inbox.message(size_of_the_inbox+1).subject = obj.draft.message.subject;
                        recipient_communicator.inbox.message(size_of_the_inbox+1).message = obj.draft.message.message;
                        recipient_communicator.inbox.message(size_of_the_inbox+1).message_body = obj.draft.message.message_body;
                        
                        recipient_communicator.inbox.message(size_of_the_inbox+1).read_flag = 0;
                        fprintf(' ......................message from UAV %d to UAV %d : delivered successfully \n' , obj.identification_number , i)
                    else
                        fprintf(' ......................message from UAV %d to UAV %d : failed \n' , obj.identification_number , i)
                    end
                end
                
                
            else
                to = obj.draft.message.to;
                
                if obj.communication_success_flag(obj.draft.message.to,1)==1 || to == obj.identification_number

                    recipient_communicator = All_communicators(to);
                    
                    size_of_the_inbox = numel(recipient_communicator.inbox.message);
                    
                    recipient_communicator.inbox.message(size_of_the_inbox+1).from = obj.draft.message.from;
                    recipient_communicator.inbox.message(size_of_the_inbox+1).subject = obj.draft.message.subject;
                    recipient_communicator.inbox.message(size_of_the_inbox+1).message = obj.draft.message.message;
                    recipient_communicator.inbox.message(size_of_the_inbox+1).message_body = obj.draft.message.message_body;
                    
                    recipient_communicator.inbox.message(size_of_the_inbox+1).read_flag = 0;
                    fprintf(' ......................message from UAV %d to UAV %d : delivered successfully \n' , obj.identification_number , to)
                else
                    fprintf(' ......................message from UAV %d to UAV %d : failed \n' , obj.identification_number , to)
                end
            end
            
            
            % logging the sent msgs in the sent folder
            
            size_of_the_sent= numel(obj.sent.message);
            
            obj.sent.message(size_of_the_sent+1).from = obj.draft.message.from;
            obj.sent.message(size_of_the_sent+1).to = obj.identification_number;
            obj.sent.message(size_of_the_sent+1).subject = obj.draft.message.subject;
            obj.sent.message(size_of_the_sent+1).message = obj.draft.message.message;
            obj.sent.message(size_of_the_sent+1).message_body = obj.draft.message.message_body;
            
            % deleting the drafted message
            obj.draft.message.from = [];
            obj.draft.message.to = [];
            obj.draft.message.subject = [];
            obj.draft.message.message = [];
            obj.draft.message.message_body = [];
            
        end
        
        function send_heart_beat(obj , All_communicators , to)
            obj.communication_success(All_communicators);
            % send a heart beat to UAV(to) 
            if obj.communication_success_flag(to,1)==1
                
                recipient_communicator = All_communicators(to);
                
                size_of_the_hb_inbox = numel(recipient_communicator.hb_inbox.hb);
                
                recipient_communicator.hb_inbox.hb(size_of_the_hb_inbox+1).from = obj.identification_number;
                
                recipient_communicator.hb_inbox.hb(size_of_the_hb_inbox+1).time  = obj.owner.hb.time;
                recipient_communicator.hb_inbox.hb(size_of_the_hb_inbox+1).pos.x  = obj.owner.hb.pos.x ;
                recipient_communicator.hb_inbox.hb(size_of_the_hb_inbox+1).pos.y  = obj.owner.hb.pos.y ;
                recipient_communicator.hb_inbox.hb(size_of_the_hb_inbox+1).pos.h  = obj.owner.hb.pos.h ;
                recipient_communicator.hb_inbox.hb(size_of_the_hb_inbox+1).speed   = obj.owner.hb.speed ;
                recipient_communicator.hb_inbox.hb(size_of_the_hb_inbox+1).hdg  = obj.owner.hb.hdg;
                recipient_communicator.hb_inbox.hb(size_of_the_hb_inbox+1).fp   = obj.owner.hb.fp ;
                
            end
            
        end
        
        function refresh_map (obj )
            
            % Update the map data with the owner's current position
            obj.map.current_location.x = obj.owner.state.x;
            obj.map.current_location.y = obj.owner.state.y;
            obj.map.current_location.z = obj.owner.state.h;
            
        end
        
        function communication_success(obj , All_communicators)
            % Evaluating the communication succes flags
            if isempty(obj.previous_communication_success_flag)
                obj.previous_communication_success_flag = ones(numel(All_communicators),1);
                
            end
            
            for i= 1:1:numel(All_communicators)
                
                
                vecBetUAVs = [obj.owner.state.x ; obj.owner.state.y;obj.owner.state.h] ...
                    -[All_communicators(i).owner.state.x;All_communicators(i).owner.state.y;All_communicators(i).owner.state.h];
                
                
                
                distBetUAVs = norm(vecBetUAVs);
                if distBetUAVs < obj.commsRange
                    % Check if the msg is received (random number check)
                    % ===================================================
                    % Case in which it was previously received
                    
                    
                    
                    if obj.previous_communication_success_flag(i,1)==1
                        randNum = rand(1);
                        if randNum < obj.pRecv_prevRecd
                            commsSuccess = 1;
                        else
                            commsSuccess = 0;
                        end
                        % Case in which it was previously lost
                    else
                        randNum = rand(1);
                        if randNum < obj.pRecv_prevLost
                            commsSuccess = 1;
                        else
                            commsSuccess = 0;
                        end
                    end
                    
                else
                    commsSuccess = 0;
                end
                
                obj.communication_success_flag(i,1) = commsSuccess;
                obj.previous_communication_success_flag(i,1) = commsSuccess;
                
            end
            
        end
        
    end
    
end

function cost = find_cost(obj , x, y)

% cost to track only one target

obj.refresh_map

cost = sqrt((obj.map.current_location.x-x)^2 +(obj.map.current_location.y-y)^2);

end



