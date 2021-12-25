classdef CustomMsgConsts
    %CustomMsgConsts This class stores all message types
    %   The message types are constant properties, which in turn resolve
    %   to the strings of the actual types.
    
    %   Copyright 2014-2021 The MathWorks, Inc.
    
    properties (Constant)
        bot_control_Bot_status = 'bot_control/Bot_status'
        bot_control_Bot_task = 'bot_control/Bot_task'
        bot_control_CompletePlan = 'bot_control/CompletePlan'
        bot_control_PathArray = 'bot_control/PathArray'
        bot_control_Poses = 'bot_control/Poses'
        bot_control_StartGoal = 'bot_control/StartGoal'
        bot_control_dest_id = 'bot_control/dest_id'
        bot_control_pkg_flag = 'bot_control/pkg_flag'
    end
    
    methods (Static, Hidden)
        function messageList = getMessageList
            %getMessageList Generate a cell array with all message types.
            %   The list will be sorted alphabetically.
            
            persistent msgList
            if isempty(msgList)
                msgList = cell(8, 1);
                msgList{1} = 'bot_control/Bot_status';
                msgList{2} = 'bot_control/Bot_task';
                msgList{3} = 'bot_control/CompletePlan';
                msgList{4} = 'bot_control/PathArray';
                msgList{5} = 'bot_control/Poses';
                msgList{6} = 'bot_control/StartGoal';
                msgList{7} = 'bot_control/dest_id';
                msgList{8} = 'bot_control/pkg_flag';
            end
            
            messageList = msgList;
        end
        
        function serviceList = getServiceList
            %getServiceList Generate a cell array with all service types.
            %   The list will be sorted alphabetically.
            
            persistent svcList
            if isempty(svcList)
                svcList = cell(0, 1);
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            serviceList = svcList;
        end
        
        function actionList = getActionList
            %getActionList Generate a cell array with all action types.
            %   The list will be sorted alphabetically.
            
            persistent actList
            if isempty(actList)
                actList = cell(0, 1);
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            actionList = actList;
        end
    end
end
