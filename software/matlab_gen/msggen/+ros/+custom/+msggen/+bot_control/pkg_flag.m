classdef pkg_flag < ros.Message
    %pkg_flag MATLAB implementation of bot_control/pkg_flag
    %   This class was automatically generated by
    %   ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2021 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'bot_control/pkg_flag' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = '61d328f84af1c60f118e0cc0b21a5fae' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Dependent)
        BotNum
        LS
    end
    
    properties (Constant, Hidden)
        PropertyList = {'BotNum', 'LS'} % List of non-constant message properties
        ROSPropertyList = {'bot_num', 'LS'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = pkg_flag(msg)
            %pkg_flag Construct the message object pkg_flag
            import com.mathworks.toolbox.ros.message.MessageInfo;
            
            % Support default constructor
            if nargin == 0
                obj.JavaMessage = obj.createNewJavaMessage;
                return;
            end
            
            % Construct appropriate empty array
            if isempty(msg)
                obj = obj.empty(0,1);
                return;
            end
            
            % Make scalar construction fast
            if isscalar(msg)
                % Check for correct input class
                if ~MessageInfo.compareTypes(msg(1), obj.MessageType)
                    error(message('ros:mlros:message:NoTypeMatch', obj.MessageType, ...
                        char(MessageInfo.getType(msg(1))) ));
                end
                obj.JavaMessage = msg(1);
                return;
            end
            
            % Check that this is a vector of scalar messages. Since this
            % is an object array, use arrayfun to verify.
            if ~all(arrayfun(@isscalar, msg))
                error(message('ros:mlros:message:MessageArraySizeError'));
            end
            
            % Check that all messages in the array have the correct type
            if ~all(arrayfun(@(x) MessageInfo.compareTypes(x, obj.MessageType), msg))
                error(message('ros:mlros:message:NoTypeMatchArray', obj.MessageType));
            end
            
            % Construct array of objects if necessary
            objType = class(obj);
            for i = 1:length(msg)
                obj(i,1) = feval(objType, msg(i)); %#ok<AGROW>
            end
        end
        
        function botnum = get.BotNum(obj)
            %get.BotNum Get the value for property BotNum
            botnum = int16(obj.JavaMessage.getBotNum);
        end
        
        function set.BotNum(obj, botnum)
            %set.BotNum Set the value for property BotNum
            validateattributes(botnum, {'numeric'}, {'nonempty', 'scalar'}, 'pkg_flag', 'BotNum');
            
            obj.JavaMessage.setBotNum(botnum);
        end
        
        function ls = get.LS(obj)
            %get.LS Get the value for property LS
            ls = int16(obj.JavaMessage.getLS);
        end
        
        function set.LS(obj, ls)
            %set.LS Set the value for property LS
            validateattributes(ls, {'numeric'}, {'nonempty', 'scalar'}, 'pkg_flag', 'LS');
            
            obj.JavaMessage.setLS(ls);
        end
    end
    
    methods (Access = protected)
        function cpObj = copyElement(obj)
            %copyElement Implements deep copy behavior for message
            
            % Call default copy method for shallow copy
            cpObj = copyElement@ros.Message(obj);
            
            % Create a new Java message object
            cpObj.JavaMessage = obj.createNewJavaMessage;
            
            % Iterate over all primitive properties
            cpObj.BotNum = obj.BotNum;
            cpObj.LS = obj.LS;
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.BotNum = strObj.BotNum;
            obj.LS = strObj.LS;
        end
    end
    
    methods (Access = ?ros.Message)
        function strObj = saveobj(obj)
            %saveobj Implements saving of message to MAT file
            
            % Return an empty element if object array is empty
            if isempty(obj)
                strObj = struct.empty;
                return
            end
            
            strObj.BotNum = obj.BotNum;
            strObj.LS = obj.LS;
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.custom.msggen.bot_control.pkg_flag.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = ros.custom.msggen.bot_control.pkg_flag;
            obj.reload(strObj);
        end
    end
end
