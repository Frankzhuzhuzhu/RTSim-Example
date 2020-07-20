classdef RTRobot < RTSim
    %RTRobot Controller for RTSim real-time robot simulation.
    %    https://github.com/FJFranklin/wifi-py-rpi-car-controller/tree/master/RTSim
    
    properties
        % list of global (class) variables used by setup(), loop() and ping_receive()
        target
        last_ping_time
        last_ping_distance
        position
        orientation
        step         
        last_print 
        
        barriers   % Added (should add a function in RTSim.m 
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Added%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %         function barr = get_barriers(obj)
        %             barr = obj.barriers;
        %         end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        
        
      
    end
    
    methods
        function obj = RTRobot (seconds, test_name)
            % This is the Matlab version of the coursework 'Matlab Robot':
            % In the following line, replace the number with your Student ID
            id_number = 190142346;  % Cai Xiao Gou

            if (nargin() < 1) % default to 180s (3 minutes)
                seconds = 180;
            end
            if (nargin() < 2) % default to simple layout
                test_name = 'default';
                % other options are: 'random', 'TNT', 'CWC' & 'BSB'
            end
            obj@RTSim (seconds, test_name, id_number);
            
        end
        
        
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%       
        
        % Let's create a function that tells us whether it's okay to print
        % It returns true only once every quarter of a second
        % Otherwise we'd be printing so much it would slow the program down
        % (the robot also) and would be almost unreadable.
        function result = CanPrint (obj)
            currentTime = obj.millis ();

            if currentTime - obj.last_print > 250
                obj.last_print = currentTime;
                result = true;
            else
                result = false;
            end
        end
        
        % Make sure an angle is between -180 and 180:
        function fixed = FixAngle (obj, angle)
            if angle > 180
                fixed = angle - 360;
            elseif angle < -180
                fixed = angle + 360;
            else
                fixed = angle;
            end
        end
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Find the closest target then to track
        function closest_target = Findtarget (obj)
            
            closest_target = [];
            barriers = obj.barriers;
            
            % Remove irrelevant barriers
            count = size(barriers,1);
            for c = 1:count
                %Remove walls
                if barriers(c,3) == 10 || barriers(c,4) == 10
                    barriers(c,:) = 0;
                end
                %Remove the only horizontal barrier if the map is not default
                if barriers(c,4) < 1 && count == 11
                    barriers(c,:) = 0;
                end
                %Remove the vertical barriers with the same X-axis
                for d = 1:count
                    if c ~= d && barriers(c,1) == barriers(d,1) && barriers(c,3) < 1 && barriers(d,3) < 1
                        if barriers(c,2) == -5
                            barriers(c,:) = 0;
                        elseif barriers(d,2) == -5
                            barriers(d,:) = 0;
                        end
                    end
                end
            end
            %Remove the unnecessary barriers (reduce the barriers size)
            barriers(all(barriers == 0,2),:) = [];
            
            %Let's decide the intermediate target points ***************
            count = size(barriers,1);
            
            % Sort rows of the bariers matrix
            barriers = sortrows(barriers,1);
            
            %Two situations 1.default map 2.random map
            %For a random map
            if count == 4
                for c = 1:count
                    % Four points of one barrier
                    A(1,:) = [barriers(c,1), barriers(c,2)];
                    A(2,:) = [barriers(c,1) + barriers(c,3), barriers(c,2)];
                    A(3,:) = [barriers(c,1) + barriers(c,3), barriers(c,2) + barriers(c,4)];
                    A(4,:) = [barriers(c,1), barriers(c,2) + barriers(c,4)];
                    
                    %Two side targets
                    if c == 1 || c == 4
                        closest_target(c,:) = A(1,:);
                        closest_target(c,2) = closest_target(c,2) - 0.6;
                        
                        %The middle targets (there are two points here,need to consider which one is better later)
                    elseif c == 2
                        closest_target(c,:) = A(4,:);
                        closest_target(c,2) = closest_target(c,2) + 0.5;
                    elseif c == 3
                        closest_target(c,:) = A(1,:);
                        closest_target(c,2) = closest_target(c,2) - 0.5;
                    end
                end
                
                %For the default map
            elseif count == 3
                for c = 1:count
                    
                    % Four points of one barrier
                    A(1,:) = [barriers(c,1), barriers(c,2)];
                    A(2,:) = [barriers(c,1) + barriers(c,3), barriers(c,2)];
                    A(3,:) = [barriers(c,1) + barriers(c,3), barriers(c,2) + barriers(c,4)];
                    A(4,:) = [barriers(c,1), barriers(c,2) + barriers(c,4)];
                    
                    %Left hand
                    if c == 1
                        closest_target(c,:) = A(1,:);
                        closest_target(c,2) = closest_target(c,2) - 0.6;
                        %Middle
                    elseif c == 2
                        closest_target(c,:) = A(2,:);
                        closest_target(c,1) = closest_target(c,1) + 0.6;
                        %Right hand
                    elseif c == 3
                        closest_target(c,:) = A(4,:);
                        closest_target(c,2) = closest_target(c,2) + 0.6;
                    end
                end
            end
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
              
                             
        
        function setup (obj) % setup() is called once at the beginning
            obj.target = obj.get_target ();       % where we are trying to get to
            
            % For example:
            obj.last_ping_time = 0;               % see ping_receive()
            obj.last_ping_distance = -1;
            
            
            
            
            
            %    obj.set_wheel_speeds (-127,-127);
            
            % variables need to be given initial values
            % setup() is the best place to do this
            obj.last_print = 0;
            
            [~,~,trial,~] = obj.get_result (true);
            if strcmp (trial, 'default')
                disp ('This is the default map.')
                obj.step = 1;
            elseif strcmp (trial, 'random')
                disp ('This is a random map.')
                obj.step = 1; % use a flexible journey logic
            else
                disp ('This is my personal map.')
                obj.step = 1; % use a fast journey logic
            end
            
        end
        
        
        
        function loop (obj)
            % loop() is called repeatedly
            
            % For example:
            currentTime = obj.millis () / 1000;
            
            obj.target = obj.get_target ();
            obj.position = obj.get_GPS ();        % roughly where we are
            obj.orientation = obj.get_compass (); % which direction we are looking
            
            if (currentTime > 5)
                obj.ping_send ();                 % it will not actually send more often than every 0.1s
            end
            
            obj.barriers = obj.get_barriers;
            Target = obj.Findtarget();
            
            %%For the random map, decide to delete one target in the middle, i choose to delete the top one
%             if size(Target,1) == 4
%                 if Target(1,2) > Target(3,2) && Target(2,1) - Target(1,1) > Target(4,1) - Target(3,1)
%                     n = 2;
%                 else
%                     n = 3;
%                 end
%             end
            

           %%% Calculate distance to decide the target point
           if size(Target,1) == 4
               if abs(norm(Target(1,:) - Target(2,:))) + abs(norm(Target(2,:) - Target(4,:))) > abs(norm(Target(1,:) - Target(3,:))) + abs(norm(Target(3,:) - Target(4,:)))
                   n = 3;
               else
                   n = 2;
               end
           else
                n = 1;
           end



            %%% n = 2, Target 2 is the mid target point
            %%% n = 3, Target 3 is the mid target point
            %%% n = 1, default map
            %Target 2 
            if n == 2
                if Target(1,1) - obj.position(1) > 0.2
                    T = Target(1,:);
                elseif Target(2,1) - obj.position(1) > 0.2
                    T = Target(2,:);
                    if obj.last_ping_distance == -1
                        obj.step = 2;
                    end
                elseif Target(4,1) - obj.position(1) > 0.2
                    T = Target(4,:);
                    if obj.last_ping_distance == -1
                        obj.step = 2;
                    end
                else
                    T = obj.target;
                    if obj.last_ping_distance == -1
                        if Target(4,2) < T(2)
                            obj.step = 3;
                        else
                            obj.step = 2;
                        end
                    end
                end
            %Target 3
            elseif n == 3
                if Target(1,1) - obj.position(1) > 0.2
                    T = Target(1,:);
                elseif Target(3,1) - obj.position(1) > 0.2
                    T = Target(3,:);
                    if obj.last_ping_distance == -1
                        obj.step = 3;
                    end
                elseif Target(4,1) - obj.position(1) > 0.2
                    T = Target(4,:);
                    if obj.last_ping_distance == -1
                        obj.step = 2;
                    end
                else
                    T = obj.target;
                    if obj.last_ping_distance == -1
                        if Target(4,2) < T(2)
                            obj.step = 3;
                        else
                            obj.step = 2;
                        end
                    end
                end
                %For default map
                elseif n == 1
                if Target(1,1) - obj.position(1) > 0.2
                    T = Target(1,:);
                elseif Target(2,1) - obj.position(1) > 0.2
                    T = Target(2,:);
                    if obj.last_ping_distance == -1
                        obj.step = 3;
                    end
                elseif Target(3,1) - obj.position(1) > 0.1
                    T = Target(3,:);
                    if obj.last_ping_distance == -1
                        obj.step = 2;
                    end
                else
                    T = obj.target;
                    if obj.last_ping_distance == -1
                        if Target(3,2) < T(2)
                            obj.step = 3;
                        else
                            obj.step = 2;
                        end
                    end
                end
            end
            % Can use it to see the position of the target
            %       disp(T);
            
            if obj.step == 1
                % Step 1: Adjust the aim of the robot - robotate on the spot
                %(Just copy from Franklin)
                
                % The vector from the robot to the target is:
                V = T - obj.position;
                % The compass direction to the target is:
                A = 90 - atan2d(V(2),V(1));
                % The difference in orientation is:
                O = obj.FixAngle(A - obj.orientation);
                % Let's rotate on the spot:
                obj.set_wheel_speeds (O/2, -O/2);
                
                if obj.CanPrint()
                    fprintf( 'Step %d: misorientation=%f\n' , obj.step, O)
                end
                
                % If we're orientated more-or-less correctly, continue:
                if abs(O) < 5
                    %Decide the position of the sonar
                    if obj.orientation < 90   %Right hand side
                        obj.step = 2;
                    elseif obj.orientation > 90 % Left hand side
                        obj.step = 3;
                    end
                end
                
            % Step 2&3: Head for the target - *with* course correction
            elseif obj.step == 2
                %Sonar right hand side
                obj.set_ping_angle (35);
                
                % The vector from the robot to the target is:
                V = T - obj.position;
                % The distance to the target is:
                D = norm(V);
                % The compass direction to the target is:
                A = 90 - atan2d(V(2),V(1));
                
                % The difference in orientation is:
                O = obj.FixAngle(A - obj.orientation);
                
                % Let's curve slightly to correct course:
                obj.set_wheel_speeds (125+O, 125-O);
                if obj.CanPrint()
                    fprintf('Step %d: distance to target=%f\n', obj.step, D)
                end
                % If we're more-or-less near, continue:
%                 if D < 0.5
%                     obj.step = 1;
%                 end
                
            elseif obj.step == 3
                %Sonar left hand side
                obj.set_ping_angle (-35);
                
                % The vector from the robot to the target is:
                V = T - obj.position;
                % The distance to the target is:
                D = norm(V);
                % The compass direction to the target is:
                A = 90 - atan2d(V(2),V(1));
                
                % The difference in orientation is:
                O = obj.FixAngle(A - obj.orientation);
                
                % Let's curve slightly to correct course:
                obj.set_wheel_speeds (125+O, 125-O);
                if obj.CanPrint()
                    fprintf('Step %d: distance to target=%f\n', obj.step, D)
                end
                % If we're more-or-less near, continue:
%                 if D < 0.5
%                     obj.step = 1;
%                 end
                
            %Step 4&5:Keep away from the wall if detected
            elseif obj.step == 4
                %Right hand side wall
                
                obj.set_ping_angle (15);
                obj.set_wheel_speeds (110,127);
                
                if obj.CanPrint()
                    fprintf( 'Step %d:' , obj.step)
                end
                
            elseif obj.step == 5
                %Left hand side wall
                
                obj.set_ping_angle (-15);
                obj.set_wheel_speeds (127,110);
                
                if obj.CanPrint()
                    fprintf( 'Step %d:' , obj.step)
                end
            end
        end
        

        function ping_receive (obj, distance)
            % response to an obj.ping_send ()

            %If sonar receive a value, then try to aviod this obstacle
            if distance >= 0
                %Right hand side wall
                if obj.step == 2
                    obj.step = 4;
                %Left hand side wall
                elseif obj.step == 3
                    obj.step =5;
                end
            end
             
            %If no obstacle detected, continue the previous movement
            if distance == -1 
                if obj.step == 4
                    obj.step = 2;
                    
                elseif obj.step == 5
                    obj.step = 3;
                end
            end
            

        
            
            % For example:
            obj.last_ping_time = obj.millis ();   % the last time we received a ping [in milliseconds]
            obj.last_ping_distance = distance;    % distance measured (-ve if no echo)

%            if (distance >= 0)                    % a -ve distance implies nothing seen
%                 disp (['position=(',num2str(obj.position(1)),',',num2str(obj.position(2)),...
%                        '), orientation=',num2str(obj.orientation),...
%                        '; distance=',num2str(distance)]);
%             end
        end
    end
end

