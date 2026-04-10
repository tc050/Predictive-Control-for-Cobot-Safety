classdef customRobotMapValidator < nav.StateValidator

    properties
        Robot % rigidBodyTree
        Map % occupancyMap3D
        ValidationDistance % sampling resolution
        BodyNames % body names of links
    end

    methods
        function obj = customRobotMapValidator(ss, robot, map)
            obj@nav.StateValidator(ss);

            obj.Robot = robot;
            obj.Map = map;
            obj.ValidationDistance = 0.05; % default value
            obj.BodyNames = robot.BodyNames;
        end

        function isValid = isStateValid(obj, q)
            % q can be Nx7 or 1x7 (or 7x1)
            if size(q,1) > 1 % Nx7 input
                % if multiple states, validate each row
                N = size(q, 1);
                isValid = true; % assume valid

                for k = 1:N
                    if ~obj.isStateValidSingle(q(k,:)')
                        isValid = false;
                        return
                    end
                end
                return
            else
                % single state
                isValid = obj.isStateValidSingle(q);
            end
        end

        function isValid = isStateValidSingle(obj, q) 
            % ensure joints are a column vector
            q = q(:);

            % loop through bodies
            for i = 1:numel(obj.BodyNames)
                bodyName = obj.BodyNames{i};

                % get transform
                T = getTransform(obj.Robot, q, bodyName);

                % obtain XYZ positional coordinates
                pos = T(1:3,4)';

                % check if occupancy at joint position
                if getOccupancy(obj.Map, pos) >= obj.Map.OccupiedThreshold
                    isValid = false;
                    return;
                end

                % sample link geometry (not the base)
                if i > 1
                    prevT = getTransform(obj.Robot, q, obj.BodyNames{i-1});
                    prevPos = prevT(1:3,4)';

                    % interpolate along link
                    dist = norm(pos-prevPos);
                    nSamples = max(2, ceil(dist / obj.ValidationDistance));

                    for j = 0:nSamples
                        alpha = j / nSamples;
                        p = (1 - alpha) * prevPos + alpha * pos;

                        if getOccupancy(obj.Map, p) >= obj.Map.OccupiedThreshold
                            isValid = false;
                            return;
                        end
                    end
                end
            end

            isValid = true;
        end

        function isValid = isMotionValid(obj, q1, q2)
            dist = norm(q2 - q1);
            nSteps = ceil(dist / obj.ValidationDistance);

            for i = 0:nSteps
                alpha = i / nSteps;
                q = (1 - alpha) * q1 + alpha * q2;

                if ~obj.isStateValid(q)
                    isValid = false;
                    return;
                end

                isValid = true;
            end
        end

        function newObj = copy(obj)
            % create a new instance
            newObj = customRobotMapValidator(obj.StateSpace, obj.Robot, obj.Map);

            % copy properties
            newObj.ValidationDistance = obj.ValidationDistance;
        end
    end
end