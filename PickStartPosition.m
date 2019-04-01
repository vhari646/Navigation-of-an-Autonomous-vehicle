function [scaledPose,mapScale,xOffset,yOffset] = ...
    PickStartPosition(ref, map, customScaling)

% PickStartPosition  Lets the user pick a position and orientation on a 
%                    map made from MapCreator and will receive the scaled 
%                    pose back.
%
% CALL SEQUENCE: 
% [scaledPose,mapScale,xOffset,yOffset] = 
%           PickStartPosition(ref, map, customScaling)
%
% INPUT:
%   ref           Reference image path, e.g. 'TFE_Level2_GrayScale.png'.
%   map           Input map file, e.g. 'MyMap.txt'.
%   customScaling (optional) Lets the user set a custom scaling.
%   
% OUTPUT:
%   scaledPose  Scaled pose of the chosen position and orientation
%   mapScale    Scaling of the map to the real world
%   xOffset     X offset of the map
%   yOffset     Y offset of the map
%
% USAGE INSTRUCTIONS:
%   Press LEFT MOUSE BUTTON to interact with the program. The first time
%   you press it, you will choose position. The second time you press it,
%   you will choose orientation, which will be the angle in the direction
%   from the chosen position to the location of the mouse when LEFT MOUSE 
%   BUTTON is pressed.
%
% USAGE EXAMPLE: 
%   [scaledPose, mapScale, ~, ~] = ...
%       PickStartPosition('TFE_Level2_GrayScale.png', 'MyMap.txt');
%   
%   oRobot = MobileRobot(0, 'MyMap.txt', mapScale, 0, 0, scaledPose);
%
%   fig = figure
%   hold on
%   plot(oRobot.oLidar.mWorldX,oRobot.oLidar.mWorldY,'k')
%   plot(scaledPose(1),scaledPose(2),'b*');
%   quiver(scaledPose(1),scaledPose(2), ...
%       2*cos(scaledPose(3)), 2*sin(scaledPose(3)), 'b');
%
% PROGRAMMING by Henrik Soderlund (henrik.soderlund@umu.se)
%   2018-12-01	 Initial programming

% Define button values for ginput
LEFT_MOUSE  = 1;

% Define maximum number of chains
maxChains = 9;

% Preallocate memory for chains
chains = zeros(2*maxChains,round(10000/maxChains));

% Define number of elements in each chain
j = zeros(maxChains,1);

% Define default outputs
scaledPose = zeros(1,3);
mapScale   = 15/274;
xOffset    = 0;
yOffset    = 0;

% Pick custom scaling if defined
if exist('customScaling','var')
    mapScale = customScaling;
end

% Define step variable
step = 0;
% Define finished indicator
isDone = false;

% Check if using MATLAB R2017a or lower
isOldVersion = false;
oldDate = 2017;
curVersion = version;
if isempty(strfind(curVersion, '2017b'))
    for i = oldDate:-1:2006
        if ~isempty(strfind(curVersion, num2str(i)))
            isOldVersion = true;
            break;
        end
    end
end

% Use the correct way to check if a file exists depending on MATLAB version
if (isOldVersion == true)
    fileExist = exist([pwd, '\\', map], 'file') == 2;
else
    fileExist = isfile(map);
end

% Check if output file already exists, if so, load it.
if (fileExist == true)
    try
        % Read file
        M = dlmread(map);
        % Iterate through each possible chain
        for i = 1:maxChains
            % Find the current chain in the existing map
            curM = M(1:2, M(3,:) == i);
            % If there exists a chain i in the map
            if (~isempty(curM))
                % Extract the coordinates
                numM = numel(curM(1,:));
                % And store them in the chain matrix
                chains((i*2-1):(i*2), 1:numM) = curM;
                j(i) = numM;
            end
        end
    catch
        % Could not read the file for some reason
        disp(['Could not read file ', out, '. Cancelling...']);
        isDone = true;
    end
else
    disp(['File ', out, ' does not exist! Cancelling...']);
    isDone = true;
end

% Create a new figure
fig = figure;
% Set background color as white
set(fig, 'Color', [1,1,1]);
% Set axes to be equal
axis equal;

% Load reference image
ref_img = imread(ref);

% Begin loop
while (isDone == false)
    
    % Clear figure
    clf
    % Show the reference map in the figure
    imshow(ref_img);
    
    % Show x and y axis
    axis on
    
    % Allow lines to be plotted on the image
    hold on
    
    % Plot dashed lines and points of chains
    for i = 1:maxChains
            % Get number of elements in current chain
            js_i = j(i);
            % Define chain i row indices
            rs_i = (i*2-1):(i*2);
            
            % Only plot non-empty chains
            if (js_i > 0)
                % Plot dashed lines from point to point
                plot(chains(rs_i(1),1:js_i),chains(rs_i(2),1:js_i), ...
                    'r-.','LineWidth',2);
                % Plot points as black dots
                plot(chains(rs_i(1),1:js_i),chains(rs_i(2),1:js_i), ...
                    'r.','MarkerSize',12);
            end
    end
    
    % Plot chosen position
    if (step > 0)
        
        % Plot chosen position
        plot(scaledPose(1),scaledPose(2), 'b*','MarkerSize',12);

    end
    
    % Get new input from the user
    [x,y,pressedButton] = ginput(1);
    
    if (pressedButton == LEFT_MOUSE)
        switch (step)
            % Pick position
            case 0
            
                % Store position in pixels temporarily
                scaledPose = [x, y, 0];
                % Next step
                step = 1;

            % Pick orientation 
            case 1

                % Store computed angle
                scaledPose(3) = atan2(-(y-scaledPose(2)),x-scaledPose(1));
                % We are done!
                isDone = true;
        end
    end
end

% Scale the output pose
scaledPose = [scaledPose(1)*mapScale+xOffset, ...
             -scaledPose(2)*mapScale+yOffset, ...
              scaledPose(3)];
         
% Close figure
close(fig)