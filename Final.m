clear all
close all
javaaddpath('ProjectJarV2.jar');
sMapName='TFE_coordinates2.txt'
sMapName='MyMap.txt'
% rScale = 15/575;
% xOffSet=-10;
% yOffset=-10;
% vStartPose = [6,8,30.14];
% bRealTime = 1>0   % Makes the simulator to run in thread.
% oRobot = MobileRobot(bRealTime, sMapName, 60/575,-20,-15,vStartPose);     % Set expression to 1<0 to not run robot in realtime
[scaledPose,mapScale,xOffset,yOffset] =PickStartPosition('TFE_Level2_GrayScale.png', 'MyMap.txt');
bRealTime = 1<0   % Makes the simulator to run in thread.
oRobot = MobileRobot(bRealTime, sMapName, mapScale ,xOffset,yOffset,scaledPose);     % Set expression to 1<0 to not run robot in realtime

nLen=1600;
mPose = zeros(3,nLen);
kPose = zeros(3,nLen);
yPose = zeros(3,nLen);
vTime = zeros(1,nLen);
Current_pose = zeros(3,nLen);
vOmegaVelocity = zeros(2,nLen);

vCos = cos([-135:1:135]*pi/180); 
vSin = sin([-135:1:135]*pi/180);

rSpeed = 1;
rAlpha=15*pi/180;

vInput =[rAlpha, rSpeed];
oRobot.setInput([vInput]);

xC=cos([0:20]*2*pi/(20*180));
yC=sin([0:20]*2*pi/(20*180));

maxLidarRange = 8;
mapResolution = 20;

slamAlg = robotics.LidarSLAM(mapResolution, maxLidarRange);
% slamAlg.MovementThreshold = [0.6 0.2];
slamAlg.LoopClosureThreshold = 210;  
slamAlg.LoopClosureSearchRadius = 8;
fig = figure(77);
k=1;
target1 = 0;
target2 = 0;
p = [8.7 -1.22];
s = [35.4 -10.54];
t = [31.44 -9.525];
ss = [7.73 -4.27];
for n=1:nLen
% 	if(n==50)
% 	  vInput=[0,1.0]
%     end
    
%     set(fig,'KeyPressFcn',@KeyPressCb);
%     rAlpha
%     rSpeed
%     vInput=[rAlpha,rSpeed];
% 	oRobot.setInput(vInput);

%     oRobot.setInput(vInput);
	% Implement your dead-reckoning code here
	
        %	vPose = [0,0,0]';
    
    
    
    
    oCarMeas = oRobot.oGetMeasurements
    
    vScan = oCarMeas.vGetScan;          %% Laser scan
    rScanTime = oCarMeas.rGetScanTime   %% Scan time
    
    rTime = oCarMeas.rGetTime;          %% dead-reckoning time
    iIndex = oCarMeas.iGetIndex         %% the current epoch of data
    
    rOmega =  oCarMeas.rGetOmega;       %% measured angular velocity
    rSpeed =oCarMeas.rGetSpeed;    %% Measured wheel speed
   
    if(mod(n,1)==0)
        figure(77);
        clf
        xScan= vCos'.*vScan;
        yScan= vSin'.*vScan;
        
        if target1 == 0&&target2 == 0
            rSpeed=0.5;       
            kt=0;
            rAlpha=VHFtest_3(xScan,yScan,kt);
            if rAlpha==0
                rSpeed=1.5;
            end
        elseif target1 == 1&&target2 == 0
            rAlpha=VHFtest_4(xScan,yScan,0);
            rSpeed=0.2*sqrt((cPose(1)-t(1))^2+(cPose(2)-t(2))^2);
        elseif target1 == 1&&target2 == 1
            rAlpha=VHFtest_4(xScan,yScan,0);
            rSpeed=0.2*sqrt((cPose(1)-p(1))^2+(cPose(2)-p(2))^2);
            if rSpeed<0.3
                rSpeed=0;
%                 angdif = atan((cPose(2)-ss(2))/(cPose(1)-ss(1))) - cPose(3);
%                 kt = rad2deg(angdif);
%                 rAlpha=VHFtest_4(xScan,yScan,kt);
%                  rAlpha=35*pi/180;
            end
          
         end
%         rAlpha=oba(vScan);
        
        cPose = oCarMeas.vGetPose;
        if sqrt((cPose(1)-s(1))^2+(cPose(2)-s(2))^2)<2
            target1 = 1;
        end
        if sqrt((cPose(1)-t(1))^2+(cPose(2)-t(2))^2)<3
            target2 = 1;
        end
        disi = sqrt((cPose(1)-mPose(1,1))^2+(cPose(1)-mPose(2,1))^2);
        fprintf('disi is %d',disi);
%         if disi <10
%             rAlpha = 0;
%             rSpeed = 0;
%         end
        vInput=[rAlpha, rSpeed];
        oRobot.setInput(vInput);
        plot(xScan,yScan,'k.');
        vScanSLAM(:,:,k)=[xScan, yScan];
        grid on
        axis([-30,30,-20,20]);
        axis equal
        
        hold on
        %	plot(oRobot.vX,oRobot.vY,'k-');t
        title(sprintf('Epoch: %d  Time:%.2f',n, rTime));
        xlabel(' x_{lidar} (m)');
        ylabel(' y_{lidar} (m)');
        fill([-.8,-0.8,0,-0.8],[-0.5,0.5,0,-0.5],'r-')
        plot(xC,yC,'m-');
        drawnow
        k=k+1;
        findcorridor(vScan,xScan,yScan)
    end
    vPose = oCarMeas.vGetPose;
    if n==1
        yPose = oCarMeas.vGetPose;
    else
        delta = rTime-vTime(1,n-1);
        [yPose]=kalmanfilter(rSpeed,rOmega,Current_pose,n,delta,vOmegaSpeed);
    end
    vTime(1,n)= rTime;
    mPose(:,n) = cPose;
    Current_pose(:,n) = yPose;
    vOmegaSpeed(:,n) = [rOmega, rSpeed];
end
% tic
% for i=1:n
%     [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, lidarScan([vScanSLAM]));
%     if isScanAccepted
%         fprintf('Added scan %d \n', i)
%     end
%     i
% end
% toc

firstTimeLCDetected = false;
tic

for i=1:n
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, lidarScan([vScanSLAM(:,1,i) vScanSLAM(:,2,i)]));
    if isScanAccepted
        fprintf('Added scan %d \n', i)
    end
    if optimizationInfo.IsPerformed && ~firstTimeLCDetected
        show(slamAlg, 'Poses', 'off');
        hold on;
        show(slamAlg.PoseGraph); 
        hold off;
        firstTimeLCDetected = true;
        drawnow
    end
    i
    [scans, optimizedPoses]  = scansAndPoses(slamAlg);
    map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);
end
toc
 
figure;
show(slamAlg);
title({'Map of the Environment','Pose Graph for Scans'});
 
figure; 
show(map);
hold on
show(slamAlg.PoseGraph, 'IDs', 'off');
hold off
title('Occupancy Grid Map Built Using Lidar SLAM');

figure
plot(mPose(1,:), mPose(2,:),'k-');
xlabel('x(m)');
ylabel('y(m)');
axis equal
grid on
print('-dpng','TeleroboticsL_Pose.png')

figure
plot(Current_pose(1,:), Current_pose(2,:),'k-');
xlabel('x(m)');
ylabel('y(m)');
title("implemented")
axis equal
grid on
print('-dpng','TeleroboticsLab2_Pose.png')

% figure
% plot(kPose(1,:), kPose(2,:),'k-');
% xlabel('x(m)');
% ylabel('y(m)');
% axis equal
% grid on
% figure
% plot(yPose(1,:), yPose(2,:),'k-');
% xlabel('x(m)');
% ylabel('y(m)');
% axis equal
% grid on
% print('-dpng','TeleroboticsL_Pose.png')
figure
subplot(2,1,1);
plot(vTime, vOmegaSpeed(1,:));
xlabel('Time (s)');
ylabel('\omega (rad/s)');
title('Angular Velocity');
subplot(2,1,2);
plot(vTime,vOmegaSpeed(2,:));
xlabel('Time (s)');
ylabel('Speed (m/s)');
print('-dpng','TeleroboticsLab2_OmegaVel.png')