t = 0:2*pi/100:2*pi;

lissajous = @(m,n,d) sin(m*t) + 1i*cos(n*t+pi*d);

f = lissajous(5,3,0);
%plot(real(f), imag(f),'color',[0.00,0.45,0.74], 'linewidth', 0.7, 'marker', '*')

xl = real(f);
ylk = imag(f);

length = 2;
r = 20*length/(2*pi);

theta1 = xl*2*pi/length;

k = 1;
for i=1:101
    
  if(abs(theta1(i)- 3.1416) > 0)
      theta(k) = theta1(i);
      yl(k) = ylk(i);
      k = k + 1;
  end 
end

x = 0; y = 0; z = 0;

x = r*sin(theta);
y = r*cos(theta);
z = yl;

path(1,:) = [0 0 0];
path(2:102,:) = [x' y'-4-2.3662 z'+2];
thetad = wrapToPi([0 theta]' + pi);

% Connect to ROS master
rosshutdown;
turtlebotIp = '192.168.247.138';
rosinit(turtlebotIp);
% Create ROS subscribers and publishers
imgSub = rossubscriber('/ardrone/front/image_raw');
receive(imgSub,10); % Wait to receive first message
odomSub = rossubscriber('/ground_truth/state');
receive(odomSub,10);
[velPub,velMsg] = rospublisher('/cmd_vel');
% Create video player for visualization
vidPlayer = vision.DeployableVideoPlayer;
% Load control parameters
odompose = odomSub.LatestMessage;

odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
    odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
odomRotation = quat2eul(odomQuat);
pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y, odompose.Pose.Pose.Position.Z odomRotation(1)];

robotCurrentPose = pose';
sampleTime = 0.001;
vizRate = rateControl(2/sampleTime);

k = 1;
realPoseY = 0; realPoseX = 0;
realPoseY(k) = 0; realPoseX(k) = 0; realPoseZ(k) = 0;
realW1 = 0; realW2 = 0;
realW1(k) = 0; realW2(k) = 0;
theta = [0;0;pi/2;pi;-pi/2];
figure
i =1;

while ( i < 103 )
    eold = 0;
    E = 0;
    E1 = 0;
    eold1 = 0;   
    w1 = 0;
    w2 = 0;
    eold2 = 0;
    eold1a = 0;
    eold2a = 0;
    thetapath = myTheta(robotCurrentPose,path,i);
        
        while( norm(robotCurrentPose(1:3)' - path(i+1,:)) > 0.1 )
        
            img = readImage(imgSub.LatestMessage);
            odompose = odomSub.LatestMessage;

            odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
                        odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
            odomRotation = quat2eul(odomQuat);
            pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y, odompose.Pose.Pose.Position.Z odomRotation(1)];
            robotCurrentPose = pose';
            thetapath = myTheta(robotCurrentPose,path,i);
            robotCurrentPose(3);
            [v, w, vz, E, eold, E1, eold1,eold2] = controller(robotCurrentPose', [path(i+1,:),thetapath], E, eold, E1, eold1,eold2);
            waitfor(sampleTime);
             % Package ROS message and send to the robot
            velMsg.Linear.X = v;
            velMsg.Linear.Z = vz;
            velMsg.Angular.Z = w;
            send(velPub,velMsg);
            step(vidPlayer,img);
            % Perform forward discrete integration step
            % Grab images
  
            odompose = odomSub.LatestMessage;

            odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
                odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
            odomRotation = quat2eul(odomQuat);
            pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y, odompose.Pose.Pose.Position.Z odomRotation(1)];
            robotCurrentPose = pose';

            k = k + 1;
            realPoseX(k) = robotCurrentPose(1);
            realPoseY(k) = robotCurrentPose(2);
            realPoseZ(k) = robotCurrentPose(3);
            realW1(k) = w1;
            realW2(k) = w2;

        hold off
        plot3(path(:,1), path(:,2), path(:,3),"k--d")
        hold all
        % Plot the path of the robot as a set of transforms
        plotTrVec = [robotCurrentPose(1:3)];
        plot3(realPoseX,realPoseY,realPoseZ);
        plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
        plotTransforms(plotTrVec', plotRot, "MeshFilePath", "multirotor.stl", "View","3D", "FrameSize", 2);
        light;
        %xlim([-6 6])
        %ylim([-9 1])  
        waitfor(vizRate); 
        
        end
        
        w=0;
        e1 = 1;
        while(abs(e1) > 0.2)
            
            img = readImage(imgSub.LatestMessage);
            odompose = odomSub.LatestMessage;

            odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
                odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
            odomRotation = quat2eul(odomQuat);
            pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y, odompose.Pose.Pose.Position.Z odomRotation(1)];
            robotCurrentPose = pose';
            
            [v, w, vz, E, eold, E1, eold1,eold2] = controller(robotCurrentPose', [path(i+1,:),thetapath], E, eold, E1, eold1,eold2);
            waitfor(sampleTime);
             % Package ROS message and send to the robot
            velMsg.Linear.X = v;
            velMsg.Linear.Z = vz;
            send(velPub,velMsg);
            
            e1 = atan2(-5 -robotCurrentPose(2), 0-robotCurrentPose(1)) - robotCurrentPose(4);
            e1 = wrapToPi(e1);
            w = 100*4e-3*e1;
            
            velMsg.Angular.Z = w;
            send(velPub,velMsg);
            step(vidPlayer,img);
            
        end
        waitfor(2); 
        img = readImage(imgSub.LatestMessage);
        
        I = num2str(i);
        str = strcat("image1",I,".jpg");
    
        imwrite(img, str);
        
   i = i+1;
end
velMsg.Linear.X = 0;
velMsg.Angular.Z = 0;
send(velPub,velMsg);

 
function [v, w, vz, E, eold, E1, eold1,eold2] = controller(curPose, finalPose, E, eold, E1, eold1,eold2)
           
    if(abs(finalPose(4) -curPose(4)) < 0.2)
        e = (finalPose(1:2) - curPose(1:2));
        edot = e - eold ;
        U = 1000*2e-3*e + 1*2e-3*edot;
        eold = e; 
        v = sqrt(U(1)*U(1) + U(2)*U(2));
    else
        v = 0;
    end 
    
    e1 = atan2(finalPose(2)-curPose(2), finalPose(1)-curPose(1)) - curPose(4);
    e1 = wrapToPi(e1);
    
    edot1 = e1 - eold1 ;
    E1 = E1 + e1;
    U1 = 100*4e-3*e1 + 4e-3*edot1 ;
    eold1 = e1; 
    w = U1;
    
    e2 = finalPose(3) - curPose(3) ;
    edot2 = e2 - eold2 ;
    U2 = 100*4e-3*e2 + + 4e-3*edot2 ;
    vz = U2;
    
    if v > 0.5
            v = 0.5;
        elseif v < -0.5
            v = -0.5;
    end
    
    if vz > 0.5
            vz = 0.5;
        elseif vz < -0.5
            vz = -0.5;
    end
    
    if w > 0.75
            w = 0.75;
        elseif w < -0.75
            w = -0.75;
    end
end

function theta = myTheta(robotCurrentPose, path, i)

    theta = atan2(path(i+1,2)-robotCurrentPose(2), path(i+1,1)-robotCurrentPose(1));
end


