% Connect to ROS master
rosshutdown;
turtlebotIp = '192.168.247.133';
rosinit(turtlebotIp);
% Create ROS subscribers and publishers
imgSub = rossubscriber('/camera/rgb/image_raw');
receive(imgSub,10); % Wait to receive first message
odomSub = rossubscriber('/odom');
receive(odomSub,10);
[velPub,velMsg] = rospublisher('/cmd_vel');
% Create video player for visualization
vidPlayer = vision.DeployableVideoPlayer;
% Load control parameters
params = controlParams;
odompose = odomSub.LatestMessage;

odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
    odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
odomRotation = quat2eul(odomQuat);
pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y odomRotation(1)];

path = [0 0; 4 0; 4 4; 0 4; 0 0];
robotCurrentPose = pose';
sampleTime = 0.001;
vizRate = rateControl(2/sampleTime);

k = 1;
realPoseY = 0; realPoseX = 0;
realPoseY(k) = 0; realPoseX(k) = 0;
realW1 = 0; realW2 = 0;
realW1(k) = 0; realW2(k) = 0;
theta = [0;0;pi/2;pi;-pi/2];
figure
thetad = [3*pi/4;-3*pi/4;-pi/4;pi/4];
i =1;

while ( i < 5 )
          eold = 0;
    E = 0;
    E1 = 0;
    eold1 = 0;   
    w1 = 0;
    w2 = 0;
    eold1a = 0;
    eold2a = 0;
    thetapath = myTheta(robotCurrentPose,path,i);
        
        while( norm(robotCurrentPose(1:2)' - path(i+1,:)) > 0.1 )
        
            img = readImage(imgSub.LatestMessage);
            odompose = odomSub.LatestMessage;

            odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
                        odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
            odomRotation = quat2eul(odomQuat);
            pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y odomRotation(1)];
            robotCurrentPose = pose';
            thetapath = myTheta(robotCurrentPose,path,i);
            robotCurrentPose(3);
            [v, w, E, eold, E1, eold1] = controller(robotCurrentPose', [path(i+1,:),thetapath], E, eold, E1, eold1);
            waitfor(sampleTime);
             % Package ROS message and send to the robot
            velMsg.Linear.X = v;
            velMsg.Angular.Z = w;
            send(velPub,velMsg);
            step(vidPlayer,img);
            % Perform forward discrete integration step
            % Grab images
  
            odompose = odomSub.LatestMessage;

            odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
                odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
            odomRotation = quat2eul(odomQuat);
            pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y wrapToPi(odomRotation(1))];
            robotCurrentPose = pose';

            k = k + 1;
            realPoseX(k) = robotCurrentPose(1);
            realPoseY(k) = robotCurrentPose(2);
            realW1(k) = w1;
            realW2(k) = w2;

        hold off
        plot(path(:,1), path(:,2),"k--d")
        hold all
        % Plot the path of the robot as a set of transforms
        plotTrVec = [robotCurrentPose(1:2); 0];
        plot(realPoseX,realPoseY);
        plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
        plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "View","2D", "FrameSize", 2);
        light;
        xlim([-3 10])
        ylim([-5 10])  
        waitfor(vizRate); 
        
        end
    
              w=0;
        while(abs(thetad(i)-  robotCurrentPose(3)) > 0.1)
            
            img = readImage(imgSub.LatestMessage);
            odompose = odomSub.LatestMessage;
            odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
                odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
            odomRotation = quat2eul(odomQuat);
            pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y wrapToPi(odomRotation(1))];
            robotCurrentPose = pose';
            
            e1 = wrapToPi(thetad(i)) - robotCurrentPose(3);
            e1 = wrapToPi(e1);
            w = 1000*4e-3*e1;
            
            v = 0;
            velMsg.Linear.X = v;
            velMsg.Angular.Z = w;
            send(velPub,velMsg);
            step(vidPlayer,img);
            
        end
        waitfor(2); 
        img = readImage(imgSub.LatestMessage);
        str = ["image1.jpg";"image2.jpg";
        "image3.jpg";"image4.jpg"] ;
        imwrite(img, str(i));    
        
   i = i+1;
end
velMsg.Linear.X = 0;
velMsg.Angular.Z = 0;
send(velPub,velMsg);

 
function [v, w, E, eold, E1, eold1] = controller(curPose, finalPose, E, eold, E1, eold1)
           
    if(abs(finalPose(3) -curPose(3)) < 0.2)
        e = (finalPose(1:2) - curPose(1:2));
        edot = e - eold ;
        U = 1000*2e-3*e + 1*2e-3*edot;
        eold = e; 
        v = sqrt(U(1)*U(1) + U(2)*U(2));
    else
        v = 0;
    end 
    
    e1 = atan2(finalPose(2)-curPose(2), finalPose(1)-curPose(1)) - curPose(3);
    e1 = wrapToPi(e1);
    
    edot1 = e1 - eold1 ;
    E1 = E1 + e1;
    U1 = 100*4e-3*e1 + 4e-3*edot1 ;
    eold1 = e1; 
    w = U1;
    
    if v > 0.5
            v = 0.5;
        elseif v < -0.5
            v = -0.5;
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





