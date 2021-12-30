% -------------------------------------------------------------------------
% Description : Navigation of Unicycle-Like Vehicle with Dynamic Window Approach
%
% Author : Turhan Can Kargın, Adil Mortepe
%
% -------------------------------------------------------------------------
close all;
clear all;
clc

disp('Navigation of a Unicycle-Like Vehicle has Started!!')

x=[0 0 pi/2 0 0]';% Robot initial state [x(m),y(m),yaw(Rad),v(m/s),ω(rad/s)]
goal=[1,1];% Goal position [x(m),y(m)]
% Obstacle list [x(m) y(m)]
obstacle = load('obstacle.txt'); % You can load the obstacle from text file
      
obstacleR=0.5;% Radius of obstacle for collision detection
global dt; dt=0.1; % Stepping time [s]

% %%%%Robotic Kinematic Variable%%%%%%%
% Maximum Linear Velocity [m / s]
% Maximum Angular Velocity [rad / s], 
% Maximum acceleration / deceleration [m / s^2], 
% Maximum Angular acceleration / deceleration [rad / s^2],
% Velocity resolution [m / s]
% Angular velocity resolution [rad / s]]

Kinematic=[1.0,toRadian(20.0),0.2,toRadian(50.0),0.01,toRadian(1)];

% Evaluation function parameters [heading,dist,velocity,predictDT]
evalParam=[0.1,0.2,0.1,3.0]; % alpha, beta, evaldt
area=[-1 11 -1 11];% The size of the simulation area [xmin xmax ymin ymax]

% simulation result
result.x=[]; % Accumulate and store the state value of the trajectory points traversed
tic; % Start of estimating program running time
%movcount=0;
% Main loop
for i=1:5000 % Cycle operation 5000 times, guide to reach the destination or end of 5000 times operation
    % Calculation of input value by DWA
    [u,traj]=DynamicWindowApproach(x,Kinematic,goal,evalParam,obstacle,obstacleR);
    x=f(x,u);%Movement by motion model
    
    %Saving simulation results
    result.x=[result.x; x']; % The latest result is added to result.x as a column
    
    %Goal judgment
    if norm(x(1:2)-goal')<0.5
        disp('The Robot Reached the Goal');break;
    end
    
    %====Animation====
    hold off;
    ArrowLength=0.5;%Arrow length
    %robot
    quiver(x(1),x(2),ArrowLength*cos(x(3)),ArrowLength*sin(x(3)),'sk');hold on;
    plot(result.x(:,1),result.x(:,2),'-b');hold on;
    plot(goal(1),goal(2),'or');hold on;
    plot(obstacle(:,1),obstacle(:,2),'*k');hold on;
    % Search trajectory display
    if ~isempty(traj)
        for it=1:length(traj(:,1))/5
            ind=1+(it-1)*5;
            plot(traj(ind,:),traj(ind+1,:),'-g');hold on;
        end
    end
    axis(area);
    grid on;
    drawnow;
    %movcount=movcount+1;
    %mov(movcount) = getframe(gcf);% Get an animation frame
end
figure(2)
plot(result.x(:,1),result.x(:,2));
xlabel("Robot Position in x axis [m]")
ylabel("Robot Position in y axis [m]")
title("Trajectory of the Robot")
figure(3)
plot(result.x(:,4));
xlabel("Time [s]")
ylabel("Velocity [m/s]")
title("Velocity of the Robot")
figure(4)
plot(result.x(:,5));
xlabel("Time [s]")
ylabel("Angular Velocity [rad/s]")
title("Angular Velocity of the Robot")
figure(5)
plot(result.x(:,3));
xlabel("Time [s]")
ylabel("Orientation Angle [Rad]")
title("Orientation Angle of the Robot")
toc
% movie2avi(mov,'movie.avi');