function [x,traj]=GenerateTrajectory(x,vt,ot,evaldt,model)
% Function to create trajectory data
global dt;
time=0;
u=[vt;ot];%Input value
traj=x;%Trajectory data
while time<=evaldt
    time=time+dt;%Simulation time update
    x=f(x,u);%Transition by motion model
    traj=[traj x];
end