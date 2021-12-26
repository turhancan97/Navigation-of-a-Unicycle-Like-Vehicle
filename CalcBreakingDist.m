function stopDist=CalcBreakingDist(vel,model)
%A function that calculates the braking distance from the current speed according to the dynamic model
global dt;
stopDist=0;
while vel>0
    stopDist=stopDist+vel*dt;%Braking distance calculation
    vel=vel-model(3)*dt;%Highest principle
end