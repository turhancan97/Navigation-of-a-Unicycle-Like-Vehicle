function heading=CalcHeadingEval(x,goal)
%A function that calculates the evaluation function of heading

theta=toDegree(x(3));%Robot orientation
goalTheta=toDegree(atan2(goal(2)-x(2),goal(1)-x(1)));%Goal direction

if goalTheta>theta
    targetTheta=goalTheta-theta;%Direction difference to the goal [deg]
else
    targetTheta=theta-goalTheta;%Direction difference to the goal [deg]
end

heading=180-targetTheta;