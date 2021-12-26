function dist=CalcDistEval(x,ob,R)
%A function that calculates the distance evaluation value with an obstacle

dist=2;
for io=1:length(ob(:,1))
    disttmp=norm(ob(io,:)-x(1:2)')-R;%Calculate the norm error between the position of the path and the obstacle
    if dist>disttmp%Find the minimum
        dist=disttmp;
    end
end