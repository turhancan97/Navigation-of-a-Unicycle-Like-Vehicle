function [evalDB,trajDB]=Evaluation(x,Vr,goal,ob,R,model,evalParam)
%A function that calculates the evaluation value for each path
evalDB=[];
trajDB=[];

for vt=Vr(1):model(5):Vr(2)
    for ot=Vr(3):model(6):Vr(4)
        %Trajectory estimation
        [xt,traj]=GenerateTrajectory(x,vt,ot,evalParam(4),model);
        %Calculation of each evaluation function
        heading=CalcHeadingEval(xt,goal);
        dist=CalcDistEval(xt,ob,R);
        vel=abs(vt);
        
        evalDB=[evalDB;[vt ot heading dist vel]];
        trajDB=[trajDB;traj];     
    end
end