function Vr=CalcDynamicWindow(x,model)
%Calculate DynamicWindow from model and current state
global dt;
%Window by vehicle model
Vs=[0 model(1) -model(2) model(2)];

%Window by motion model
Vd=[x(4)-model(3)*dt x(4)+model(3)*dt x(5)-model(4)*dt x(5)+model(4)*dt];

%Final Dynamic Window calculation
Vtmp=[Vs;Vd];
Vr=[max(Vtmp(:,1)) min(Vtmp(:,2)) max(Vtmp(:,3)) min(Vtmp(:,4))];
%[vmin,vmax,ωmin,ωmax]