clc;clear;close all;

dt = 1/20;
n = 2;
vl = 13;
qd = [-5,5 ; -5,15];  % Agent 1 to leader; Agent 2 to leader
kl = 1;
ka = 0.5;

alpha1 = 0.001;
alpha2 = 10;
d = 6;


x0l =  [0,0,90]' %initial conditions
%x0 = [400*rand(1,2*n)-200]';
x0 = [-62,112,70,-200]';

time = [0:dt:54]';
dtSub=1/100;

phiDotL=5*ones(size(time)); %degrees/second
%phiDotL=50*sin(time); %degrees/second

tout = [time(1)];
xout = [x0l;x0]';%define x to be leader states (3), followed by pairs of agent states (2)3

for (k=1:length(time)-1)
   startTime=time(k); 
   tSub = [startTime:dtSub:startTime+dt]'; %substep time interval
   %Compute control
   xs = [];
   phi = xout(end,3);
   Obi= [ cosd(phi) sind(phi);...
            -sind(phi) cosd(phi)];
   phiDot = phiDotL(k)/180*pi; %rad/sec. This is important
   v= [vl;0];
   pl = Obi'*v;
   ql = xout(end,1:2)';

   for(i=1:n) %loop over all agents
       qi = xout(end,4+2*(i-1):4+2*(i-1) + 1)';
      % ui = pl - kl * (qi-ql + Obi'*qd(i,:)');
       ui = pl - kl * (qi-ql + Obi'*qd(i,:)') + phiDot*[0,-1;1,0]*(qi-ql); % Changed sign of phiDot compared to derivation. Should the phiDot term use qd,il, rather than qi? 
     %  ui = pl - kl * (qi-ql + Obi'*qd(i,:)') + phiDot*[0,-1;1,0]*((Obi'*(-qd(i,:))')); %Should the phiDot term use qd,il, rather than qi? 
       for(j=1:n) %loop over other agents
           if(j~=i)
               qj = xout(end,4+2*(j-1):4+2*(j-1) + 1)';
               qij = qi-qj;
               ui = ui - ka * (qij + Obi' * (qd(i,:) - qd(j,:))');
               % pij = %Doensn't make sense, as this simplified control assumes velicity can change instantly
               ata = norm(qij,2);
               if(ata<d)
                  frepel = alpha2/(alpha1+1)-alpha2/(alpha1+ata^2/d^2);
                  ui = ui - frepel * qij / norm(qij,2);
               end;
           end;
       end;
       
       qil = qi-ql;
       %pil = %Doesn't make sense, as velocity can change instantly
       ata = norm(qil,2);
       if(ata<d)
          ata
          frepel = alpha2/(alpha1+1)-alpha2/(alpha1+ata^2/d^2);
          ui = ui - frepel * qil / norm(qil,2);
       end;

       followerFunction = @(t,x) singleIntegratorDynamics(t,x,ui);
       [tf,xf] = ode45(followerFunction,tSub,qi );
       xs(:,4+2*(i-1):4+2*(i-1)+1) = xf;
   end;
   tout = [tout ; tf(2:end)];
   
   %leader dynamics
   leaderFunction = @(t,x) leaderDynamics(t,x,vl,phiDotL(k));
   [tl,xl] = ode45(leaderFunction,tSub,xout(end,1:3));
   xs(:,1:3) = xl;
   
   xout = vertcat(xout, xs(2:end,:));
   instantPlotter( xout,n,qd,Obi,d );
   pause(dt/1);
   
end;

plotter;




