clc;clear;close all;

DIDynamics = false;

gamma = [0,-1;1,0];

dt = 1/20;
n = 1;
vl = 17;
qd = [-2,10];% -5,15];  % Agent 1 to leader; Agent 2 to leader
kl = .05;
ka = 0*0.1;
ktheta = 1;
kbackstep = 0;

alpha1 = 0.001;
alpha2 = 100;
d = 0.0001;

headingRateLimitAbs= 0.5; %rad/sec
sigmaPhi =.00001;
vMin = 15; %m/s
vMax=26; %m/s




x0l =  [0,0,0]'; %initial conditions
%x0 = [400*rand(1,2*n)-200]';
x0 = [3,5,.005]';%,  20,10,0]';
thetaDLast = zeros(n,1); %history for numerical differentiation

time = [0:dt:30]';
dtSub=1/100;

phiDotL=0.045*ones(size(time)); %degrees/second
%phiDotL=0.3*cos(0.25*time*2*pi); %degrees/second

tout = [time(1)];
xout = [x0l;x0]';%define x to be leader states (3), followed by pairs of agent states (2)3

for (k=1:length(time)-1)
   startTime=time(k); 
   tSub = [startTime:dtSub:startTime+dt]'; %substep time interval
   %Compute control
   xs = [];
   phi = xout(end,3);
   Obi= [ cos(phi) sin(phi);...
            -sin(phi) cos(phi)];
   phiDot = phiDotL(k)+sigmaPhi*randn(1); %rad/sec. This is important
   v= [vl;0];
   pl = Obi'*v;
   ql = xout(end,1:2)';

   for(i=1:n) %loop over all agents
       qi = xout(end,4+3*(i-1):4+3*(i-1) + 1)';
       theta = wrapToPi(xout(end,4+3*(i-1) + 2));
       
      % ui = pl - kl * (qi-ql + Obi'*qd(i,:)');
       ui = pl - kl * (qi-ql - Obi'*qd(i,:)') + phiDot*[0,-1;1,0]*(qi-ql); % Changed sign of phiDot compared to derivation. Should the phiDot term use qd,il, rather than qi? 
       %ui = pl - kl * (qi-ql + Obi'*qd(i,:)') + phiDot*[0,-1;1,0]*((Obi'*(-qd(i,:))')); %Should the phiDot term use qd,il, rather than qi? 
       for(j=1:n) %loop over other agents
           if(j~=i)
               qj = xout(end,4+3*(j-1):4+3*(j-1) + 1)';
               qij = qi-qj;
               ui = ui - ka * (qij + Obi' * -(qd(i,:) - qd(j,:))');
               % pij = %Doensn't make sense, as this simplified control assumes velicity can change instantly
               ata = norm(qij,2);
               if(ata<d)
                    ata
                    frepel = alpha2/(alpha1+1)-alpha2/(alpha1+ata^2/d^2);
                    ui = ui - frepel * qij ;
               end;
           end;
       end;
       
       qil = qi-ql;
       %pil = %Doesn't make sense, as velocity can change instantly
       ata = norm(qil,2);
       if(ata<d)
         ata
         frepel = alpha2/(alpha1+1)-alpha2/(alpha1+ata^2/d^2)
         ui = ui - frepel * qil ;
       end;
     
       
       vDesired = norm(ui,2);
       vDesiredTrue = vDesired;
       
       vDesired=max(vMin,min(vMax,vDesired)); %saturate desired velocity to bound of aircraft
       
       
       thetaD = atan2(ui(2),ui(1)); %note that this heading is in [-pi to pi]
       
       thetaDDotApprox = wrapToPi(thetaD-thetaDLast(i)) / dt ;
       etheta = wrapToPi(theta-thetaD);
       
       if(abs(thetaDDotApprox)>10) %mostly for startup. Should probably saturate this a little better
           thetaDDotApprox=0;
       end;
       
%      
       thetaDLast(i)=thetaD;
       
       u1i=vDesired;
       eq = Obi*qil - qd(i,:)'; %This is in leader-body
       
       u2i=(-ktheta*etheta-kbackstep*u1i*eq'*gamma*[cos(theta);sin(theta)]+thetaDDotApprox);
       
       effectiveHeadingRateLimit=headingRateLimitAbs; %provisions for more realistic  velocity dependant ratelimit (since Pixhawk limits the roll angle to a configurable angle)
       
       u2i = max(-effectiveHeadingRateLimit,min(effectiveHeadingRateLimit,u2i));%apply rate limit
       
       
       
       
       followerFunction = @(t,x) simplifiedDynamics(t,x,u1i,u2i);
       [tf,xf] = ode45(followerFunction,tSub,xout(end,4+3*(i-1):4+3*(i-1) + 2)');
       %followerFunction = @(t,x) singleIntegratorDynamics(t,x,ui);
       %[tf,xf] = ode45(followerFunction,tSub,xout(end,4+3*(i-1):4+3*(i-1) + 1)');
       
       xs(:,4+3*(i-1):4+3*(i-1)+2) = xf;
       % xs(:,4+3*(i-1):4+3*(i-1)+2) = [xf zeros(size(xf,1),1)];
   end;
   tout = [tout ; tf(2:end)];
   
   %leader dynamics
   leaderFunction = @(t,x) simplifiedDynamics(t,x,vl,phiDotL(k));
   [tl,xl] = ode45(leaderFunction,tSub,xout(end,1:3));
   xs(:,1:3) = xl;
   
   xout = vertcat(xout, xs(2:end,:));
   instantPlotter( xout,n,qd,Obi,d );
   pause(dt/10);
   
end;

plotter;




