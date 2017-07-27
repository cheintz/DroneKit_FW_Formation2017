function [] = instantPlotter( xout,n,qd,Obi,d )

hold off
relCor=1;
absCor = 0;
%scatter(xout(end,1),xout(end,2), 6, 'k','filled')
lState = xout(end,1:3);
scatter(absCor*lState(1),absCor*lState(2), 12, 'k','filled') %Leader Position


hold all;
plot([0,d*cos(lState(3))]+absCor*lState(1),[0,d*sin(lState(3))]+absCor*lState(2),'k','linewidth',6) %Leader Heading line
viscircles(absCor*lState(1:2),d);

for(i=1:n)
    thisState=xout(end,4+3*(i-1):4+3*(i-1)+2);
    scatter(thisState(1)-lState(1)*relCor,thisState(2)-lState(2)*relCor,12);
    qdi = Obi'*qd(i,:)';
    plot([0,qdi(1)]+absCor*lState(1),[0,qdi(2)]+absCor*lState(2),'linewidth',5) %desired position
    plot( [thisState(1),thisState(1) + 2*cos(thisState(3))]-lState(1)*relCor,[thisState(2),thisState(2) + 2*sin(thisState(3))]-lState(2)*relCor); %agent heading
end; 

% xlim([0,170]);
% ylim([-20,20]);
% xlim([-10,20]);
% ylim([-10,20]);
axis('equal');




