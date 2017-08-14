function [] = instantPlotter( xout,n,qd,Obi,d )

hold off
%scatter(xout(end,1),xout(end,2), 6, 'k','filled')
scatter(0,0, 12, 'k','filled') %Leader Position
hold all;
plot([0,d*cosd(xout(end,3))],[0,d*sind(xout(end,3))],'k','linewidth',6)
viscircles([0,0],d);
%xlim([-300,10]);
%ylim([-200,175]);
xlim([-10,20]);
ylim([-10,20]);
axis('equal');

for(i=1:n)
    scatter(xout(end,3+2*i-1)-xout(end,1),xout(end,4+2*i-1)-xout(end,2),12);
    qdi = -Obi'*qd(i,:)';
    plot([0,qdi(1)],[0,qdi(2)])
end;

end

