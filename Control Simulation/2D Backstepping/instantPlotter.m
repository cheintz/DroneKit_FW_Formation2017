function [] = instantPlotter( xout,n )

hold off
scatter(xout(end,1),xout(end,2), 6, 'k','filled')
xlim([-300,10]);
ylim([-200,175]);
axis('equal');
hold all;
for(i=1:n)
    scatter(xout(end,3+2*i-1),xout(end,4+2*i-1),6);
end;

end

