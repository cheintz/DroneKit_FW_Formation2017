figure;

scatter(xout(:,1),xout(:,2), 2, 'k')
xlim
hold all;
for(i=1:n)
    scatter(xout(:,3+2*i-1),xout(:,4+2*i-1),2);
end;

axis('equal')