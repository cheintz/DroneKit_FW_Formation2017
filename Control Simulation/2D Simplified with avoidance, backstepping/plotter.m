figure;

scatter(xout(:,1),xout(:,2), 2, 'k')
xlim
hold all;
for(i=1:n)
    scatter(xout(:,4+3*(i-1)),xout(:,4+3*(i-1)+1),2);
end;

axis('equal')