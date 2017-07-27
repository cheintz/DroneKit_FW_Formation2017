function [ xDot ] = Dynamics(t,x,v, phiDot )
    %let x be [ii, ij, phi]
    phi = x(3);
    Obi= [ cosd(phi) sind(phi);...
            -sind(phi) cosd(phi)];
    vl = [v;0];
    xDot(1:2) = Obi'*vl;
    xDot(3)=phiDot;
    xDot = xDot';
end

