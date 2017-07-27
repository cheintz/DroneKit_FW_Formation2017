function [ xDot ] = simplifiedDynamics(t,x,v, phiDot )
    %let x be [ii, ij, phi]
    phi = x(3);
    Obi= [ cos(phi) sin(phi);...
            -sin(phi) cos(phi)];
    vl = [v;0];
    xDot(1:2) = Obi'*vl;
    xDot(3)=phiDot;
    xDot = xDot';
end

