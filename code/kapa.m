function [k,dD_dl] = kapa(l,p)

global traj_model

if traj_model ==1 % Circle
    r = p(1);
    k = 2*pi;
    dD_dl = 2*pi*r;
    k = 1/r;
    dxd = -sin(l/r);
    dzd = cos(l/r);
elseif traj_model == 2 % Spiral 
    a=p(1); % Constant radius part
    b=p(2); % Coeff of variable radius
    c=p(3); % No of spirals
    k = 2*pi*c*(4*pi^2*b^2*c^2*l^2+4*pi*a*b*c*l+2*b^2+a^2)/...
        (4*pi^2*b^2*c^2*l^2+4*pi*a*b*c*l+b^2+a^2);
    dD_dl = 2*pi*c*sqrt(((2*pi*b*c*l+a)*sin(2*pi*c*l)-b*cos(2*pi*c*l))^2+((2*pi*b*c*l+a)*cos(2*pi*c*l)+b*sin(2*pi*c*l))^2);
elseif traj_model == 3 % Elliptical
    a=p(1);
    b=p(2);
    k = (2*pi*a*b*(csc(2*pi*l))^2)/(b^2*(cot(2*pi*l))^2+a^2);
    dD_dl = 2*pi*sqrt(a^2*(sin(2*pi*l))^2+b^2*(cos(2*pi*l))^2);
elseif traj_model == 4 % Straight line
    a=p(1);
    b=p(2);
    k = 0;
    
    dD_dl = a*sqrt(1+b^2);
elseif traj_model == 5 % Straight line
    % a=p(1);
    % xd = -sqrt(2)*sin(l/2);
    % yd = -sqrt(cos(l));
    % dxd = (-sqrt(2)/2)*(cos(l/2));
    % ddxd = (sqrt(2)/4)*sin(l/2);
    % dyd = sin(l)/(2*sqrt(cos(l)));
    % ddyd = (1/2)*((cos(l))^2 + 1)/(2*cos(l)*sqrt(cos(l)));
    % k = (dxd*ddyd - ddxd*dyd)/(dxd^2 + dyd^2)^(3/2);
    % % k = 0;
    % 
    % % dD_dl = a*sqrt(1+b^2);
    % dD_dl = 0;
    r = p(1);
    c = p(2);
    dx = r*cos(l+c);
    ddx = -r*sin(l+c);
    dy = -r*sin(l+c);
    ddy = -r*cos(l+c);
    k = (dx*ddy - ddx*dy)/(dx^2 + dy^2)^(3/2);
    dD_dl = 0;
end 

end