function [c,ceq,dc,dceq] = C_fun(Z)
global Nn Dmat traj_model

x = Z(1:Nn) ;
z = Z(Nn +1:2*Nn) ;
th = Z(2*Nn +1:3*Nn) ;
xe = Z(3*Nn+1:4*Nn) ;
ze = Z(4*Nn+1:5*Nn) ;
the = Z(5*Nn +1:6* Nn) ;
l = Z(6*Nn+1:7*Nn) ;
w2 = Z(7*Nn+1:8*Nn) ;
Tf = Z(end);

%%
for i =1:Nn
[x_dot(i),z_dot(i),th_dot(i),xe_dot(i),ze_dot(i),the_dot(i),l_dot(i)] = ...
    EquationsRhs([x(i),z(i),th(i),xe(i),ze(i),the(i),l(i)],w2(i),Tf);
end

%%
ceq(1:Nn) = Dmat*x - x_dot';
ceq(Nn+1:2*Nn) = Dmat*z -z_dot';
ceq(2*Nn+1:3*Nn) = Dmat*th - th_dot';
ceq(3*Nn+1:4*Nn) = Dmat*xe - xe_dot';
ceq(4*Nn+1:5*Nn) = Dmat*ze - ze_dot';
ceq(5*Nn+1:6*Nn) = Dmat*the - the_dot';
ceq(6*Nn+1:7*Nn) = Dmat*l - l_dot';
ceq(7*Nn+1) = x(1)-0.2;
ceq(7*Nn+2) = z(1)+0;
if traj_model == 4
    % ceq(7*Nn+3) = X(1)-pi/3;
    ceq(7*Nn+3) = th(1)-pi/6;
else
    ceq(7*Nn+3) = th(1)-pi/2;
end
% ceq(7*Nn+4) = l(1)+3*pi;
% ceq(7*Nn+5) = l(end); % change with traj
ceq(7*Nn+4) = l(1);
ceq(7*Nn+5) = l(end)-5.733; % change with traj
ceq(7*Nn+6) = xe(1)-0.1732;
ceq(7*Nn+7) = ze(1)-0.1;
ceq(7*Nn+8) = the(1);
% ceq(7*Nn+9) = x(21)+0;
% ceq(7*Nn+10) = y(21)+1;
% ceq(7*Nn+11) = x(end)-2;
% ceq(7*Nn+12) = y(end);

c = 0;
dc = 0;
dceq = 0;
end