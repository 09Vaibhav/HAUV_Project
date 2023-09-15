function [cost,Dcost] = Costfunc(Z)
global Nn x_node obs Enhancements 
xe = Z(3*Nn+1:4*Nn) ;
ze = Z(4*Nn+1:5*Nn) ;
the = Z(5*Nn+1:6*Nn) ;
w2 = Z(7*Nn+1:8*Nn) ;
Tf = Z(end);

tvec = Tf/2*(x_node);

Q_1 = 10;
Q_2 = 10;
Q_3 = 1;
R = 0.1;
rho_e = xe.^2*Q_1+ze.^2*Q_2+the.^2*Q_3;
rho_u = w2.^2*R;
n = length(obs)/2;
if (Enhancements == 1)
    rho_oa = ones(Nn,1);
    for i =1:n
        rho_oa = rho_oa.*10.^(-((xe-obs(2*(i-1)+1)).^2 + (ye-obs(2*i)).^2));
    end

    cost = trapz(tvec,rho_e+rho_oa+rho_u);
else
    cost = trapz(tvec,rho_e+rho_u);
end
Dcost=0;
end