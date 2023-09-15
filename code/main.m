clc;clear;close all
global Nn Dmat x_node traj_model save_state p obs Enhancements 
% Tf = 10;
Enhancements =0;
if Enhancements ==1
obs = [0,2];%                      0 2
end

traj_model = 4;  %type of traj , change l_u, l(end) depending on type of trajectry
save_state = 1;

if (traj_model == 1) % Circle
    r=3; % radius                       1
    p=[r]; 
elseif (traj_model ==2) % Spiral 
    a=1;   % Constant radius part       1
    b=0.1; % Coeff of variable radius   0.1
    c=1;   % No of spirals              1
    p=[a,b,c]; 
elseif (traj_model ==3) % Elliptical
    a=2;                % 2
    b=1;                % 1
    p=[a,b];
elseif (traj_model ==4)
    a=5;
    b = 1;
    b = tan(45*pi/180);
    p=[a,b];
elseif (traj_model ==5)
    a=2;
    c = -3.7;
    % b = 1;
    % b = tan(45*pi/180);
    p=[a,c];
end

Nn = 41;

[x_node,Dmat] = legDc(Nn-1);
x_node = 1-x_node;
if traj_model == 1
    load('Circular_traj.mat')
elseif traj_model == 2
    load('Circular_traj.mat')
elseif traj_model == 3
    load('Circular_traj.mat')
end
% initial Guess array
x_i  = ones(Nn,1)*0.2; 
y_i  = -ones(Nn,1)*0;
X_i  = ones(Nn,1)*(pi/6);  %pi/2 circle
x_ei = 0.1732*ones(Nn,1);
y_ei = ones(Nn,1)*0.1;
X_ei = zeros(Nn,1);
% l_i  = -1*ones(Nn,1);
l_i  = 0*ones(Nn,1);
u_i  = 0*ones(Nn,1);
Tf_i = 20;

% tvec_i = guess(:,1);
% x_i = guess(:,2);
% y_i = guess(:,3);
% X_i = guess(:,4);
% x_ei = guess(:,5);
% y_ei = guess(:,6);
% X_ei = guess(:,7);
% l_i = guess(:,8);
% u_i = guess(:,9);
% Tf_i = tvec_i(end);
Z_0 = [x_i;y_i;X_i;x_ei;y_ei;X_ei;l_i;u_i;Tf_i];
figure(1)
plot(x_i,y_i)

%lb
x_l  =  -500*ones(Nn,1);
y_l  =  -500*ones(Nn,1);
X_l  =  ones(Nn,1)*(-2*pi);
x_el = -50*ones(Nn,1); 
y_el = -50*ones(Nn,1);
X_el = -75*pi/180*ones(Nn,1);
% l_l  = -3*pi*ones(Nn,1);
l_l  = 0*ones(Nn,1);
u_l  = -3*ones(Nn,1);
Tf_l = 0.1;

lb = [x_l;y_l;X_l;x_el;y_el;X_el;l_l;u_l;Tf_l];

%ub
x_u  =  500*ones(Nn,1);
y_u  =  500*ones(Nn,1);
X_u  =  2*pi*ones(Nn,1);
x_eu =  50*ones(Nn,1); 
y_eu =  50*ones(Nn,1);
X_eu =  75*pi/180*ones(Nn,1);
l_u  =  ones(Nn,1)*0; % change with traj
l_u  =  ones(Nn,1)*10/sqrt(3);
u_u  =  3*ones(Nn,1);
Tf_u = 100;

ub = [x_u;y_u;X_u;x_eu;y_eu;X_eu;l_u;u_u;Tf_u];

A = []; % No other constraints
b = [];
Aeq = [];
beq = [];

if (traj_model == 4)
    ct = 1e-10;
else
    ct = 1e-08;
end

 options =  optimoptions ('fmincon','Display','Iter','OptimalityTolerance',1e-08,'StepTolerance',...
     1e-08, 'ConstraintTolerance' ,ct, 'MaxIterations', 5000,'MaxFunctionEvaluations',1200000,'Algorithm', 'sqp');
[Z, costval, exitflag, output] = fmincon(@(Z)Costfunc(Z), Z_0, A, b,...
    Aeq, beq, lb, ub, @(Z)C_fun(Z),options);

x = Z(1:Nn) ;
y = Z(Nn +1:2*Nn) ;
X = Z(2*Nn +1:3*Nn) ;
xe = Z(3*Nn+1:4*Nn) ;
ye = Z(4*Nn+1:5*Nn) ;
Xe = Z(5*Nn +1:6* Nn) ;
l = Z(6*Nn+1:7*Nn) ;
w2 = Z(7*Nn+1:8*Nn) ;
Tf = Z(end);

tvec = Tf/2*(x_node);
%%
figure(2)
plot(x,y,"*")
hold on
plot(x-(xe.*cos(X-Xe)+ye.*sin(X-Xe)),y-(ye.*cos(X-Xe)-xe.*sin(X-Xe)),"o")
hold on
x1 = linspace(0,5,41);
y1 = tan(-pi/6)*x1;
plot(x1,y1)
xlabel('x')
ylabel('z')
legend('actual','desired','ref')


%%
figure(4)
plot(tvec,X*180/pi)
hold on
plot(tvec,(X-Xe)*180/pi)
xlabel('time')
ylabel('pitch angle')
legend('actual pitch angle','desired pitch angle')

%%
figure(5)
plot(tvec,w2)
xlabel("Time")
ylabel("pitch rate")
%%
figure(6)
plot(x-(xe.*cos(X-Xe)+ye.*sin(X-Xe)),y-(ye.*cos(X-Xe)-xe.*sin(X-Xe)))
hold on
plot(x,y)
% hold on
% plot(3*cos(l/3),3*sin(l/3))
hold on
xlabel("X")
ylabel("Z")
% legend("Desire Trajectory","Actual trajectory","ref")


legend("Desire Trajectory","ref ")

%%
% guess= [tvec;x;y;X;xe;ye;Xe;l;mu];
% guess = reshape(guess,Nn,9);
% if save_state ==1
%     save("Elliptical_traj.mat",'guess')
% end

% kappa = 1; % circle 
% v = 0.3;
% vq = (v./(1+sqrt(xe.^2+ye.^2)));
% xe_d = ye.*kappa.*vq + v.*cos(Xe) - vq;
% ye_d = -xe.*kappa.*vq + v.*sin(Xe);
% Xe_d = w2 - kappa.*vq;
% ld = xe.*xe_d + ye.*ye_d + Xe.*Xe_d;
% 

% figure(6)
% plot3(x-(xe.*cos(X-Xe)+ye.*sin(X-Xe)),zeros(41,1),y-(ye.*cos(X-Xe)-xe.*sin(X-Xe)))
% hold on
% plot3(x,zeros(41,1),y)
% % hold on
% % t1 = linspace(-pi,0,20);
% % xd = cos(t1);
% % yd = sin(t1);
% % plot(xd,yd)
% xlabel("X")
% ylabel("Y")
% zlabel("Z")
% % legend("Desire Trajectory","Actual trajectory","ref")
% legend("Desire Trajectory","Actual trajectory")
% 
% figure(7)
% plot(cos(l),sin(l))