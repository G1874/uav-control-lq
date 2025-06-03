c_T = 8.54858e-06;  % Thrust coefficient
c_d = 8.06428e-05;  % Drag coefficient
l = 0.25;           % Distance to rotor
m = 2.0;            % Mass of the UAV
g = 9.81;           % Gravitational acceleration
w_max = 1000;       % Max speed

% Coefficient matrix: coordinate system aligned drone body
C_F = [
    c_T c_T c_T c_T;
    -sqrt(2)/2*l*c_T sqrt(2)/2*l*c_T sqrt(2)/2*l*c_T -sqrt(2)/2*l*c_T;
    -sqrt(2)/2*l*c_T sqrt(2)/2*l*c_T -sqrt(2)/2*l*c_T sqrt(2)/2*l*c_T;
    -c_d -c_d c_d c_d
];

% Inertia tensor
Jxx = 0.02166666666666667; Jxy = 0.0; Jxz = 0.0;
Jyx = 0.0; Jyy = 0.02166666666666667; Jyz = 0.0;
Jzx = 0.0; Jzy = 0.0; Jzz = 0.04000000000000001;

J = [Jxx Jxy Jxz;
     Jyx Jyy Jyz;
     Jzx Jzy Jzz];

g = 9.81;

K = readmatrix("K.txt");

%%

Tstep = 0.001;
dt = 0.0001;
steps = 1:5000;

r = [1;1;10;
     0;0;0;
     0;0;0;
     0;0;0];

x0 = [0;0;0;
      0;0;0;
      0;0;0;
      0;0;0];

u = [0;0;0;0];

% tspan = 0:dt:Tstep;
% [t,x] = ode45(@(t,x)uav_dynamics(t,x,u,C_F,J,m,g),tspan,x0);

traj = [];
ctrl = [];
for k = steps
    tspan = 0:dt:Tstep;
    [t,x] = ode45(@(t,x)uav_dynamics(t,x,u,C_F,J,m,g),tspan,x0);
    x0 = x(end,:);

    traj = [traj;[t+Tstep*(k-1),x]];
    ctrl = [ctrl;[Tstep*(k-1),u']];

    u = -K*(x0'-r);
    u(u<0) = 0;
    u(u>w_max^2) = w_max^2;
end

%%

tsim = traj(:,1);
pos = traj(:,2:4);
ang = traj(:,8:10);

mid = floor(length(tsim)/2);

figure(1)
subplot(3,2,[1,3,5])
plot3(pos(:,1),pos(:,2),pos(:,3),"LineWidth",1); hold on
q = quiver3(pos(mid,1),pos(mid,2),pos(mid,3),...
    (pos(mid+100,1)-pos(mid,1))*50,(pos(mid+100,2)-pos(mid,2))*50,(pos(mid+100,3)-pos(mid,3))*50,...
    "AutoScale","off","MaxHeadSize",inf,"Alignment","head","LineWidth",2);
pause(0.1); % this appears to help
q.NodeChildren(2).Visible = 'off';
hold off
xlabel("x [m]"); ylabel("y [m]"); zlabel("z [m]")
grid on
subplot(3,2,2)
plot(tsim,rad2deg(ang(:,1)))
title("\theta_x")
xlabel("Time [s]"); ylabel("\theta_x [deg]")
grid on
subplot(3,2,4)
plot(tsim,rad2deg(ang(:,2)))
title("\theta_y")
xlabel("Time [s]"); ylabel("\theta_y [deg]")
grid on
subplot(3,2,6)
plot(tsim,rad2deg(ang(:,3)))
title("\theta_z")
xlabel("Time [s]"); ylabel("\theta_z [deg]")
grid on

%%

figure(2)
subplot(4,1,1)
stairs(ctrl(:,1),ctrl(:,2))
subplot(4,1,2)
stairs(ctrl(:,1),ctrl(:,3))
subplot(4,1,3)
stairs(ctrl(:,1),ctrl(:,4))
subplot(4,1,4)
stairs(ctrl(:,1),ctrl(:,5))