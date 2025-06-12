c_T = 8.54858e-06;  % Thrust coefficient
c_d = 8.06428e-05;  % Drag coefficient
l = 0.25;           % Distance to rotor
m = 2.0;            % Mass of the UAV
g = 9.81;           % Gravitational acceleration
w_max = 1000;       % Max speed

% Coefficient matrix
C_F = [
    c_T c_T c_T c_T;
    -sqrt(2)/2*l*c_T sqrt(2)/2*l*c_T sqrt(2)/2*l*c_T -sqrt(2)/2*l*c_T;
    -sqrt(2)/2*l*c_T sqrt(2)/2*l*c_T -sqrt(2)/2*l*c_T sqrt(2)/2*l*c_T;
    -c_d -c_d c_d c_d
];

% C_F = [
%     -c_T -c_T -c_T -c_T;
%     -sqrt(2)/2*l*c_T sqrt(2)/2*l*c_T sqrt(2)/2*l*c_T -sqrt(2)/2*l*c_T;
%     sqrt(2)/2*l*c_T -sqrt(2)/2*l*c_T sqrt(2)/2*l*c_T -sqrt(2)/2*l*c_T;
%     c_d c_d -c_d -c_d
% ];

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
steps = 1:10000;

pos_ref = [1,1,-10];
yaw_ref = 0;

r = [1;1;10;
     0;0;0;
     0;0;0;
     0;0;0];

x0 = [0;0;0;
      0;0;0;
      0;0;0;
      0;0;0];

u = [0;0;0;0];

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
vel = traj(:,5:7);
ang = traj(:,8:10);
ang_vel = traj(:,11:13);

mid = floor(length(tsim)/2);

f1 = figure(1);
plot3(pos(:,1),pos(:,2),pos(:,3),"LineWidth",1); hold on
q = quiver3(pos(mid,1),pos(mid,2),pos(mid,3),...
    (pos(mid+100,1)-pos(mid,1))*50,(pos(mid+100,2)-pos(mid,2))*50,(pos(mid+100,3)-pos(mid,3))*50,...
    "AutoScale","off","MaxHeadSize",inf,"Alignment","head","LineWidth",2);
pause(0.1); % this appears to help
q.NodeChildren(2).Visible = 'off';
hold off
xlabel("x [m]"); ylabel("y [m]"); zlabel("z [m]")
grid on
exportgraphics(f1,'img/f1.png','Resolution',600)

f2 = figure(2);
subplot(3,1,1)
plot(tsim,ang(:,1))
title("\theta_x")
xlabel("Time [s]"); ylabel("\theta_x [rad]")
grid on
subplot(3,1,2)
plot(tsim,ang(:,2))
title("\theta_y")
xlabel("Time [s]"); ylabel("\theta_y [rad]")
grid on
subplot(3,1,3)
plot(tsim,ang(:,3))
title("\theta_z")
xlabel("Time [s]"); ylabel("\theta_z [rad]")
grid on
exportgraphics(f2,'img/f2.png','Resolution',600)

f3 = figure(3);
subplot(3,1,1)
plot(tsim,ang_vel(:,1))
title("$\dot{\theta_x}$", "Interpreter", "latex")
xlabel("Time [s]"); ylabel("$\dot{\theta_x}$ [rad/s]", "Interpreter", "latex")
grid on
subplot(3,1,2)
plot(tsim,ang_vel(:,2))
title("$\dot{\theta_y}$", "Interpreter", "latex")
xlabel("Time [s]"); ylabel("$\dot{\theta_y}$ [rad/s]", "Interpreter", "latex")
grid on
subplot(3,1,3)
plot(tsim,ang_vel(:,3))
title("$\dot{\theta_z}$", "Interpreter", "latex")
xlabel("Time [s]"); ylabel("$\dot{\theta_z}$ [rad/s]", "Interpreter", "latex")
grid on
exportgraphics(f3,'img/f3.png','Resolution',600)

f4 = figure(4);
subplot(3,1,1)
plot(tsim,pos(:,1))
title("\xi_x")
xlabel("Time [s]"); ylabel("\xi_x [m]")
grid on
subplot(3,1,2)
plot(tsim,pos(:,2))
title("\xi_y")
xlabel("Time [s]"); ylabel("\xi_y [m]")
grid on
subplot(3,1,3)
plot(tsim,pos(:,3))
title("\xi_z")
xlabel("Time [s]"); ylabel("\xi_z [m]")
grid on
exportgraphics(f4,'img/f4.png','Resolution',600)

f5 = figure(5);
subplot(3,1,1)
plot(tsim,vel(:,1))
title("$\dot{\xi_x}$", "Interpreter", "latex")
xlabel("Time [s]"); ylabel("$\dot{\xi_x}$ [m/s]", "Interpreter", "latex")
grid on
subplot(3,1,2)
plot(tsim,vel(:,2))
title("$\dot{\xi_y}$", "Interpreter", "latex")
xlabel("Time [s]"); ylabel("$\dot{\xi_y}$ [m/s]", "Interpreter", "latex")
grid on
subplot(3,1,3)
plot(tsim,vel(:,3))
title("$\dot{\xi_z}$", "Interpreter", "latex")
xlabel("Time [s]"); ylabel("$\dot{\xi_z}$ [m/s]", "Interpreter", "latex")
grid on
exportgraphics(f5,'img/f5.png','Resolution',600)

f6 = figure(6);
subplot(4,1,1)
stairs(ctrl(:,1),ctrl(:,2))
title("\omega_0^2")
xlabel("Time [s]"); ylabel("\omega_0^2 [rad^2/s^2]")
grid on
subplot(4,1,2)
stairs(ctrl(:,1),ctrl(:,3))
title("\omega_1^2")
xlabel("Time [s]"); ylabel("\omega_1^2 [rad^2/s^2]")
grid on
subplot(4,1,3)
stairs(ctrl(:,1),ctrl(:,4))
title("\omega_2^2")
xlabel("Time [s]"); ylabel("\omega_2^2 [rad^2/s^2]")
grid on
subplot(4,1,4)
stairs(ctrl(:,1),ctrl(:,5))
title("\omega_3^2")
xlabel("Time [s]"); ylabel("\omega_3^2 [rad^2/s^2]")
grid on
exportgraphics(f6,'img/f6.png','Resolution',600)
