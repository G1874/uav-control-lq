function x_dot = uav_dynamics(t,x,u,C_F,J,m,g)

    pos = x(1:3);
    vel = x(4:6);
    ang = x(7:9);
    ang_vel = x(10:12);

    % Rotation matrices
    Rx = [...
        1 0 0;
        0 cos(ang(1)) -sin(ang(1));
        0 sin(ang(1)) cos(ang(1))
    ];
    
    Ry = [...
        cos(ang(2)) 0 sin(ang(2));
        0 1 0;
        -sin(ang(2)) 0 cos(ang(2))
    ];
    
    Rz = [...
        cos(ang(3)) -sin(ang(3)) 0;
        sin(ang(3)) cos(ang(3)) 0;
        0 0 1
    ];

    T = C_F * u;
    T_sigma = T(1);
    tau = T(2:4);

    % Rotation matrix local to global
    R = (Rz * Ry * Rx)';

    pos_dot = vel;
    vel_dot = [0.0;0.0;-g] + 1/m * R*[0.0;0.0;T_sigma];
    ang_dot = ang_vel;
    ang_vel_dot = J \ (tau - cross(ang_vel,J*ang_vel));

    x_dot = [pos_dot;vel_dot;ang_dot;ang_vel_dot];
end

