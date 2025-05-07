using LinearAlgebra
using DifferentialEquations
using ControlSystems
using ForwardDiff


c_T = 0.5   # Thrust coefficient
c_d = 0.5   # Drag coefficient
l = 0.25    # Distance to rotor
m = 2.0     # Mass of the UAV
g = 9.81    # Gravitational acceleration

"Coefficient matrix"
C_F = [
    c_T c_T c_T c_T;
    0 l*c_T 0 -l*c_T;
    -l*c_T 0 l*c_T 0;
    -c_d c_d -c_d c_d
]

"Rotation matrices"
Rx(θ) = [
    1 0 0;
    0 cos(θ) -sin(θ);
    0 sin(θ) cos(θ)
]

Ry(θ) = [
    cos(θ) 0 sin(θ);
    0 1 0;
    -sin(θ) 0 cos(θ)
]

Rz(θ) = [
    cos(θ) -sin(θ) 0;
    sin(θ) cos(θ) 0;
    0 0 1
]

"Inertia tensor"
Jxx = 0.02166666666666667; Jxy = 0.0; Jxz = 0.0;
Jyx = 0.0; Jyy = 0.02166666666666667; Jyz = 0.0;
Jzx = 0.0; Jzy = 0.0; Jzz = 0.04000000000000001;

J = [Jxx Jxy Jxz;
     Jyx Jyy Jyz;
     Jzx Jzy Jzz]

"Dynamical system equations"
function f(x, u)
    ξ = x[1:3]
    v = x[4:6]
    Θ = x[7:9]
    Ω = x[10:12]

    T = C_F * u
    T_Σ = T[1]
    τ = T[2:4]

    Θ_x, Θ_y, Θ_z = Θ

    "Rotation matrix local to global"
    R = (Rx(Θ_x) * Ry(Θ_y) * Rz(Θ_z))'

    ξ_dot = v
    v_dot = [0.0,0.0,-g] + 1/m * R*[0.0,0.0,T_Σ]
    Θ_dot = Ω
    Ω_dot = J \ (τ - cross(Ω,J*Ω))

    return [ξ_dot;v_dot;Θ_dot;Ω_dot]
end

"Equilibrium point for UAV hanging in the air"
ξ_0 = [0.0, 0.0, 0.0]
v_0 = [0.0, 0.0, 0.0]
Θ_0 = [0.0, 0.0, 0.0]
Ω_0 = [0.0, 0.0, 0.0]

T_Σ0 = m*g; τ_x0 = 0.0; τ_y0 = 0.0; τ_z0 = 0.0
T0 = [T_Σ0, τ_x0, τ_y0, τ_z0]

x0 = [ξ_0; v_0; Θ_0; Ω_0]
u0 = C_F \ T0

A = ForwardDiff.jacobian(x -> f(x, u0), x0)
B = ForwardDiff.jacobian(u -> f(x0, u), u0)
C = I
D = 0.0

linear_sys = ss(A, B, C, D)

print(linear_sys)