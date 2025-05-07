using ControlSystems, ForwardDiff

"An example of nonlinear dynamics"
function f(x, u)
    x1, x2 = x
    u1, u2 = u
    [x2; u1*x1 + u2*x2]
end

x0 = [1.0, 0.0] # Operating point to linearize around
u0 = [0.0, 1.0]

A = ForwardDiff.jacobian(x -> f(x, u0), x0)
B = ForwardDiff.jacobian(u -> f(x0, u), u0)

"An example of a nonlinear output (measurement) function"
function g(x, u)
    y = [x[1] + 0.1x[1]*u[2]; x[2]]
end

C = ForwardDiff.jacobian(x -> g(x, u0), x0)
D = ForwardDiff.jacobian(u -> g(x0, u), u0)

linear_sys = ss(A, B, C, D)

print(linear_sys)