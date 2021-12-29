import numpy as np
import mpctools as mpc

'''
x[0]: xt
x[1]: yt
x[2]: theta_r
x[3]: gamma
x[4]: theta_r - theta_t
x[5]: phi (angulo de steer)
u[0]: vf (velocidad del eje delantero en la direccion de i_f)
u[1]: om_1
u[2]: om_2
:return: vector dx
'''

def export_articulated_tractor_trailer_model():
    useCollocation = True
    Nx = 6
    Nu = 3
    Nc = 3  # Number of collocation points.
    Delta = 0.1
    Nseconds = 6.0
    Nt = int(Nseconds/Delta)
    Q = np.diag([150., 300., 1., 100., 1. , 100.])
    R = np.diag([25., 1., 1.])

    Lf = 0.8
    Lr = 1.3
    d1 = .5
    d2 = 1.3

    def ode_control_trailer(x, u):
        ''''
        x[0]: xt
        x[1]: yt
        x[2]: theta_r
        x[3]: gamma
        x[4]: theta_r - theta_t
        x[5]: phi (angulo de steer)
        u[0]: vf (velocidad del eje delantero en la direccion de i_f)
        u[1]: om_1
        u[2]: om_2
        :return: vector dx
        '''
        theta_r_dot = (u[0] * np.sin(x[3] + x[5]) - Lf * np.cos(x[3]) * u[1]) / (Lf * np.cos(x[3]) + Lr)
        vr = u[0] * np.cos(x[3] + x[5]) + Lf * np.sin(x[3]) * (theta_r_dot + u[1])
        theta_t_dot = np.sin(x[4]) * vr / d2 - np.cos(x[4]) * theta_r_dot * d1 / d2

        return np.array([vr * np.cos(x[2]) + d2 * np.sin(x[2] - x[4]) * theta_t_dot + d1 * np.sin(x[2]) * theta_r_dot,
                         vr * np.sin(x[2]) - d2 * np.cos(x[2] - x[4]) * theta_t_dot - d1 * np.cos(x[2]) * theta_r_dot,
                         theta_r_dot,
                         u[1],
                         theta_r_dot - theta_t_dot,
                         u[2]])


    if useCollocation:
        ode_casadi = mpc.getCasadiFunc(ode_control_trailer, [Nx, Nu], ["x", "u"], funcname="f")
    else:
        ode_casadi = mpc.getCasadiFunc(ode_control_trailer, [Nx, Nu], ["x", "u"], rk4=True, Delta=Delta, M=1)

    def lfunc(x, u, x_sp=None, u_sp=None):
        if x_sp is None:
            x_sp = np.zeros(x.shape)
        if u_sp is None:
            u_sp = np.zeros(u.shape)
        dx = x[0:6] - x_sp[0:6]
        du = u[0:3] - u_sp[0:3]
        return mpc.mtimes(dx.T, Q, dx) + mpc.mtimes(du.T, R, du)
    l = mpc.getCasadiFunc(lfunc, [Nx, Nu, Nx, Nu], ["x", "u", "x_sp", "u_sp"], funcname="l")

    def Pffunc(x, x_sp=None):
        if x_sp is None:
            x_sp = np.zeros(x.shape)
        dx = x[0:6] - x_sp[0:6]
        return mpc.mtimes(dx.T, Q, dx)
    Pf = mpc.getCasadiFunc(Pffunc, [Nx, Nx], ["x", "x_sp"], funcname="Pf")

    x0 = np.zeros(Nx)
    uprev = np.zeros(Nu)

    x_lb = np.array([-np.inf, -np.inf, -np.inf, -np.pi/3, -np.inf, -np.pi/3])
    x_ub = np.array([+np.inf, +np.inf, +np.inf, +np.pi/3, +np.inf, +np.pi/3])

    u_lb = np.array([-2., -np.deg2rad(15), -np.deg2rad(15)])
    u_ub = np.array([2., np.deg2rad(15), np.deg2rad(15)])

    Du_lb = np.array([-np.inf, -np.inf, -np.inf])
    Du_ub = np.array([np.inf, np.inf, np.inf])
    Du_lb = np.array([-0.5, -np.deg2rad(10.0), -np.deg2rad(10.0)])
    Du_ub = np.array([+0.5, +np.deg2rad(10.0), +np.deg2rad(10.0)])

    commonargs = dict(
        verbosity=0,
        l=l,
        x0=x0,
        sp={"x": np.zeros((Nt+1,1)), "u": np.zeros((Nt, Nu))},
        Pf=Pf,
        lb={"u" : np.tile(u_lb,(Nt,1)),
            "x" : np.tile(x_lb,(Nt+1,1)),
            "Du": np.tile(Du_lb,(Nt,1)) },
        ub={"u" : np.tile(u_ub,(Nt,1)),
            "x" : np.tile(x_ub,(Nt+1,1)),
            "Du": np.tile(Du_ub,(Nt,1))},
        uprev=uprev,
        )
    Nnonlin = {"t":Nt, "x":Nx, "u":Nu}
    if useCollocation:
        Nnonlin["c"] = Nc # Use collocation to discretize.

    guess = {"x": np.tile(x0[np.newaxis, :], (Nt + 1, 1)),
             "xc": np.tile(x0[np.newaxis, :, np.newaxis], (Nt, 1, Nc))}

    controller = mpc.nmpc(f=ode_casadi,N=Nnonlin,Delta=Delta,guess=guess,**commonargs)
#    controller.initialize(casadioptions={'ipopt.linear_solver': 'ma97', 'ipopt.print_level': 0, 'jit': True})

    return (controller, Nt, Delta)
