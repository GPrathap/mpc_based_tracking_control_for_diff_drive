from time import time
import casadi as ca
import numpy as np
from casadi import sin, cos, pi
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class MPCDiffDriveControl():
    def __init__(self, deltat, horizon, v_max):
        self.T = deltat
        self.N = horizon
        self.v_max = v_max
        self.v_min = -self.v_max
        self.omega_max = pi/2
        self.omega_min = -self.omega_max
        self.map_dim = [-200, 200, -200, 200]
        self.init_reg = False

        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')
        states = ca.vertcat(x, y, theta)
        self.n_status = states.size()[0]

        v = ca.SX.sym('v_x')
        omega = ca.SX.sym('omega')

        controls = ca.vertcat(v, omega)
        self.n_controls = controls.size()[0]
        rhs = ca.vertcat(v*cos(theta), v*sin(theta), omega)

        self.f = ca.Function('f', [states, controls], [rhs])

        U = ca.SX.sym('U', self.n_controls, self.N)
        P = ca.SX.sym('P', self.n_status + self.n_status)
        X = ca.SX.sym('X', self.n_status, self.N+1)

        Q = ca.DM.zeros(3,3)
        Q[0,0] = 1
        Q[1,1] = 1
        Q[2,2] = 0.1
       
        R = ca.DM.zeros(2,2)
        R[0,0] = 0.05
        R[1,1] = 0.05

        obj = 0
        g = ca.SX.sym('g', self.N+1, self.n_status)
        st = X[:,0]

        g[0,:] = st - P[0:self.n_status]
        ibj = 1
        for k in range(0, self.N):
            st = X[:,k]
            con = U[:,k]
            obj = obj + ca.mtimes((st-P[3:6]).T, ca.mtimes(Q,(st-P[3:6]))) + ca.mtimes((con).T, ca.mtimes(R, (con))) 
            st_next = X[:,k+1]
            f_value = self.f(st, con)
            st_next_euler = st + self.T*f_value
            g[ibj,:] = st_next - st_next_euler
            ibj += 1

        g = ca.reshape(g, self.n_status*(self.N+1), 1)
        OPT_variables = ca.vertcat(ca.reshape(X, self.n_status*(self.N+1), 1), ca.reshape(U, self.n_controls*self.N, 1))

        opts = {}
        opts["expand"] = True
        opts["ipopt.max_iter"] = 2000
        opts["ipopt.tol"] = 1e-4
        opts["ipopt.print_level"] = 0
        opts["print_time"] = 0
        opts["ipopt.acceptable_tol"] = 1e-8

        nlp_prob = {'f':obj, 'x':OPT_variables, 'p':P, 'g':g}
        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

        lbg = ca.DM(1, self.n_status*(self.N+1))
        ubg = ca.DM(1, self.n_status*(self.N+1))

        lbg[0,0:self.n_status*(self.N+1):1] = 0
        ubg[0,0:self.n_status*(self.N+1):1] = 0

        lbx = ca.DM(self.n_status*(self.N+1)+self.n_controls*self.N,1)
        ubx = ca.DM(self.n_status*(self.N+1)+self.n_controls*self.N,1)

        lbx[0:self.n_status*(self.N+1):self.n_status,0] = self.map_dim[0]
        ubx[0:self.n_status*(self.N+1):self.n_status,0] = self.map_dim[1]
        lbx[1:self.n_status*(self.N+1):self.n_status,0] = self.map_dim[2]
        ubx[1:self.n_status*(self.N+1):self.n_status,0] = self.map_dim[3]
        lbx[2:self.n_status*(self.N+1):self.n_status,0] = -ca.inf
        ubx[2:self.n_status*(self.N+1):self.n_status,0] = ca.inf

        lbx[self.n_status*(self.N+1):self.n_status*(self.N+1) + self.n_controls*self.N:self.n_controls,0] = self.v_min
        ubx[self.n_status*(self.N+1):self.n_status*(self.N+1) + self.n_controls*self.N:self.n_controls,0] = self.v_max
        lbx[self.n_status*(self.N+1)+1:self.n_status*(self.N+1) +self.n_controls*self.N:self.n_controls,0] = self.omega_min
        ubx[self.n_status*(self.N+1)+1:self.n_status*(self.N+1) +self.n_controls*self.N:self.n_controls,0] = self.omega_max
        self.args = {'lbx':lbx, 'ubx':ubx, 'lbg':lbg, 'ubg':ubg, 'p':[], 'x0':[0.5, 1.0, 0.0]}
        
    def shift(self, T, t0, x0, u):
        st = x0
        con = u[0,:].T 
        f_value = self.f(st, con)
        st = st + (T*f_value)
        x0 = st
        t0 = t0 + T
        u_rest = u[1:u.size()[0]:1,:]
        u_last = u[u.size()[0]-1:u.size()[0]:1,:]
        self.u0 = ca.vertcat(u_rest, u_last)
        return t0, x0, self.u0
        
    def init_regulator(self, start_pose, target_pose):
        self.init_reg = True
        self.t0 = 0
        self.x0 = ca.DM([[start_pose[0]], [start_pose[1]], [start_pose[2]]])
        self.xs = ca.DM([[target_pose[0]], [target_pose[1]], [target_pose[2]]])

        xx = ca.DM(self.n_status, 300)
        xx[:,0] = self.x0
        t = ca.DM(1, 300)
        t[0] = self.t0
        self.u0 = ca.DM.zeros(self.N, self.n_controls)
        self.X0 = ca.repmat(self.x0, 1, self.N+1).T
        self.mpciter = 0
        self.error_ = ca.norm_2(self.x0-self.xs)
        
    def update(self, odom_pose):
        self.x0 = ca.DM([[odom_pose[0]], [odom_pose[1]], [odom_pose[2]]]) 
        self.args['p'] = ca.vertcat(self.x0, self.xs)
        self.args['x0'] = ca.vertcat(ca.reshape(self.X0.T, self.n_status*(self.N+1), 1), ca.reshape(self.u0.T, self.n_controls*self.N, 1))
        sol = self.solver(**self.args)
        u = ca.reshape(sol['x'][self.n_status*(self.N+1):sol['x'].size()[0]:1].T, self.n_controls, self.N).T
        
        self.t0, self.x0, self.u0 = self.shift(self.T, self.t0, self.x0, u) 
        self.X0 = ca.reshape(sol['x'][0:self.n_status*(self.N+1)].T, self.n_status, self.N+1).T
        self.X0 = ca.reshape(sol['x'][0:self.n_status*(self.N+1):1].T, self.n_status, self.N+1).T
        x0_rest = self.X0[1:self.X0.size()[0]:1,:]
        x0_last = self.X0[self.X0.size()[0]-1:self.X0.size()[0]:1,:]
        self.X0 = ca.vertcat(x0_rest, x0_last)
        self.mpciter = self.mpciter + 1
        self.error_ =  ca.norm_2(self.x0-self.xs)
        return  self.u0[0,:].full().flatten(), self.x0.full().flatten()
    