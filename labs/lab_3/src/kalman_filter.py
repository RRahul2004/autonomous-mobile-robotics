

import numpy as np



# TODO Part 3: Comment the code explaining each part
# EKF with state x = [x, y, th, w, v, vdot] and measurements z = [v, w, ax, ay].
# Each cycle: predict with the motion model, then correct with the sensors.
class kalman_filter:
    
    # TODO Part 3: Initialize the covariances and the states    
    def __init__(self, P,Q,R, x, dt):
        
        self.P= P   # uncertainty of the current estimate
        self.Q= Q   # process noise: how much we distrust the motion model
        self.R= R   # measurement noise: how much we distrust the sensors
        self.x= x   # current state estimate
        self.dt = dt
        
    # TODO Part 3: Replace the matrices with Jacobians where needed        
    def predict(self):

        # Linearize around the current state (A must be taken before x is overwritten)
        self.A = self.jacobian_A()
        self.C = self.jacobian_H()
        
        self.motion_model()
        
        # Uncertainty grows when we predict without a measurement
        self.P= np.dot( np.dot(self.A, self.P), self.A.T) + self.Q

    # TODO Part 3: Calculate the Kalman gain and correct your estimates
    def update(self, z):
        
        # Gain decides how much to trust the sensors vs. the prediction
        kalman_gain= self.P @ self.C.T @ np.linalg.inv(self.C @ self.P @ self.C.T + self.R)
        
        # Move the estimate toward the measurement by the innovation (z - h(x))
        self.x= self.x + kalman_gain @ (z - self.measurement_model() )         
        # Uncertainty shrinks after using a measurement
        self.P= (np.eye(len(self.x)) - kalman_gain @ self.C) @ self.P
        
    
    # TODO Part 3: Implement here the measurement model
    def measurement_model(self):
        # What the sensors should read given the state
        x, y, th, w, v, vdot = self.x
        return np.array([
            v, # v
            w, # w
            vdot,# ax: change in forward speed
            v*w, # ay: centripetal acceleration from turning
        ])
        
    # TODO Part 3: Impelment the motion model (state-transition matrice)
    def motion_model(self):
        # Unicycle model, assuming constant w and vdot over one step
        
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        
        self.x = np.array([
            x + v* np.cos(th) * dt,
            y + v* np.sin(th) * dt,
            th + w * dt,
            w,
            v  + vdot*dt,
            vdot,
        ])
        


    
    # Jacobian of the motion model
    def jacobian_A(self):
        x, y, th, w, v, vdot = self.x
        dt = self.dt
        
        return np.array([
            #x, y,               th, w,             v, vdot
            [1, 0,              -v * np.sin(th) * dt,0,  np.cos(th) * dt,  0],
            [0, 1,              v * np.cos(th) * dt, 0,  np.sin(th) * dt,  0],
            [0, 0,                1, dt,           0,  0],
            [0, 0,                0, 1,            0,  0],
            [0, 0,                0, 0,            1,  dt],
            [0, 0,                0, 0,            0,  1 ]
        ])
    
    
    # TODO Part 3: Implement here the jacobian of the H matrix (measurements)    
    def jacobian_H(self):
        # Jacobian of the measurement model (sensors don't see x, y, th directly)
        x, y, th, w, v, vdot=self.x
        return np.array([
            #x, y,th, w, v,vdot
            [0,0,0  , 0, 1, 0], # v
            [0,0,0  , 1, 0, 0], # w
            [0,0,0  , 0, 0, 1], # ax
            [0,0,0  , v, w, 0], # ay
        ])
        
    # TODO Part 3: return the states here    
    def get_states(self):
        return self.x
