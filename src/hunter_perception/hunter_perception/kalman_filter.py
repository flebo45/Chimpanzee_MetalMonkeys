import numpy as np

class KalmanFilter:
    def __init__(self, dt: float = 0.1):
        """
        Kalman Filter for 2d tracking (Constant Velocity Model)
        State Vector: [x, y, vx, vy]
        Args:
            dt (float): Time step between measurements
        """
        self.dt = dt

        #State vector [x, y, vx, vy]
        self.x = np.zeros((4, 1))

        #State Transition Matrix
        #x_new = x + vx*dt
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        #Measurement Matrix
        #We can only measure position (x, y)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])

        #Covariance Matrix
        self.P = np.eye(4) * 1000.0  # High uncertainty in initial

        #Process Noise Covariance
        self.Q = np.eye(4) * 1.0
        
        #Noise Covariance
        self.R = np.array([
            [10, 0],
            [0, 10]
        ])

    def predict(self):
        """
        Predict: Predict target on past velocity
        """

        #Predict the state
        self.x = np.dot(self.F, self.x)

        #Predict the error covariance
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.get_state()
    
    def update(self, z_meas):
        """
        Correct: Correct the state with measurement
        Args:
            z_meas: Tuple (x, y) measurement
        """
        z = np.array([[z_meas[0]], [z_meas[1]]])

        #Error mesurement (y = z - Hx)
        y = z - np.dot(self.H, self.x)

        #Kalman Gain
        #S = H*P*H.T + R
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        #K = P*H.T*inv(S)
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))

        #Update the state
        #x = x + K*y
        self.x = self.x + np.dot(K, y)

        #Update the error covariance
        #P = (I - K*H)*P
        I = np.eye(self.F.shape[0])
        self.P = np.dot(I - np.dot(K, self.H), self.P)

    def get_state(self):
        """
        Get current state
        Returns:
            Tuple (x, y) position
        """
        return int(self.x[0,0]), int(self.x[1,0])