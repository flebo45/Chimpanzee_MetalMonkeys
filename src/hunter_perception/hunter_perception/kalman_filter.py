"""
Hunter Perception - Kalman Filter Module
========================================

This module implements a linear Discrete Kalman Filter (DKF) for 2D object tracking.
It is designed to estimate the state of a dynamic target (Red Ball) based on noisy 
visual measurements from YOLOv8.

Mathematical Model:
-------------------
- **State Space**: The system is modeled using a Constant Velocity (CV) model.
  State Vector x = [x_pos, y_pos, x_vel, y_vel]^T
- **Process Model**: Assumes linear motion between frames. Deviations (acceleration/friction)
  are modeled as Process Noise (Q).
- **Measurement Model**: We observe only the position (pixels), not velocity.
  Measurement Vector z = [x_pos, y_pos]^T

Key Functions:
--------------
1.  **Filtering**: Smooths out jittery bounding box coordinates from YOLO.
2.  **Prediction**: Provides state estimates when the target is temporarily occluded ("Ghost Tracking").

Authors: [Metal Monkeys Team]
Version: 1.0.0
"""
import numpy as np

class KalmanFilter:
    """
    Linear Kalman Filter implementation for 2D tracking.
    
    Attributes:
        dt (float): Time step between prediction cycles (seconds).
        x (np.ndarray): State vector [x, y, vx, vy].
        P (np.ndarray): Error covariance matrix (uncertainty of the estimate).
        F (np.ndarray): State Transition Matrix (Physics model).
        H (np.ndarray): Measurement Matrix (Mapping state to sensors).
        Q (np.ndarray): Process Noise Covariance (Model uncertainty).
        R (np.ndarray): Measurement Noise Covariance (Sensor noise).
    """
    def __init__(self, dt: float = 0.1):
        """
        Initialize the Kalman Filter with given time step.

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
        Time Update Step (A Priori)
        
        Projects the current state and error covariance forward in time 
        using the physics model (Matrix F). This step is performed 
        even if no measurement is available (Blind Prediction).

        Equations:
            x_{k|k-1} = F * x_{k-1|k-1}
            P_{k|k-1} = F * P_{k-1|k-1} * F^T + Q
            
        Returns:
            tuple: Predicted (x, y) coordinates.
        """

        #Predict the state
        self.x = np.dot(self.F, self.x)

        #Predict the error covariance
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.get_state()
    
    def update(self, z_meas):
        """
        Measurement Update Step (A Posteriori)
        
        Corrects the predicted state using the actual measurement from YOLO.
        Computes the Kalman Gain (K) to optimally fuse prediction and measurement.

        Args:
            z_meas (tuple): The observed (x, y) pixel coordinates.
            
        Equations:
            y = z - H * x_pred          (Innovation/Residual)
            S = H * P * H^T + R         (Innovation Covariance)
            K = P * H^T * S^-1          (Optimal Kalman Gain)
            x_new = x_pred + K * y      (State Update)
            P_new = (I - K * H) * P     (Covariance Update)
        """

        # Convert measurement to column vector
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
        Helper to retrieve the current position estimate as integers.
        
        Returns:
            tuple(int, int): The estimated (x, y) pixel coordinates.
        """
        return int(self.x[0,0]), int(self.x[1,0])