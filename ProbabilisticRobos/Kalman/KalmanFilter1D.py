import numpy as np

"""

This is an exercice for the implementation of a Kalman Linear Filter 
based on the algorithm presented in the book Probabilistic Robotics on 
Chapter 3, page 36, table 3.1.

The only difference with the algorithm shown in the table and this implementation will be the 
mean calculation, the prediction will be done with a state vector of the previous state.

"""



"""
1.- Define Linear Kalman Parameters
"""
# Define time dimension for the simulation
# Number of iterations of the Kalman Filter
iterations = 1000
t_start = 0 
t_end = 10
time = np.linspace(t_start,t_end, iterations)
# Calculate sample dt for derivatives in simulation
dt = t_end/iterations

# Define predicted and calculated position arrays for plotting purposes
x_true = np.zeros((time.size,2))
x_pred = np.zeros(time.shape)
x_filtered = np.zeros(time.shape)

# Define initial variables
x0 = 0.0    # Initial Position
v0 = 2.0    # Initial Velocity
a0 = 0.0    # Initial Acceleration

# Add the initial position to 
x_true[0] = [x0, v0]

# Matrix A - For the state vector 
A = np.array([[1, dt], [0 ,1]])
# Matrix B - For the control vector
B = np.array([ [(dt**2)/2], [dt] ])
# Matrix C - Meassurement matrix
C = np.array([0,1])
# Q matrix - covariance matrix
sigma_model_x = 0.25    # Position model error variance
sigma_modal_v = 0.25    # Velocity model error variance
Q = np.array([[ sigma_model_x**2, 0],[0,sigma_modal_v**2 ]])

# P matrix - initial process covariance matrix
P = np.eye(2)
# R matrix - meassurement covariance 
sigma_meas = 0.3
R = sigma_meas**2
# Current state vector
state = np.array([[0],[v0]])

# Control vector
u = np.array([a0])

"""
# 1 iteration of a Linear Kalman Filter
"""
# --------------------------
# 1.- Sensor data simulation
# --------------------------
# Get the true value of the current velocity and add noise from a random number of a standar deviation
# Multiply sigma_meas with a random number from the standard distribuition to change sigma=1 for sigma_meas
z = np.dot( np.transpose(C), x_true[0]) + sigma_meas*np.random.randn(1)
# -------------------------
# 2.- Next State prediction
# -------------------------
# Calculate the current state prediction 
xPred = np.dot(A,state) + np.dot(B,u)
# Calculate the predicted covariance of the process
PPred = np.dot( np.dot(A,P), np.transpose(A) ) + Q
# -------------------------------
# 3.- Kalman constant update
# -------------------------------
K = np.dot(PPred,np.transpose(C))
k = np.linalg.inv( np.dot( np.dot(C,PPred), np.transpose(C)) ) 
