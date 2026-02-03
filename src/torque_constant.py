import numpy as np
from sklearn.linear_model import LinearRegression

# 1. Your Data (Replace with your real measurements)
# Mass (kg) -> Torque (Nm)
weights_kg = np.array([0.200, 0.305, 0.410, 0.515, 0.620])
currents_a = np.array([0.49, 0.74, 1.33, 1.79, 2.00]) # Example values from before

radius = 0.28  # meters
g = 9.81

# 2. Convert to Torque
torques = weights_kg * g * radius
X = torques.reshape(-1, 1)  # X axis = Torque
y = currents_a              # Y axis = Current

# 3. Fit Line
model = LinearRegression()
model.fit(X, y)

# 4. Extract True Constants
slope = model.coef_[0]      # This is Amps per Nm
intercept = model.intercept_ # This is Friction Current

true_kt = 1 / slope

print(f"True Kt: {true_kt:.3f} Nm/A")
print(f"Internal Friction: {intercept:.3f} A")