import matplotlib.pyplot as plt

# Generate distances in meters and convert them to centimeters
distances_cm = [(0 + 0.01 * k) * 100 for k in range(60)]  # Now distances_cm is in centimeters

def vertical_control(distance_cm, height=1):
    # Convert distance from cm to meters before using it in the formula
    distance_m = distance_cm / 100
    return min((distance_m ** 2 - 0.25), 0.0) * min(1.0, height + 0.5)

# Calculate controls for each distance in centimeters
controls = [vertical_control(distance) for distance in distances_cm]

plt.plot(distances_cm, controls)
plt.grid()
plt.xlabel('Distance (centimeters)')
plt.ylabel('Vertical control (m/s)')
plt.title('Vertical Control as a Function of Distance (for a high height)')

plt.show()
