import pandas as pd
import matplotlib.pyplot as plt

# Load the data
file_path = '~/Documents/dreams-pod/attitude_brightness_data.csv'
data = pd.read_csv(file_path)

# Extract yaw, pitch, and brightness
yaw = data['Yaw']
pitch = data['Pitch']
brightness = data['Brightness']

# Create the scatter plot
plt.figure(figsize=(10, 6))
scatter = plt.scatter(yaw, pitch, c=brightness, cmap='viridis')
plt.colorbar(scatter, label='Brightness')
plt.xlabel('Yaw')
plt.ylabel('Pitch')
plt.title('Yaw vs. Pitch Colored by Brightness')
plt.grid(True)
plt.show()

