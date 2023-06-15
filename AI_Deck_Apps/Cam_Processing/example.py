import numpy as np
import matplotlib.pyplot as plt

# Create a 1-D array of size 400 (20x20)
arr = np.array([i // 20 for i in range(400)])

# Reshape the array to 20x20
arr = arr.reshape(20, 20)

# Plot the array using matplotlib's imshow
plt.imshow(arr, cmap='gray')
plt.colorbar(label='Value')
plt.title('2D Array Visualization')
plt.show()
