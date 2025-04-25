import numpy as np
import matplotlib.pyplot as plt

# Load data
A = np.loadtxt("thisCOMpos.txt")

sA = A.shape
tA = np.arange(1, sA[0] + 1)

# Plotting
plt.figure()

# First subplot
plt.subplot(3, 1, 1)
plt.plot(A[:, 0])
plt.title("X Position")

# Second subplot
plt.subplot(3, 1, 2)
plt.plot(A[:, 1])
plt.title("Y Position")

# Third subplot
plt.subplot(3, 1, 3)
plt.plot(A[:, 2])
plt.title("Z Position")

plt.show()