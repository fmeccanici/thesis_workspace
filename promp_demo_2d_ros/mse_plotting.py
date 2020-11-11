import matplotlib.pyplot as plt
import numpy as np

mse = lambda y_20, y_36, y1, y2: 0.5 * ((y_20 - y1)**2 + (y_36 - y2)**2)

mse_plotting = [100, mse(4.7, 5.1, 10, 0), mse(12.0, -9, 10, -10), mse(-0.4, 10.0, 0, 10), 
                mse(10.65, -10, 10, -10), mse(0, 0, 0, 0), mse(-10.39, 10.1, -10, 10), 
                mse(-10, 0, -10, 0), mse(-9.56, -10.24, -10, -10)]

plt.plot(np.arange(1, 10), mse_plotting)
plt.ylabel("MSE [-]", fontsize=30)
plt.xlabel("Observed conditions [-]", fontsize=30)
plt.xticks([1, 2, 3, 4, 5, 6, 7, 8, 9], fontsize=20)
plt.yticks([0, 20, 40, 60, 80, 100], fontsize=20)
plt.grid()
plt.tight_layout()
plt.savefig('figures/mse.pdf')