from time import sleep

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

data = pd.read_csv('rele.csv')
plt.plot(data['time'], data['angle'])
y = 90 * np.ones_like(data['time'])
plt.plot(data['time'], y, 'r--', label='y=90')
plt.xlim(0)
plt.ylim(0)
plt.show()
sleep(10)
