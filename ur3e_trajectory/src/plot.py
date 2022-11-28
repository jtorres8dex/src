import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('square_pose.csv')

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

ax.scatter(df['x'], df['y'], df['z'])

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.show()