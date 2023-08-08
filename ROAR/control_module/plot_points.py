import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

path = "C:\\Users\\artme\\Documents\\CarCode\\ROAR\\configurations\\major_center_waypoints_1LAPONLY_righted.txt"

# df = pd.read_csv(path, names=['X', 'Z', 'Y', 'ROLL', 'PITCH', 'YAW'], header=None)
# df['Z'] *= -1
df = pd.read_csv(path, names=['Y', 'Z', 'X', 'ROLL', 'PITCH', 'YAW'], header=None)
# print(df)

ax = plt.axes(projection='3d')

# Data for a three-dimensional line
# zline = np.linspace(0, 15, 1000)
# xline = np.sin(zline)
# yline = np.cos(zline)
# ax.plot3D(x, y, z, 'gray')

# Data for three-dimensional scattered points
# zdata = 15 * np.random.random(100)
# xdata = np.sin(zdata) + 0.1 * np.random.randn(100)
# ydata = np.cos(zdata) + 0.1 * np.random.randn(100)
# 26530
# 26946
start = 22400 
p1 = start + 3
p2 = 26436
p3 = 26456
end = 24690
st_x = df['X'][start:p1]
st_y = df['Y'][start:p1]
st_z = df['Z'][start:p1]
xdata = df['X'][p2:p3]
ydata = df['Y'][p2:p3]
zdata = df['Z'][p2:p3]

x1 = df['X'][p1:p2]
y1 = df['Y'][p1:p2]
z1 = df['Z'][p1:p2]

m_x = df['X'][p2:p3]
m_y = df['Y'][p2:p3]
m_z = df['Z'][p2:p3]

x2 = df['X'][p2+5:end]
y2 = df['Y'][p2+5:end]
z2 = df['Z'][p2+5:end]

ax.set_zlim(50, 450)

# z = np.polyfit(xdata, ydata, zdata, 1)
# p = np.poly1d(z)
# plt.plot(xdata, ydata, p(xdata, ydata))

# # print(xdata)
ax.scatter3D(st_x, st_y, st_z, c='r')
ax.scatter3D(x1, y1, z1, c='blue')
ax.scatter3D(m_x, m_y, m_z, c='g')
ax.scatter3D(x2, y2, z2, c='g')

plt.show()

