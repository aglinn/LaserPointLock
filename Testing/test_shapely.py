import shapely.errors
from shapely.geometry import Polygon
from shapely import validation
import numpy as np
A = Polygon([(0, 0), (10, 1), (9, 6), (-5, 4)])
A = Polygon([(0, 0), (9, 6), (10, 1), (-5, 4)])
#B = Polygon([(1, 1), (11, 2), (10, 7), (-4, 5)])
#B = Polygon([(9, 6), (12, 8), (13, 12), (8, 9)])
B = Polygon([(10, 1), (9, 6), (15, 10), (13, 0)])

corners = np.asarray(list(B.exterior.coords)[0:-1])
null_vecs = np.arange(12).reshape(6, 2)
temp = np.matmul(null_vecs, corners.transpose())
temp += np.array([10, 50, 60, 8, 97, 47]).reshape(6, 1)
temp = np.sum(temp, axis=0)
print(temp.shape)
print(corners, null_vecs, np.matmul(null_vecs, corners.transpose()))



if 'Self-intersection' in validation.explain_validity(A):
    print("caught error")
    A = Polygon([(0, 0), (10, 1), (9, 6), (-5, 4)])
else:
    print(validation.explain_validity(A))
import matplotlib.pyplot as plt

fig, ax = plt.subplots()

x = []
y = []
for point in list(A.exterior.coords):
    x.append(point[0])
    y.append(point[1])

plt.plot(x, y, 'r')

x = []
y = []
for point in list(B.exterior.coords):
    x.append(point[0])
    y.append(point[1])
plt.plot(x, y, 'b')

print(A.intersects(B))
C = A.intersection(B)
if C.type == 'Point':
    print("caught the type")
else:
    print(C.type)
corners = list(C._get_coords())
print(np.sum(corners, axis=1))
print("solution is", corners[np.argmin(np.sum(corners, axis=1))])
plt.plot(list(C._get_coords())[0][0], list(C._get_coords())[0][1], 'x')

x = []
y = []
if C.type == 'Polygon':
    for point in list(A.intersection(B).exterior.coords):
        x.append(point[0])
        y.append(point[1])
    plt.plot(x, y, 'g')

plt.show()