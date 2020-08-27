from Packages.UpdateManager import UpdateManager
from Packages.UpdateManager import InsufficientInformation

manager = UpdateManager()

manager.dx = [0, 1, 2, 3]

manager.dx = [4, 5, 6, 7]

manager.dx = [8, 9, 10, 11]

manager.dx = [12, 13, 14, 15]

manager.dx = [16, 17, 18, 19]

manager.t1 = 1
manager.t1 = 2
manager.t1 = 3.6

print(manager.dx)
print(manager.standard_deviation)