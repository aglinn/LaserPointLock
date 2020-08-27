from Packages.UpdateManager import UpdateManager
from Packages.UpdateManager import InsufficientInformation

manager = UpdateManager()

manager.dx = [0, 1, 2, 3]

manager.dx = [4, 5, 6, 7]

manager.dx = [8, 9, 10, 11.7]

manager.t1 = 1
manager.t1 = 2
manager.t1 = 3.6

manager.store_data()