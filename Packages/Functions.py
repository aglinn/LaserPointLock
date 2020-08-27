from Packages.UpdateManager import UpdateManager

#  TODO: I should close cameras and com ports etc.


def shut_down():
    UpdateManager.store_data()
    return