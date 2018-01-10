from Px4Gimbal import Px4Gimbal
import time

gimbal = Px4Gimbal("/dev/ttyAMA0")

gimbal.start()