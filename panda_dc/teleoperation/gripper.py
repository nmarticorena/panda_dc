import panda_py
import pyrobotiqgripper as robotiq
from abc import abstractmethod, ABC

class Gripper(ABC):
    @abstractmethod
    def open(self) -> None:
        ...

    @abstractmethod
    def close(self) -> None:
        ...

    @abstractmethod
    def move(self, width: float, speed: float = 0.1) -> None:
        ...

    @abstractmethod
    def grasp(
        self,
        width: float,
        speed: float,
        force: float,
        **kwargs    
    ) -> bool:
        ...

    @abstractmethod
    def read(self):
        """Return the current gripper state."""
        ...

    @abstractmethod
    def width_m(self) -> float:
        """Return the current gripper width in meters."""
        ...

class FrankaGripper(Gripper):
    def __init__(self, hostname: str):
        self._gripper = panda_py.libfranka.Gripper(hostname)

    def open(self):
        self._gripper.move(0.08, 0.1)

    def close(self):
        self._gripper.grasp(0.0, 0.1, 40)

    def move(self, width, speed=0.1):
        self._gripper.move(width, speed)

    def grasp(self, width, speed, force, epsilon_inner=0.005, epsilon_outer=0.005):
        return self._gripper.grasp(
            width, speed, force, epsilon_inner, epsilon_outer
        )

    def read(self):
        return self._gripper.read_once()
    
    def width_m(self) -> float:
        return self._gripper.read_once().width

class NoneGripper(Gripper):
    def open(self):
        pass

    def close(self):
        pass

    def move(self, width, speed=0.1):
        pass

    def grasp(self, width, speed, force, **kwargs):
        return True

    def read(self):
        return None

    def width_m(self) -> float:
        return 0.0

class RobotiqGripper(Gripper):
    def __init__(self):
        self._gripper = robotiq.RobotiqGripper()
        if not self._gripper.isActivated():
            print("Activating Robotiq gripper...")
            self._gripper.activate()
        if not self._gripper.is_mm_calibrated():
            print("Calibrating Robotiq gripper...")
            self._gripper.calibrate_mm(0, 85)

    def open(self):
        self._gripper.open()

    def close(self):
        self._gripper.close()

    def move(self, width, speed=0.1):
        pos = int((1.0 - width / 0.085) * 255)
        self._gripper.move(pos, speed)

    def grasp(self, width, speed, force, **kwargs):
        pos = int((1.0 - width / 0.085) * 255)
        return self._gripper.grasp(pos, speed, force)

    def read(self):
        return self._gripper.status()

    def width_m(self) -> float:
        return self._gripper.position_mm() / 1000.0
