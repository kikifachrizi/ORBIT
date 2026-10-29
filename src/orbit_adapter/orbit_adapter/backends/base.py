"""Interface backend robot (ADR-002). Core adapter hanya bicara lewat kelas ini."""
from abc import ABC, abstractmethod
from enum import Enum
from typing import Optional, Tuple

Pose2D = Tuple[float, float, float]  # x, y, yaw di frame peta robot


class NavStatus(Enum):
    IDLE = 'idle'
    ACTIVE = 'active'
    SUCCEEDED = 'succeeded'
    FAILED = 'failed'


class RobotBackend(ABC):

    @abstractmethod
    def pose(self) -> Optional[Pose2D]:
        """Pose terbaru, atau None bila belum ada atau sudah basi."""

    @abstractmethod
    def battery_soc(self) -> float:
        """State of charge baterai, 0.0 sampai 1.0."""

    @abstractmethod
    def navigate(self, x: float, y: float, yaw: float) -> None:
        """Mulai bergerak ke target. Perintah sebelumnya diganti."""

    @abstractmethod
    def stop(self) -> None:
        """Hentikan perintah yang sedang berjalan."""

    @abstractmethod
    def nav_status(self) -> NavStatus:
        """Status perintah navigate terakhir."""

    def nodes(self):
        """Node rclpy milik backend, untuk ditambahkan ke executor adapter."""
        return []
