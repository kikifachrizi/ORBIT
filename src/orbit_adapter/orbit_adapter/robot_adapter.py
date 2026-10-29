"""Jembatan satu robot: callback EasyFullControl RMF <-> RobotBackend (ADR-002).

Transformasi koordinat nav graph <-> peta robot dilakukan RMF sendiri dari
reference_coordinates di config fleet, jadi posisi di sini sudah di frame peta robot.
"""
import threading
import time

import rmf_adapter.easy_full_control as rmf_easy

from .backends.base import NavStatus


class RobotAdapter:

    def __init__(self, name, backend, fleet_handle, robot_config, map_name, logger,
                 max_retries=3, retry_delay=2.0):
        self.name = name
        self.backend = backend
        self.fleet_handle = fleet_handle
        self.robot_config = robot_config
        self.map_name = map_name
        self.logger = logger
        self.max_retries = max_retries
        self.retry_delay = retry_delay

        self.update_handle = None
        # RLock: RMF bisa memanggil navigate/stop secara sinkron dari dalam update()
        self._lock = threading.RLock()
        self._execution = None
        self._target = None
        self._retries = 0
        self._retry_at = None

    def update(self):
        """Dipanggil periodik oleh fleet adapter."""
        pose = self.backend.pose()
        if pose is None:
            return
        state = rmf_easy.RobotState(self.map_name, list(pose), self.backend.battery_soc())

        if self.update_handle is None:
            # Robot didaftarkan ke RMF setelah pose pertama tersedia (robot sudah terlokalisasi)
            callbacks = rmf_easy.RobotCallbacks(self._navigate, self._stop, self._execute_action)
            self.update_handle = self.fleet_handle.add_robot(
                self.name, state, self.robot_config, callbacks)
            self.logger.info(f'{self.name} terdaftar di RMF pada {pose}')
            return

        with self._lock:
            self._check_navigation()
            activity = self._execution.identifier if self._execution else None
        self.update_handle.update(state, activity)

    def _check_navigation(self):
        if self._execution is None:
            return
        status = self.backend.nav_status()
        if status == NavStatus.SUCCEEDED:
            self._execution.finished()
            self._execution = None
        elif status == NavStatus.FAILED:
            if self._retry_at is None:
                if self._retries >= self.max_retries:
                    # RMF membandingkan posisi aktual dan merencanakan ulang dari sana
                    self.logger.error(f'{self.name}: Nav2 gagal {self._retries + 1}x, perintah diakhiri')
                    self._execution.finished()
                    self._execution = None
                    return
                self._retries += 1
                self._retry_at = time.monotonic() + self.retry_delay
                self.logger.warning(
                    f'{self.name}: Nav2 gagal, coba lagi ({self._retries}/{self.max_retries})')
            elif time.monotonic() >= self._retry_at:
                self._retry_at = None
                self.backend.navigate(*self._target)

    def _navigate(self, destination, execution):
        x, y, yaw = (float(v) for v in destination.position)
        with self._lock:
            self._execution = execution
            self._target = (x, y, yaw)
            self._retries = 0
            self._retry_at = None
        target = destination.name or '(tanpa nama)'
        self.logger.info(f'{self.name}: navigate ke {target} ({x:.2f}, {y:.2f}, {yaw:.2f})')
        self.backend.navigate(x, y, yaw)

    def _stop(self, activity):
        with self._lock:
            if self._execution is None or not self._execution.identifier.is_same(activity):
                return
            self._execution = None
        self.logger.info(f'{self.name}: stop')
        self.backend.stop()

    def _execute_action(self, category, description, execution):
        # Belum ada aksi khusus yang didukung; aksi langsung dianggap selesai
        self.logger.warning(f'{self.name}: aksi {category!r} belum didukung, dilewati')
        execution.finished()
