# Copyright 2026 Robot Mission System
"""Persistent waypoint storage (YAML). Thread-safe file I/O with Hot-Reloading for teach + mission."""

from __future__ import annotations

import logging
import os
import threading
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import yaml

_LOG = logging.getLogger(__name__)


@dataclass
class WaypointRecord:
    name: str
    frame_id: str
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    meta: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        d: Dict[str, Any] = {
            'name': self.name,
            'frame_id': self.frame_id,
            'pose': {
                'position': {'x': self.x, 'y': self.y, 'z': self.z},
                'orientation': {'x': self.qx, 'y': self.qy, 'z': self.qz, 'w': self.qw},
            },
        }
        if self.meta:
            d['meta'] = dict(self.meta)
        return d

    @staticmethod
    def from_dict(data: Dict[str, Any]) -> WaypointRecord:
        pose = data.get('pose') or {}
        pos = pose.get('position') or {}
        ori = pose.get('orientation') or {}
        return WaypointRecord(
            name=str(data['name']),
            frame_id=str(data.get('frame_id', 'map')),
            x=float(pos.get('x', 0.0)),
            y=float(pos.get('y', 0.0)),
            z=float(pos.get('z', 0.0)),
            qx=float(ori.get('x', 0.0)),
            qy=float(ori.get('y', 0.0)),
            qz=float(ori.get('z', 0.0)),
            qw=float(ori.get('w', 1.0)),
            meta=dict(data.get('meta') or {}),
        )


class WaypointRepository:
    """Load / save named poses. Features auto-hot-reloading to prevent RAM/Disk de-sync."""

    FILE_VERSION = 1

    def __init__(self, path: str) -> None:
        self._path = os.path.expanduser(path)
        self._lock = threading.RLock()
        self._waypoints: Dict[str, WaypointRecord] = {}
        self._last_mtime: float = -1.0  # TÁI CẤU TRÚC: Lưu trữ thời gian chỉnh sửa file cuối cùng
        self._load_or_init()

    @property
    def path(self) -> str:
        return self._path

    def _load_or_init(self) -> None:
        if not os.path.isfile(self._path):
            os.makedirs(os.path.dirname(self._path) or '.', exist_ok=True)
            self._atomic_write({'version': self.FILE_VERSION, 'waypoints': []})
        self.reload()

    def reload(self) -> None:
        """Tự động đồng bộ RAM với Ổ cứng nếu có thay đổi."""
        with self._lock:
            # 1. Nếu file bị xóa đột ngột (ví dụ: gõ lệnh rm trên terminal)
            if not os.path.isfile(self._path):
                if self._waypoints:
                    _LOG.warning('Waypoint file missing from disk! Auto-clearing RAM cache.')
                    self._waypoints.clear()
                    self._last_mtime = -1.0
                return

            # 2. Nếu file không bị thay đổi (Tối ưu hóa: bỏ qua đọc đĩa để tiết kiệm CPU)
            current_mtime = os.path.getmtime(self._path)
            if current_mtime == self._last_mtime:
                return

            # 3. Nếu file có nội dung mới (Ai đó sửa bằng nano hoặc file mới được tạo lại)
            with open(self._path, 'r', encoding='utf-8') as f:
                raw = yaml.safe_load(f) or {}
            
            wps = raw.get('waypoints') or []
            self._waypoints = {}
            for item in wps:
                try:
                    rec = WaypointRecord.from_dict(item)
                    self._waypoints[rec.name] = rec
                except (KeyError, TypeError, ValueError) as e:
                    _LOG.warning('Skip invalid waypoint entry %s: %s', item, e)
            
            self._last_mtime = current_mtime

    def list_names(self) -> List[str]:
        with self._lock:
            self.reload() # Đảm bảo luôn lấy danh sách mới nhất
            return sorted(self._waypoints.keys())

    def get(self, name: str) -> Optional[WaypointRecord]:
        with self._lock:
            self.reload() # Đảm bảo điểm chưa bị xóa khỏi đĩa
            return self._waypoints.get(name)

    def upsert(self, rec: WaypointRecord) -> None:
        with self._lock:
            self.reload() # Đọc đĩa trước khi Ghi đè để tránh "Hồi sinh" điểm cũ
            self._waypoints[rec.name] = rec
            self._persist_unlocked()

    def delete(self, name: str) -> bool:
        with self._lock:
            self.reload() # Đọc đĩa trước khi xóa
            if name not in self._waypoints:
                return False
            del self._waypoints[name]
            self._persist_unlocked()
            return True

    def _persist_unlocked(self) -> None:
        data = {
            'version': self.FILE_VERSION,
            'waypoints': [w.to_dict() for w in sorted(self._waypoints.values(), key=lambda x: x.name)],
        }
        self._atomic_write(data)
        
        # Cập nhật mtime sau khi chính mình ghi file để tránh reload thừa thãi
        if os.path.isfile(self._path):
            self._last_mtime = os.path.getmtime(self._path)

    def _atomic_write(self, data: Dict[str, Any]) -> None:
        directory = os.path.dirname(self._path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        tmp = self._path + '.tmp'
        with open(tmp, 'w', encoding='utf-8') as f:
            yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)
        backup = self._path + '.bak'
        if os.path.isfile(self._path):
            try:
                os.replace(self._path, backup)
            except OSError:
                pass
        os.replace(tmp, self._path)