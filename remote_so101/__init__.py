"""A6000-side remote SO101 helpers.

The LeRobot adapter is imported lazily so SO101-side gateway code can reuse
proto/config helpers without importing LeRobot.
"""

from __future__ import annotations

from typing import Any

__all__ = ["RemoteSO101", "RemoteSO101Config"]


def __getattr__(name: str) -> Any:
    """Lazily expose the LeRobot adapter classes."""
    if name == "RemoteSO101":
        from remote_so101.remote_robot import RemoteSO101

        return RemoteSO101
    if name == "RemoteSO101Config":
        from remote_so101.config import RemoteSO101Config

        return RemoteSO101Config
    raise AttributeError(name)
