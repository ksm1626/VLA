"""YAML config helpers for remote SO101 runners."""

from __future__ import annotations

from pathlib import Path
from typing import Any


def load_yaml(path: str | Path) -> dict[str, Any]:
    """Load a YAML mapping."""
    config_path = Path(path)
    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {config_path}")
    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError("PyYAML is required to read config files.") from exc

    with config_path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"Config must be a YAML mapping: {config_path}")
    return data


def section(config: dict[str, Any], name: str) -> dict[str, Any]:
    """Return a required mapping section."""
    value = config.get(name)
    if not isinstance(value, dict):
        raise ValueError(f"Missing or invalid `{name}` config section")
    return value


def value(section_data: dict[str, Any], key: str, section_name: str) -> Any:
    """Return a required non-empty config value."""
    item = section_data.get(key)
    if item is None or item == "":
        raise ValueError(f"Missing required `{section_name}.{key}` value")
    return item
