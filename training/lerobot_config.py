"""Small config helpers for LeRobot CLI wrappers."""

from __future__ import annotations

from pathlib import Path
from typing import Any
import sys


def load_yaml_config(path: str | Path) -> dict[str, Any]:
    """Load a YAML config with a clear error if PyYAML is missing."""
    config_path = Path(path)
    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {config_path}")

    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError(
            "PyYAML is required to read config files. Install it with `pip install pyyaml`."
        ) from exc

    with config_path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)

    if not isinstance(data, dict):
        raise ValueError(f"Config must be a YAML mapping: {config_path}")
    return data


def require_mapping(config: dict[str, Any], key: str) -> dict[str, Any]:
    """Return a required mapping section."""
    value = config.get(key)
    if not isinstance(value, dict):
        raise ValueError(f"Missing or invalid `{key}` section in config")
    return value


def require_value(section: dict[str, Any], key: str, section_name: str) -> Any:
    """Return a required non-empty config value."""
    value = section.get(key)
    if value is None or value == "":
        raise ValueError(f"Missing required `{section_name}.{key}` value")
    return value


def optional_bool(value: Any) -> str:
    """Convert Python booleans to the lowercase strings expected by CLI flags."""
    if isinstance(value, bool):
        return str(value).lower()
    return str(value)


def env_executable(command_name: str) -> str:
    """Prefer an executable from the active Python environment over user PATH."""
    candidate = Path(sys.prefix) / "bin" / command_name
    if candidate.exists():
        return str(candidate)
    return command_name
