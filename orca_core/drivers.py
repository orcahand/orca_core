from __future__ import annotations

from typing import Callable

from .hardware.motor_client import MotorClient
from .profile import DriverConfig, HandProfile


DriverFactory = Callable[[HandProfile, DriverConfig, str | None], MotorClient]

_DRIVER_FACTORIES: dict[str, DriverFactory] = {}


def register_driver(kind: str, factory: DriverFactory) -> None:
    _DRIVER_FACTORIES[kind] = factory


def get_registered_drivers() -> list[str]:
    return sorted(_DRIVER_FACTORIES)


def create_motor_client(
    profile: HandProfile,
    *,
    port: str | None = None,
    driver: MotorClient | None = None,
) -> MotorClient:
    if driver is not None:
        return driver

    factory = _DRIVER_FACTORIES.get(profile.driver.kind)
    if factory is None:
        registered = ", ".join(get_registered_drivers())
        raise ValueError(
            f"Unknown driver kind '{profile.driver.kind}'."
            + (f" Registered drivers: {registered}" if registered else "")
        )
    return factory(profile, profile.driver, port)


def _create_dynamixel_client(profile: HandProfile, driver_config: DriverConfig, port: str | None) -> MotorClient:
    from .hardware.dynamixel_client import DynamixelClient

    return DynamixelClient(profile.motor_ids, port or driver_config.port or "/dev/ttyUSB0", driver_config.baudrate)


def _create_feetech_client(profile: HandProfile, driver_config: DriverConfig, port: str | None) -> MotorClient:
    from .hardware.feetech_client import FeetechClient

    return FeetechClient(profile.motor_ids, port or driver_config.port or "/dev/ttyUSB0", driver_config.baudrate)


def _create_mock_dynamixel_client(profile: HandProfile, driver_config: DriverConfig, port: str | None) -> MotorClient:
    from .hardware.mock_dynamixel_client import MockDynamixelClient

    return MockDynamixelClient(profile.motor_ids, port or driver_config.port or "/dev/ttyUSB0", driver_config.baudrate)


register_driver("dynamixel", _create_dynamixel_client)
register_driver("feetech", _create_feetech_client)
register_driver("mock_dynamixel", _create_mock_dynamixel_client)
