"""Sample API Client."""

import logging
import threading
import time
import serial
from types import SimpleNamespace

from homeassistant.core import HomeAssistant
import sunspec2.modbus.client as modbus_client
from sunspec2.modbus.client import SunSpecModbusClientException
from sunspec2.modbus.client import SunSpecModbusClientTimeout
from sunspec2.modbus.modbus import ModbusClientError

TIMEOUT = 120

_LOGGER: logging.Logger = logging.getLogger(__package__)


class ConnectionTimeoutError(Exception):
    pass


class ConnectionError(Exception):
    pass


class SunSpecModelWrapper:
    def __init__(self, models) -> None:
        """Sunspec model wrapper"""
        self._models = models
        self.num_models = len(models)

    def isValidPoint(self, point_name):
        point = self.getPoint(point_name)
        if point.value is None:
            return False
        if point.pdef["type"] in ("enum16", "bitfield32"):
            return True
        if point.pdef.get("units", None) is None:
            return False
        return True

    def getKeys(self):
        keys = list(filter(self.isValidPoint, self._models[0].points.keys()))
        for group_name in self._models[0].groups:
            model_group = self._models[0].groups[group_name]
            if type(model_group) is list:
                for idx, group in enumerate(model_group):
                    key_prefix = f"{group_name}:{idx}"
                    group_keys = map(
                        lambda gp: f"{key_prefix}:{gp}", group.points.keys()
                    )
                    keys.extend(filter(self.isValidPoint, group_keys))
            else:
                key_prefix = f"{group_name}:0"
                group_keys = map(
                    lambda gp: f"{key_prefix}:{gp}", model_group.points.keys()
                )
                keys.extend(filter(self.isValidPoint, group_keys))
        return keys

    def getValue(self, point_name, model_index=0):
        point = self.getPoint(point_name, model_index)
        return point.cvalue

    def getMeta(self, point_name):
        return self.getPoint(point_name).pdef

    def getGroupMeta(self):
        return self._models[0].gdef

    def getPoint(self, point_name, model_index=0):
        point_path = point_name.split(":")
        if len(point_path) == 1:
            return self._models[model_index].points[point_name]

        group = self._models[model_index].groups[point_path[0]]
        if type(group) is list:
            return group[int(point_path[1])].points[point_path[2]]
        else:
            if len(point_path) > 2:
                return group.points[
                    point_path[2]
                ]  # Access to the specific point within the group
            return group.points[
                point_name
            ]  # Generic access if no specific subgrouping is specified


# pragma: not covered
def progress(msg):
    _LOGGER.error(msg)
    return True


class SunSpecApiClient:
    CLIENT_CACHE = {}

    def __init__(
        self, adapter: str, slave_id: int, hass: HomeAssistant
    ) -> None:
        """Sunspec modbus client."""

        _LOGGER.error("New SunspecApi Client")
        self._adapter = adapter
        self._hass = hass
        self._slave_id = slave_id
        self._client_key = f"{adapter}:{slave_id}"
        self._lock = threading.Lock()
        self._reconnect = False

    def get_client(self, config=None):
        cached = SunSpecApiClient.CLIENT_CACHE.get(self._client_key, None)
        if cached is None or config is not None:
            _LOGGER.error("Not using cached connection")
            cached = self.modbus_connect(config)
            SunSpecApiClient.CLIENT_CACHE[self._client_key] = cached
        if self._reconnect:
            if self.check_port():
                cached.connect()
                self._reconnect = False
        return cached

    def async_get_client(self, config=None):
        return self._hass.async_add_executor_job(self.get_client, config)

    async def async_get_data(self, model_id) -> SunSpecModelWrapper:
        try:
            _LOGGER.error("Get data for model %s", model_id)
            return await self.read(model_id)
        except SunSpecModbusClientTimeout as timeout_error:
            _LOGGER.warning("Async get data timeout")
            raise ConnectionTimeoutError() from timeout_error
        except SunSpecModbusClientException as connect_error:
            _LOGGER.warning("Async get data connect_error")
            raise ConnectionError() from connect_error

    async def read(self, model_id) -> SunSpecModelWrapper:
        return await self._hass.async_add_executor_job(self.read_model, model_id)

    async def async_get_device_info(self) -> SunSpecModelWrapper:
        return await self.read(1)

    async def async_get_models(self, config=None) -> list:
        _LOGGER.error("Fetching models")
        client = await self.async_get_client(config)
        model_ids = sorted(list(filter(lambda m: type(m) is int, client.models.keys())))
        return model_ids

    def reconnect_next(self):
        self._reconnect = True

    def close(self):
        client = self.get_client()
        client.close()

    def check_port(self) -> bool:
        """Check if the communication port (adapter) is available."""
        with self._lock:
            _LOGGER.error(
                f"Check_Port: attempting to open adapter {self._adapter} with a 5s timeout."
            )
            try:
                ser = serial.Serial(
                    port=self._adapter,  # Changed from _port to _adapter
                    baudrate=9600,
                    timeout=5
                )
                if ser.is_open:
                    ser.close()
                    _LOGGER.error(
                        f"Check_Port (SUCCESS): adapter {self._adapter} is open and available."
                    )
                    return True
                else:
                    _LOGGER.error(
                        f"Check_Port (ERROR): adapter {self._adapter} is not open."
                    )
            except serial.SerialException as e:
                _LOGGER.error(
                    f"Check_Port (ERROR): failed to open adapter {self._adapter} - error: {e}"
                )
            return False

    def modbus_connect(self, config=None):
        use_config = SimpleNamespace(
            **(
                config
                or {"adapter": self._adapter, "slave_id": self._slave_id}
            )
        )
        _LOGGER.error(
            f"Client connect to Adapter {use_config.adapter} slave id {use_config.slave_id} using timeout {TIMEOUT}"
        )

        client = modbus_client.SunSpecModbusClientDeviceRTU(slave_id=use_config.slave_id, name=use_config.adapter)
        
        if self.check_port():
            _LOGGER.error("Inverter ready for Modbus Serial connection")
            try:
                with self._lock:
                    client.connect()
                if not client.is_connected():
                    raise ConnectionError(
                        f"Failed to connect to {self._adapter} slave id {self._slave_id}"
                    )
                _LOGGER.error("Client connected, perform initial scan")
                client.scan(
                    connect=False, progress=progress, full_model_read=False, delay=0.5
                )
                return client
            except ModbusClientError as e:
                _LOGGER.error(f"Modbus client error during scan: {e}")
                raise ConnectionError(
                    f"Failed to connect to {use_config._adapter} slave id {use_config.slave_id}"
                )
        
        else:
            _LOGGER.error("Inverter not ready for Modbus Serial connection")
            raise ConnectionError(f"Inverter not active on {self._adapter}")

    def read_model(self, model_id) -> dict:
        client = self.get_client()
        models = client.models[model_id]
        for model in models:
            time.sleep(0.6)
            model.read()

        return SunSpecModelWrapper(models)
