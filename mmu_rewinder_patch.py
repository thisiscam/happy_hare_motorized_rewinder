# A single DC Motor controlled by DRV8833
#
# Copyright (C) 2024 Cambridge Yang <camyang@csail.mit.edu>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

#pylint:disable=protected-access

import contextlib
import asyncio
import threading
import configfile
import struct


@contextlib.contextmanager
def patch_method(obj, method_name, new_method):
  """Handler to temporarily match a method of an object, and restore it afterwards
  It takes care of exception handling and restoring the original method
  """
  old_method = getattr(obj, method_name)
  setattr(obj, method_name,
          lambda *args, **kwargs: new_method(old_method, *args, **kwargs))
  try:
    yield
  finally:
    setattr(obj, method_name, old_method)


_mmu_trace_filament_move = None

bleak = None


class MmuRewinderPatch:
  """Patch the MMU to control the rewind motors"""

  def __init__(self, config):
    self.printer = config.get_printer()
    self.mmu = self.printer.lookup_object('mmu')
    self.gcode = self.printer.lookup_object('gcode')
    self.printer.register_event_handler('klippy:connect', self.handle_connect)
    self.printer.register_event_handler("klippy:disconnect",
                                        self.handle_disconnect)
    global bleak
    try:
      import bleak
    except ImportError:
      raise config.error("MmuRewinderPatch requires bleak module")

    self.ble_loop = None
    self.ble_addresses = config.get('ble_addresses', "").split(',')
    self.ble_devices = []
    self.ble_timeout = config.getfloat('ble_timeout', 10.0)

    self._patch_mmu()

  def _patch_mmu(self):
    global _mmu_trace_filament_move
    _mmu_trace_filament_move = self.mmu._trace_filament_move
    self.mmu._trace_filament_move = self._trace_filament_move

  def _setup_ble_thread(self):
    """Setup a new thread for the BLE event loop. All BLE commands will be run on this thread."""
    if self.ble_loop is None:
      self.ble_loop = asyncio.new_event_loop()

      def _init(loop):
        asyncio.set_event_loop(loop)
        loop.run_forever()

      threading.Thread(target=_init, args=(self.ble_loop,)).start()

  def handle_connect(self):
    self._setup_ble_thread()
    asyncio.run(self._connect_bles())

  def handle_ble_disconnect(self, client, *args, **kwargs):
    self.mmu._mmu_pause(f"BLE device {client.address} disconnected")

  async def _connect_bles(self):

    for ble_address in self.ble_addresses:
      client = bleak.BleakClient(
          ble_address,
          timeout=self.ble_timeout,
          disconnected_callback=self.handle_ble_disconnect)
      try:
        await client.connect()
      except bleak.exc.BleakError as e:
        raise configfile.error(
            f"Unable to connect to BLE device at {ble_address}: {e}")
      self.mmu._log_info(f"Connected to BLE device at {ble_address}")
      self.ble_devices.append(client)

  async def _disconnect_bles(self):
    for client in self.ble_devices:
      if client.is_connected:
        await client.disconnect()

  def handle_disconnect(self):
    self.run_on_ble_thread(self._disconnect_bles())

  def run_on_ble_thread(self, coro):
    """Run a coroutine on the BLE thread"""
    assert self.ble_loop is not None, "BLE thread not running"
    return asyncio.run_coroutine_threadsafe(coro, self.ble_loop)

  def _trace_filament_move(self, trace_str, *args, **kwargs):
    if trace_str == "Course unloading move from bowden":
      self.rewind_control("rewind_fast")
      ret = _mmu_trace_filament_move(trace_str, *args, **kwargs)
      self.rewind_control("stop")
    elif trace_str in (
        "Reverse homing to extruder sensor",
        "Reverse homing to toolhead sensor", "Bowden pre-unload test",
        "Reverse homing to gate sensor", "Unloading extruder", "Final parking",
        "Bowden pre-unload test"):  # TODO: handle case without gate sensor
      self.rewind_control("rewind_slow")
      ret = _mmu_trace_filament_move(trace_str, *args, **kwargs)
      self.rewind_control("stop")
      if trace_str == "Final parking":
        self.mmu._movequeues_wait_moves()
    elif trace_str == "Course loading move into bowden":
      if self.mmu.gate_selected >= 0 and self.mmu.gate_status[
          self.mmu.gate_selected] != self.mmu.GATE_AVAILABLE_FROM_BUFFER:
        self.rewind_control("load_slow")
        ret = _mmu_trace_filament_move(trace_str, *args, **kwargs)
        self.rewind_control("stop")
      else:
        self.rewind_control("load_fast")
        ret = _mmu_trace_filament_move(trace_str, *args, **kwargs)
        self.rewind_control("stop")
    else:
      ret = _mmu_trace_filament_move(trace_str, *args, **kwargs)
    return ret

  def rewind_control(self, mode):
    """Control the rewind motor"""
    rewinder_idx = self.mmu.gate_selected
    if rewinder_idx >= 0:
      rewinder = self.printer.lookup_object(
          f"drv8833_motor rewinder{rewinder_idx}")
      self._rewind_control_send(rewinder, mode)

  def _rewind_control_send(self, rewinder, mode):
    """Send the rewind control command to the BLE device"""
    decay_mode = 0  # default slow decay mode
    if mode == "stop":
      speed = 0
      decay_mode = 1
    elif mode == "brake":
      speed = 0
    elif mode == "rewind_fast":
      speed = -rewinder.rewind_fast_speed
    elif mode == "rewind_slow":
      speed = -rewinder.rewind_slow_speed
    elif mode == "load_fast":
      speed = rewinder.load_fast_speed
    elif mode == "load_slow":
      speed = rewinder.load_slow_speed
    else:
      raise ValueError(f"Invalid rewind mode {mode}")

    client = self.ble_devices[rewinder.ble_device_idx]
    self.run_on_ble_thread(
        send_command(client, rewinder.pin_a, rewinder.pin_b, decay_mode, speed,
                     rewinder.pwm_freq, rewinder.startup_speed,
                     rewinder.startup_duration, rewinder.soft_start_duration,
                     False))


UART_SERVICE_UUID = "fb1e4001-54ae-4a28-9f74-dfccb248601d"
UART_RX_CHAR_UUID = "fb1e4002-54ae-4a28-9f74-dfccb248601d"
UART_TX_CHAR_UUID = "fb1e4003-54ae-4a28-9f74-dfccb248601d"


async def send_command(client,
                       pin_a,
                       pin_b,
                       decay_mode=0,
                       speed=0,
                       pwm_freq=5000,
                       startup_speed=0,
                       startup_duration=0,
                       soft_start_duration=0,
                       response: bool = False):
  data = struct.pack('<BBBBbbIff', 0xEE, pin_a, pin_b, decay_mode, speed,
                     startup_speed, pwm_freq, startup_duration,
                     soft_start_duration)
  return await client.write_gatt_char(UART_RX_CHAR_UUID, bytearray(data),
                                      response)


def load_config(config):
  return MmuRewinderPatch(config)
