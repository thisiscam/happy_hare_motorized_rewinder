# A single DC Motor controlled by DRV8833
#
# Copyright (C) 2024 Cambridge Yang <camyang@csail.mit.edu>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import contextlib


@contextlib.contextmanager
def patch_method(obj, method_name, new_method):
  """Handler to temporarily match a method of an object, and restore it afterwards
  It takes care of exception handling and restoring the original method
  """
  old_method = getattr(obj, method_name)
  setattr(obj, method_name, new_method)
  try:
    yield
  finally:
    setattr(obj, method_name, old_method)


_mmu_unload_bowden = None
_mmu_unload_gate = None
_mmu_load_bowden = None


class MmuRewinderPatch:

  def __init__(self, config):
    self.printer = config.get_printer()
    self.mmu = self.printer.lookup_object('mmu')
    self.gcode = self.printer.lookup_object('gcode')
    self._patch_mmu()

  def _patch_mmu(self):
    global _mmu_unload_bowden, _mmu_unload_gate, _mmu_load_bowden
    _mmu_unload_bowden = self.mmu._unload_bowden
    _mmu_unload_gate = self.mmu._unload_gate
    _mmu_load_bowden = self.mmu._load_bowden
    self.mmu._unload_bowden = self._unload_bowden
    self.mmu._unload_gate = self._unload_gate
    self.mmu._load_bowden = self._load_bowden

  def _unload_bowden(self, *args, **kwargs):

    def servo_down():
      ret = self.mmu._servo_down()
      self.rewind_control("rewind_fast")
      return ret

    with patch_method(self.mmu, '_servo_down', servo_down):
      ret = _mmu_unload_bowden(*args, **kwargs)
      self.rewind_control("stop")

    return ret

  def _unload_gate(self, *args, **kwargs):

    def servo_down():
      ret = self.mmu._servo_down()
      self.rewind_control("rewind_slow")
      return ret

    with patch_method(self.mmu, '_servo_down', servo_down):
      ret = _mmu_unload_gate(*args, **kwargs)
      self.rewind_control("stop")

    return ret

  def _load_bowden(self, *args, **kwargs):

    def servo_down():
      ret = self.mmu._servo_down()
      self.rewind_control("fast_load")
      return ret

    with patch_method(self.mmu, '_servo_down', servo_down):
      ret = _mmu_load_bowden(*args, **kwargs)
      self.rewind_control("stop")

    return ret

  def rewind_control(self, mode):
    """Control the rewind motor"""
    rewinder_idx = self.mmu.gate_selected
    if type(rewinder_idx) is int:
      self.gcode.run_script_from_command("REWINDER_CONTROL ID=%d MODE=%s" %
                                         (rewinder_idx, mode))


def load_config(config):
  return MmuRewinderPatch(config)
