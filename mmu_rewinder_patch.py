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


class MMuRewinderPatch:

  def __init__(self, config):
    self.mmu = self.printer.lookup_object('mmu')
    self.gcode = self.printer.lookup_object('gcode')
    self._patch_mmu()

  def _patch_mmu(self):
    self.mmu._unload_bowden = self._unload_bowden
    self.mmu._unload_gate = self._unload_gate
    self.mmu._load_bowden = self._load_bowden

  def _unload_bowden(self, length):

    def servo_down():
      ret = self.mmu._servo_down()
      self.rewind_control("rewind_fast")
      return ret

    with patch_method(self.mmu, '_servo_down', servo_down):
      ret = self.mmu._unload_bowden(length)
      self.rewind_control("stop")

    return ret

  def _unload_gate(self, length):

    def servo_down():
      ret = self.mmu._servo_down()
      self.rewind_control("rewind_slow")
      return ret

    with patch_method(self.mmu, '_servo_down', servo_down):
      ret = self.mmu._unload_gate(length)
      self.rewind_control("stop")

    return ret

  def _load_bowden(self, length):

    def servo_down():
      ret = self.mmu._servo_down()
      self.rewind_control("fast_load")
      return ret

    with patch_method(self.mmu, '_servo_down', servo_down):
      ret = self.mmu._load_bowden(length)
      self.rewind_control("stop")

    return ret

  def rewind_control(self, mode):
    """Control the rewind motor"""
    self.gcode.run_script_from_command("REWINDER_CONTROL VARIABLE=%s" % mode)


def load_config(config):
  return MMuRewinderPatch(config)