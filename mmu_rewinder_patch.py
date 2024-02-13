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
  setattr(obj, method_name,
          lambda *args, **kwargs: new_method(old_method, *args, **kwargs))
  try:
    yield
  finally:
    setattr(obj, method_name, old_method)


_mmu_unload_bowden = None
_mmu_unload_gate = None
_mmu_load_bowden = None
_mmu_trace_filament_move = None


class MmuRewinderPatch:

  def __init__(self, config):
    self.printer = config.get_printer()
    self.mmu = self.printer.lookup_object('mmu')
    self.gcode = self.printer.lookup_object('gcode')
    self._patch_mmu()

  def _patch_mmu(self):
    global _mmu_trace_filament_move
    _mmu_trace_filament_move = self.mmu._trace_filament_move
    self.mmu._trace_filament_move = self._trace_filament_move

  def _trace_filament_move(self, trace_str, *args, **kwargs):
    if trace_str == "Course unloading move from bowden":
      self.rewind_control("rewind_fast")
      ret = _mmu_trace_filament_move(trace_str, *args, **kwargs)
      self.rewind_control("stop")
    elif trace_str in ("Reverse homing to extruder sensor", "Reverse homing to toolhead sensor", "Bowden pre-unload test", "Reverse homing to gate sensor", "Unloading extruder", "Final parking", "Bowden pre-unload test"):  # TODO: handle case without gate sensor
      self.rewind_control("rewind_slow")
      ret = _mmu_trace_filament_move(trace_str, *args, **kwargs)
      self.rewind_control("stop")
      if trace_str == "Final parking":
          self.mmu._movequeues_wait_moves()
    elif trace_str == "Course loading move into bowden":
      if self.mmu.gate_selected >= 0 and self.mmu.gate_status[self.mmu.gate_selected] != self.mmu.GATE_AVAILABLE_FROM_BUFFER:
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
    if type(rewinder_idx) is int:
      self.gcode.run_script_from_command("REWINDER_CONTROL ID=%d MODE=%s" %
                                         (rewinder_idx, mode))


def load_config(config):
  return MmuRewinderPatch(config)
