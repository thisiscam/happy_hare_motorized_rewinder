# A single DC Motor controlled by DRV8833
#
# Copyright (C) 2024 Cambridge Yang <camyang@csail.mit.edu>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import math

MOTOR_MIN_TIME = 0.100


class Drv8833Motor:

  def __init__(self, config):
    self.printer = config.get_printer()
    self.pin_a = config.getint('pin_a')
    self.pin_b = config.getint('pin_b')
    self.pwm_freq = config.getint('pwm_freq', 5000)
    self.rewind_fast_speed = config.getfloat('rewind_fast_speed', 1.0)
    self.rewind_slow_speed = config.getfloat('rewind_slow_speed', 0.4)
    self.load_fast_speed = config.getfloat('load_fast_speed', 0.4)
    self.load_slow_speed = config.getfloat('load_slow_speed', 0.2)
    self.startup_speed = config.getfloat('startup_speed', 1.0)
    self.startup_duration = config.getfloat('startup_duration', 0.1)
    self.soft_start_duration = config.getfloat('soft_start_duration', 0.)
    self.ble_device_idx = config.getint('ble_device_idx', 0)

    rewinder_idx = config.get_name().split()[1].split('rewinder')
    self.gcode = self.printer.lookup_object('gcode')
    self.gcode.register_mux_command('REWIND_CONTROL',
                                    'ID',
                                    rewinder_idx,
                                    self.cmd_REWIND_CONTROL,
                                    desc=self.cmd_REWIND_CONTROL.__doc__)
    self.mmu_patch = self.printer.lookup_object('mmu_rewinder_patch')

  def cmd_REWIND_CONTROL(self, gcmd):
    """Control the rewind motor"""
    mode = gcmd.get('MODE')
    self.mmu_patch._rewind_control_send(self, mode)


def load_config_prefix(config):
  return Drv8833Motor(config)
