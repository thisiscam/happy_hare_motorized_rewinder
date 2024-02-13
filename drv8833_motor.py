# A single DC Motor controlled by DRV8833
#
# Copyright (C) 2024 Cambridge Yang <camyang@csail.mit.edu>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import math

MOTOR_MIN_TIME = 0.100

class Drv8833Motor:
    def __init__(self, config, default_shutdown_speed=0.):
        self.printer = config.get_printer()
        self.last_speed_a = 0.
        self.last_speed_b = 0.
        self.last_decay_mode = 'fast'
        self.last_motor_time = 0.
        # Read config
        self.max_power = config.getfloat('max_power', 1., above=0., maxval=1.)
        self.kick_start_speed = config.getfloat('kick_start_speed', self.max_power, above=0., maxval=1.)
        self.kick_start_time = config.getfloat('kick_start_time', 0.1, minval=0.)
        cycle_time = config.getfloat('cycle_time', 0.010, above=0.)
        hardware_pwm = config.getboolean('hardware_pwm', False)
        shutdown_speed = config.getfloat(
            'shutdown_speed', default_shutdown_speed, minval=0., maxval=1.)
        # Setup pwm object
        ppins = self.printer.lookup_object('pins')
        self.mcu_motor_a = ppins.setup_pin('pwm', config.get('pin_a'))
        self.mcu_motor_b = ppins.setup_pin('pwm', config.get('pin_b'))
        self.mcu_motor_a.setup_max_duration(0.)
        self.mcu_motor_b.setup_max_duration(0.)
        self.mcu_motor_a.setup_cycle_time(cycle_time, hardware_pwm)
        self.mcu_motor_b.setup_cycle_time(cycle_time, hardware_pwm)
        shutdown_power = max(0., min(self.max_power, shutdown_speed))
        self.mcu_motor_a.setup_start_value(0., shutdown_power)
        self.mcu_motor_b.setup_start_value(0., shutdown_power)

        # Register callbacks
        self.printer.register_event_handler("gcode:request_restart",
                                            self._handle_request_restart)
        self.mcu.register_config_callback(self.build_config)

        gcode = self.printer.lookup_object("gcode")
        self.motor_name = config.get_name().split()[-1]
        gcode.register_mux_command("SET_MOTOR_VELOCITY", "MOTOR",
                                   self.motor_name,
                                   self.cmd_SET_MOTOR_VELOCITY,
                                   desc=self.cmd_SET_MOTOR_VELOCITY_help)

    @property
    def mcu(self):
        return self.mcu_motor_a.get_mcu()
    
    @property
    def _sx1509(self):
        return self.mcu_motor_a._sx1509
    
    @property
    def _i2c(self):
        return self._sx1509._i2c

    def build_config(self):
        if not hasattr(self._i2c, "i2c_write_batch_cmd"):
          self._i2c.i2c_write_batch_cmd = self.mcu.lookup_command(
              "i2c_write_batch oid=%c chunks=%*s data=%*s", cq=self._i2c.cmd_queue)

    def set_speed(self, print_time, velocity, decay_mode):
        velocity = math.copysign(max(0., min(self.max_power, abs(velocity) * self.max_power)), velocity)
        if velocity == self.last_motor_velocity and decay_mode == self.last_decay_mode:
            return
        speed = abs(velocity)
        if decay_mode == 'fast':
            speed_a, speed_b = speed, 0
            ks_a, ks_b = self.kick_start_speed, 0
        else:
            speed_a, speed_b = 1, 1 - speed
            ks_a, ks_b = 1, 1 - self.kick_start_speed
        if velocity < 0:
            speed_a, speed_b = speed_b, speed_a
            ks_a, ks_b = ks_b, ks_a

        print_time = max(self.last_motor_time + MOTOR_MIN_TIME, print_time)
        if (speed and self.kick_start_time and abs(velocity - self.last_motor_velocity) > .5):
            self.set_ab_speed(print_time, ks_a, ks_b)
            print_time += self.kick_start_time
        self.set_ab_speed(print_time, speed_a, speed_b)
        self.last_motor_time = print_time
        self.last_decay_mode = decay_mode
    
    def ab_to_velocity(self, a, b):
        if b == 0:
            return a
        elif a == 0:
            return -b
        elif b == 1:
            return a - 1
        elif a == 1:
            return 1 - b
        else:
            return None
    
    @property
    def last_motor_velocity(self):
        return self.ab_to_velocity(self.last_speed_a, self.last_speed_b)
    
    def set_ab_speed(self, print_time, a, b):
        self._set_pwm(self.mcu_motor_b, b)
        self._set_pwm(self.mcu_motor_a, a)
        self._send_registers(print_time)
        self.last_speed_a = a
        self.last_speed_b = b

    def _set_pwm(self, motor, value):
        self._sx1509.set_register(motor._i_on_reg, ~int(255 * value)
                                  if not motor._invert
                                  else int(255 * value) & 0xFF)
    
    def _send_registers(self, print_time):
        regs = [m._i_on_reg for m in (self.mcu_motor_a, self.mcu_motor_b)]
        data = []
        for reg in regs:
          data += [reg & 0xFF]
          if reg in self._sx1509.reg_dict:
              # Word
              data += [(self._sx1509.reg_dict[reg] >> 8) & 0xFF,
                       self._sx1509.reg_dict[reg] & 0xFF]
          elif reg in self._sx1509.reg_i_on_dict:
              # Byte
              data += [self._sx1509.reg_i_on_dict[reg] & 0xFF]
        clock = self.mcu.print_time_to_clock(print_time)
        self._i2c.i2c_write_batch_cmd.send([self._i2c.oid, [2, 2], data], 
                minclock=self._sx1509._last_clock, reqclock=clock)
        self._sx1509._last_clock = clock


    cmd_SET_MOTOR_VELOCITY_help = "Sets the velocity of a DC motor"
    def cmd_SET_MOTOR_VELOCITY(self, gcmd):
        velocity = gcmd.get_float('VELOCITY', 0.)
        decay_mode = gcmd.get('DECAY_MODE', default='fast')
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.register_lookahead_callback((lambda pt:
                                              self.set_speed(pt, velocity, decay_mode)))

    def _handle_request_restart(self, print_time):
        self.set_speed(print_time, 0., 'fast')

    def get_status(self, eventtime):
        return {
            'velocity': self.last_motor_velocity,
            'decay_mode': self.last_decay_mode,
        }

def load_config_prefix(config):
    return Drv8833Motor(config)
