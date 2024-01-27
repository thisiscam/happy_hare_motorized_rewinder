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
        self.last_motor_velocity = 0.
        self.last_decay_mode = 'fast'
        self.last_motor_time = 0.
        # Read config
        self.max_power = config.getfloat('max_power', 1., above=0., maxval=1.)
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

        gcode = self.printer.lookup_object("gcode")
        self.motor_name = config.get_name().split()[-1]
        gcode.register_mux_command("SET_MOTOR_VELOCITY", "MOTOR",
                                   self.motor_name,
                                   self.cmd_SET_MOTOR_VELOCITY,
                                   desc=self.cmd_SET_MOTOR_VELOCITY_help)

    def get_mcu(self):
        return self.mcu_motor_a.get_mcu()

    def set_speed(self, print_time, velocity, decay_mode):
        velocity = math.copysign(max(0., min(self.max_power, abs(velocity) * self.max_power)), velocity)
        if velocity == self.last_motor_velocity:
            return
        speed = abs(velocity)
        if decay_mode == 'fast':
            speed_a, speed_b = speed, 0
            max_a, max_b = max_power, 0
        else:
            speed_a, speed_b = 1, 1 - speed
            max_a, max_b = 1, 0
        if velocity < 0:
            speed_a, speed_b = speed_b, speed_a
            max_a, max_b = max_b, max_a

        print_time = max(self.last_motor_time + MOTOR_MIN_TIME, print_time)
        if (speed and speed < self.max_power and self.kick_start_time
            and (not self.last_motor_velocity or abs(velocity - self.last_motor_velocity) > .5)):
            # Run motor at full speed for specified kick_start_time
            self.mcu_motor_a.set_pwm(print_time, max_a)
            self.mcu_motor_b.set_pwm(print_time, max_b)
            print_time += self.kick_start_time
        self.mcu_motor_a.set_pwm(print_time, speed_a)
        self.mcu_motor_b.set_pwm(print_time, speed_b)
        self.last_motor_time = print_time
        self.last_motor_velocity = velocity
        self.last_decay_mode = decay_mode
       

    cmd_SET_MOTOR_VELOCITY_help = "Sets the velocity of a DC motor"
    def cmd_SET_MOTOR_VELOCITY(self, gcmd):
        velocity = gcmd.get_float('VELOCITY', 0.)
        decay_mode = gcmd.get('DECAY_MODE', default='fast')
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.register_lookahead_callback((lambda pt:
                                              self.set_speed(pt, velocity, decay_mode)))

    def _handle_request_restart(self, print_time):
        logging.info("test test xxxxx")
        self.set_speed(print_time, 0., 'fast')

    def get_status(self, eventtime):
        return {
            'velocity': self.last_motor_velocity,
            'decay_mode': self.last_decay_mode,
        }

def load_config_prefix(config):
    return Drv8833Motor(config)
