from typing.io import TextIO

from remote import Remote, JoyAxis, JoyButton, JOY, main, KEY
from remote.utils import translate, int_to_bytes

import remote
import argparse
import time
import os

MIN_MOVE_SPEED = -100
MAX_MOVE_SPEED = 100
KEYBOARD_MOVE_SPEED = MAX_MOVE_SPEED // 2
BAUD_RATE = 9600
MIN_CHANNEL = 1
MAX_CHANNEL = 100
DEFAULT_CHANNEL = MIN_CHANNEL
MIN_JOYSTICK_VALUE = -1.0
MAX_JOYSTICK_VALUE = 1.0
JOYSTICK_CUTOFF = 0.1

COMMAND = 'C'
COMMAND_MOVE = 'M'
COMMAND_AT = 'AT'
COMMAND_ALL_STOP = 'S'
COMMAND_CALIBRATE = 'G'
COMMAND_PITCH_OFFSET = 'P'
COMMAND_ROLL_OFFSET = 'R'
COMMAND_DOWN_OFFSET = 'D'
COMMAND_UP_OFFSET = 'U'
COMMAND_RESET_OFFSET = 'X'
COMMAND_END = '\n'

DEFAULT_ENCODING = 'utf8'
LOG_DIR = 'logs'
DATA_FILENAME = 'data_log'
DATA_FILE_EXTENSION = '.csv'

SERIAL_PORT = None


def get_args():
    parser = argparse.ArgumentParser()
    # parser.add_argument('-k', '--keyboard', action='store_true', default=False, help='Use a keyboard instead of '
    #                                                                                 'gamepad controls.')
    parser.add_argument('-s', '--move-speed', type=int, default=KEYBOARD_MOVE_SPEED, help=f'Speed to move at '
                                                                                          f'with keyboard controls '
                                                                                          f'(min. speed is '
                                                                                          f'{MIN_MOVE_SPEED}, max. '
                                                                                          f'speed is {MAX_MOVE_SPEED})')
    parser.add_argument('-p', '--serial-port', type=str, default=SERIAL_PORT, help='Serial port for the HC-12 module. '
                                                                                   'Default [None] searches for the '
                                                                                   'first available serial port.')
    parser.add_argument('-b', '--baud-rate', type=int, default=BAUD_RATE, help='Baud rate to set the HC-12 to')
    parser.add_argument('-c', '--channel', type=int, default=DEFAULT_CHANNEL, help='Channel to use for HC-12 modules')
    parser.add_argument('-v', '--verbose', action='store_true', default=False, help='Print debug info to console')
    parser.add_argument('-t', '--test', action='store_true', default=False, help='Enable test mode. Emulates serial '
                                                                                 'connection.')
    parser.add_argument('-l', '--log-data', action='store_true', default=False, help='Store debug data sent from the '
                                                                                     'drone to a logfile (named '
                                                                                     f'{DATA_FILENAME}-[time.ctime].log')
    parser.add_argument('-d', '--print-data', '--display-data', action='store_true', default=False,
                        help='Print received raw data to console.')

    return parser.parse_args()


class DroneRemote(Remote):
    _ROLL = JoyAxis.RIGHT_STICK_X
    _PITCH = JoyAxis.RIGHT_STICK_Y
    _YAW = JoyAxis.LEFT_STICK_X
    _THROTTLE = JoyAxis.LEFT_STICK_Y
    _ALL_STOP = remote.K_SPACE
    _ALL_STOP_JOY = JoyButton(7, 'all_stop')
    _CALIBRATE = JoyButton(0, 'calibrate')
    _PITCH_OFFSET_UP = JoyButton(3, 'pitch_offset_up')
    _PITCH_OFFSET_DOWN = JoyButton(2, 'pitch_offset_down')
    _ROLL_OFFSET_UP = JoyButton(5, 'roll_offset_up')
    _ROLL_OFFSET_DOWN = JoyButton(4, 'roll_offset_down')
    _RESET_OFFSET = JoyButton(6, 'reset_offset')

    def __init__(self, log_data=False, print_data=True, encoding=DEFAULT_ENCODING, *args, **kwargs):
        keycodes = [self._ROLL, self._PITCH, self._YAW, self._THROTTLE,
                    self._ALL_STOP, self._ALL_STOP_JOY, self._CALIBRATE,
                    self._PITCH_OFFSET_UP, self._PITCH_OFFSET_DOWN,
                    self._ROLL_OFFSET_UP, self._ROLL_OFFSET_DOWN, self._RESET_OFFSET]
        # keycodes.extend([JoyButton(i) for i in range(4)])
        super().__init__(keycodes, *args, **kwargs)
        self.log_data = log_data
        self.print_data = print_data
        self.encoding = encoding
        self.__last_msg = None
        self.__log_file = None
        self.__stopped = True

    def __del__(self):
        if self.__is_logfile:
            self.__log_file.close()
        super().__del__()

    @property
    def __is_logfile(self):
        return isinstance(self.__log_file, TextIO)

    def start_loop(self, *args, **kwargs):
        if self.__is_logfile:
            try:
                self.__log_file.close()
            except IOError:
                pass

        if self.log_data:
            if not os.path.exists(LOG_DIR):
                os.makedirs(LOG_DIR)
            logfile = DATA_FILENAME + '-' + time.ctime(time.time()).replace(' ', '-').replace(':', '-') + DATA_FILE_EXTENSION
            filepath = os.path.join(LOG_DIR, logfile)
            self.logger.debug('Logging received data to file: ' + filepath)
            self.__log_file = open(filepath, 'w', encoding=self.encoding)

        super().start_loop(*args, **kwargs)

    def stop_loop(self, *args, **kwargs):
        if self.__is_logfile:
            self.__log_file.close()

        if not self.is_stopped:
            self.send_stop()

        super().stop_loop(*args, **kwargs)

    def _msg_start(self, move=True) -> bytes:
        return bytes(COMMAND + (COMMAND_MOVE if move else ''), encoding=self.encoding)

    @property
    def is_stopped(self):
        return self.__stopped

    @property
    def _msg_end(self) -> bytes:
        return bytes(COMMAND_END, encoding=self.encoding)

    def _command(self, *commands: str or bytes, move=True) -> bytes:
        return self._msg_start(move=move) + b''.join([bytes(c, encoding=self.encoding) if isinstance(c, str) else c
                                                      for c in commands]) + self._msg_end

    @staticmethod
    def _joy_bytes(joystick_val: int, min_from=MIN_JOYSTICK_VALUE, max_from=MAX_JOYSTICK_VALUE,
                   min_to=MIN_MOVE_SPEED, max_to=MAX_MOVE_SPEED) -> bytes:
        return int_to_bytes(int(round(translate(joystick_val, min_from, max_from, min_to, max_to))))

    @staticmethod
    def _cutoff(value: float or int):
        return 0 if abs(value) < JOYSTICK_CUTOFF else value

    def send_stop(self):
        self.write(self._all_stop())
        self.__stopped = True

    def _all_stop(self) -> bytes:
        return self._command(COMMAND_ALL_STOP, move=False)

    def handle_poll_event(self, event: dict) -> bytes or bool:
        throttle = -self._cutoff(event[JOY][self._THROTTLE])
        roll = self._cutoff(event[JOY][self._ROLL])
        pitch = -self._cutoff(event[JOY][self._PITCH])
        yaw = self._cutoff(event[JOY][self._YAW])

        msg = self._command(
            self._joy_bytes(throttle),
            self._joy_bytes(roll),
            self._joy_bytes(pitch),
            self._joy_bytes(yaw)
        )

        if msg != self.__last_msg:
            self.logger.debug(f'Throttle: {throttle} Roll: {roll} Pitch: {pitch} Yaw: {yaw}')
            self.__last_msg = msg
            return msg
        else:
            return False

    def handle_press_event(self, event: dict) -> bytes or bool:
        if event[KEY][self._ALL_STOP] or event[JOY][self._ALL_STOP_JOY]:
            self.logger.debug('All stop pressed.')
            return self._all_stop()
        if event[JOY][self._CALIBRATE]:
            self.logger.debug('Calibration pressed.')
            return self._command(COMMAND_CALIBRATE, move=False)
        if event[JOY][self._PITCH_OFFSET_UP]:
            self.logger.debug('Increasing pitch offset.')
            return self._command(COMMAND_PITCH_OFFSET, COMMAND_UP_OFFSET, move=False)
        if event[JOY][self._PITCH_OFFSET_DOWN]:
            self.logger.debug('Decreasing pitch offset.')
            return self._command(COMMAND_PITCH_OFFSET, COMMAND_DOWN_OFFSET, move=False)
        if event[JOY][self._ROLL_OFFSET_UP]:
            self.logger.debug('Increasing roll offset.')
            return self._command(COMMAND_ROLL_OFFSET, COMMAND_UP_OFFSET, move=False)
        if event[JOY][self._ROLL_OFFSET_DOWN]:
            self.logger.debug('Decreasing roll offset.')
            return self._command(COMMAND_ROLL_OFFSET, COMMAND_DOWN_OFFSET, move=False)
        if event[JOY][self._RESET_OFFSET]:
            self.logger.debug('Resetting offset.')
            return self._command(COMMAND_RESET_OFFSET, move=False)

    def receive_event(self, data: bytes):
        if self.log_data:
            self.__log_file.write(data.decode(self.encoding).replace('\r', ''))


if __name__ == '__main__':
    main(get_args, DroneRemote, SERIAL_PORT, ['log_data', 'print_data'])
