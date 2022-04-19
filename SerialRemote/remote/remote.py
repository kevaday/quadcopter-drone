from remote.utils import serial_ports
from typing import Callable
from enum import Enum
from threading import Thread, Event
from pygame import *

import serial
import logging
import pygame
import time

_LOOP_TIMEOUT = 0.1

KEY = 0
JOY = 1


"""
class Control(object):
    def __init__(self, func: Callable, *key_codes, **func_kwargs):
        self.key_codes = list(_traverse(key_codes, tree_types=(list, tuple, Control)))
        self._func = func
        self._func_kwargs = func_kwargs

    def __repr__(self):
        return f'{self.__class__.__name__}(key_codes={self.key_codes}, function={self._func})'

    def call_func(self, key_event: KeyEvent) -> bytes:
        # self._func_args.extend([ev_type, ev_state])
        [ctrl.call_func(key_event) for ctrl in self if key_event.key_code in ctrl.key_codes]
        return
"""


class JoyAxis(Enum):
    LEFT_STICK_X = 0
    LEFT_STICK_Y = 1
    LEFT_TRIGGER = 2
    RIGHT_STICK_X = 3
    RIGHT_STICK_Y = 4
    RIGHT_TRIGGER = 5


class JoyButton(object):
    def __init__(self, value: int, name: str = None):
        self.value = value
        self.name = name

    def __repr__(self):
        return f'JoyButton({self.value}, {self.name})'


class Remote(serial.Serial):
    def __init__(self, key_codes: list, poll_keys=True, poll_timeout=_LOOP_TIMEOUT, receive_timeout=_LOOP_TIMEOUT,
                 logger=None, test_mode=False, *args, **kwargs):
        self.key_codes = key_codes
        self.poll_keys = poll_keys
        self.poll_timeout = poll_timeout
        self.receive_timeout = receive_timeout
        self.test_mode = test_mode

        self.logger = logger if logger else logging.getLogger(__name__)

        # self.logger.debug(f'[remote/inputs] Using gamepad: {use_gamepad}')
        self.logger.debug(f'[remote/inputs] Registered {len(key_codes)} controls:')
        self.logger.debug(f'\t{key_codes}')

        self.__loop_thread = None
        self.__receive_thread = None
        self.__stopped = Event()

        self.__joy = None
        self.__key_error = False
        self.__joy_error = False

        super().__init__(*args, **kwargs)

    @property
    def stopped(self):
        return self.__stopped.is_set()

    def start_loop(self, block=False):
        self.__joy = None
        self.__key_error = False
        self.__joy_error = False

        try:
            pygame.joystick.init()
            self.__joy = pygame.joystick.Joystick(0)
            self.__joy.init()
        except pygame.error:
            self.logger.warning('No joystick/gamepad detected. Please connect one to be able to use it.')

        self.__loop_thread = Thread(target=self.__poll_loop, daemon=True)
        self.__loop_thread.start()
        self.__receive_thread = Thread(target=self.__receive_loop, daemon=True)
        self.__receive_thread.start()

        self.logger.debug('[remote/threading] Started main loop thread.')
        if block:
            self.logger.debug('[remote/threading] Joining loop thread to main thread.')
            self.__loop_thread.join()
            self.__receive_thread.join()

    def stop_loop(self, block=True):
        self.logger.debug('[remote/threading] Stopping loop thread...')
        self.__stopped.set()
        if block:
            self.logger.debug('[remote/threading] Joining loop thread to main thread...')
            self.__loop_thread.join()
            self.__receive_thread.join()
        self.logger.debug('[remote/threading] Loop stopped successfully.')

    def _iter_keys(self):
        return iter(filter(lambda k: isinstance(k, int), self.key_codes))

    def _iter_joys(self):
        return iter(filter(lambda k: isinstance(k, JoyButton), self.key_codes))

    def __poll_loop(self):
        pygame.display.init()
        pygame.display.set_mode((500, 200))
        event_thread = Thread(target=self.__event_loop, daemon=True)
        event_thread.start()

        while not self.stopped:
            pygame.event.pump()
            state = {KEY: {}, JOY: {}}

            for key_code in self.key_codes:
                if isinstance(key_code, JoyAxis):  # Input from joystick axes
                    try:
                        state[JOY][key_code] = self.__joy.get_axis(key_code.value)
                    except (pygame.error, KeyError, AttributeError) as e:
                        if not self.__joy_error:
                            self.logger.error('Error getting input from joystick axis: {}'.format(e))
                            self.__joy_error = True

            self.__handle_event(state, poll=True)
            time.sleep(self.poll_timeout)

        event_thread.join()

    def __event_loop(self):
        while not self.stopped:
            state = {KEY: {k: 0 for k in self._iter_keys()}, JOY: {k: 0 for k in self._iter_joys()}}
            for event in pygame.event.get():

                if event.type == pygame.JOYBUTTONDOWN:
                    for joy_button in self._iter_joys():
                        try:
                            state[JOY][joy_button] = self.__joy.get_button(joy_button.value)
                        except (pygame.error, KeyError, AttributeError) as e:
                            if not self.__joy_error:
                                self.logger.error('Error getting input from joystick button: {}'.format(e))
                                self.__joy_error = True

                elif event.type == pygame.KEYDOWN:
                    for key in self._iter_keys():
                        try:
                            state[KEY][key] = pygame.key.get_pressed()[key]
                        except (pygame.error, KeyError) as e:
                            if not self.__key_error:
                                self.logger.error('Error getting input from keyboard: {}'.format(e))
                                self.__key_error = True

            self.__handle_event(state, poll=False)
            time.sleep(self.poll_timeout)

    def __receive_loop(self):
        while not self.stopped:
            data = b''
            while self.in_waiting:
                data += self.read()

            if data: self.receive_event(data)
            time.sleep(self.receive_timeout)

    def __handle_event(self, event: dict, poll=True):
        msg = self.handle_poll_event(event) if poll else self.handle_press_event(event)

        try:
            if msg: self.write(msg)
        except serial.SerialException as e:
            self.logger.error(f'Failed to send serial data: {e}')

    def handle_poll_event(self, event: dict) -> bytes or bool:
        """Implementation for handling a poll event"""
        ...

    def handle_press_event(self, event: dict) -> bytes or bool:
        """User implementation for handling a key press event"""
        ...

    def receive_event(self, data: bytes):
        """This is called when data is received through serial"""
        ...

    def open(self):
        self.logger.debug(f'[remote/serial] Opening serial port (fake serial: {self.test_mode})')
        if not self.test_mode: super().open()
        else: self.is_open = True

    def write(self, data: bytes) -> int:
        self.logger.debug(f'[remote/serial] Sending data through serial: {data}')
        if not self.test_mode: return super().write(data)
        else: return len(data)

    def read(self, size: int = 1) -> bytes or None:
        # self.logger.debug(f'[remote/serial] Reading {size} bytes from serial.')
        if not self.test_mode: return super().read(size)


def main(get_args: Callable, remote_class, serial_port: str = None, pass_args: list = None):
    if not pass_args: pass_args = []
    args = get_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO)
    logger = logging.getLogger(__name__)

    if not serial_port and not args.test:
        ports = serial_ports()
        if len(ports) == 0: raise RuntimeError('Could not find a serial port to connect to. '
                                               'Please connect the serial transceiver.')
        args.serial_port = ports[0]

    logger.debug(f'Got args:\n{args}')

    remote = remote_class(logger=logger, baudrate=args.baud_rate, port=args.serial_port, test_mode=args.test,
                          **{arg: getattr(args, arg) for arg in pass_args})
    if not remote.is_open: remote.open()
    time.sleep(.2)  # Wait for serial port & transceiver to initialize

    remote.start_loop(block=False)
    logger.info('Ready!')

    try:
        while True: time.sleep(1)
    except KeyboardInterrupt:
        remote.stop_loop()
