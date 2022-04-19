import serial
import glob
import sys


def translate(value, left_min, left_max, right_min, right_max):
    if value < left_min: return right_min
    if value > left_max: return right_max
    # Figure out how 'wide' each range is
    left_span = left_max - left_min
    right_span = right_max - right_min

    # Convert the left range into a 0-1 range (float)
    value_scaled = float(value - left_min) / float(left_span)

    # Convert the 0-1 range into a value in the right range.
    return right_min + (value_scaled * right_span)


def int_to_bytes(integer: int) -> bytes:
    """
    Int-to-bytes conversion for serial communication
    :param integer: integer to convert to bytes (must be in the range(-100, 100) (inclusive)
    :return: bytes (1 byte) representing the integer in the range 0x0-0xC8 (0-200) (inclusive)
    """
    if integer not in range(-100, 101): raise ValueError('Param integer must be in the range(-100, 100) (inclusive)')
    return bytes([integer+100])


def bytes_to_int(byte: bytes) -> int:
    return int.from_bytes(byte, byteorder='big')-100


def serial_ports():
    """ Lists serial port names
        https://stackoverflow.com/questions/12090503/listing-available-com-ports-with-python

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of the serial ports available on the system
    """
    if sys.platform.startswith('win'):
        ports = ['COM%s' % (i + 1) for i in range(256)]
    elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
        # this excludes your current terminal "/dev/tty"
        ports = glob.glob('/dev/tty[A-Za-z]*')
    elif sys.platform.startswith('darwin'):
        ports = glob.glob('/dev/tty.*')
    else:
        raise EnvironmentError('Unsupported platform')

    result = []
    for port in ports:
        try:
            s = serial.Serial(port)
            s.close()
            result.append(port)
        except (OSError, serial.SerialException):
            pass
    return result


def traverse(o, tree_types=(list, tuple)):
    if isinstance(o, tree_types):
        for value in o:
            for subvalue in traverse(value, tree_types):
                yield subvalue
    else:
        yield o
