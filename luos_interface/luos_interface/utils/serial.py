import platform
import glob

def get_available_ports():
    """ Tries to find the available serial ports on your system. """
    if platform.system() == 'Darwin':
        return glob.glob('/dev/tty.usb*')

    elif platform.system() == 'Linux':
        return glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyAMA*')

    elif sys.platform.lower() == 'cygwin':
        return glob.glob('/dev/com*')

    elif platform.system() == 'Windows':
        import winreg
        import itertools

        ports = []
        path = 'HARDWARE\\DEVICEMAP\\SERIALCOMM'
        key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, path)

        for i in itertools.count():
            try:
                ports.append(str(winreg.EnumValue(key, i)[1]))
            except WindowsError:
                return ports
    else:
        raise EnvironmentError('{} is an unsupported platform, cannot find serial ports!'.format(platform.system()))
    return []

