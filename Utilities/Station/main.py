import XBee
import sys
from time import sleep

class _Getch:
    """
    Get a single character from the keyboard.
    """
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self):
        return self.impl()

class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch

class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()

getch = _Getch()

address = 0x0001
throttle = 1100

def print_legend():
    print("Legend:")
    print("[q = quit] [spacebar = stop] [a = auto] [h = heartbeat]")
    print("[w = increase throttle] [s = decrease throttle]")
    print("[p = modify Kp] [i = modify Ki] [d = modify Kd]")
    print("[1-4 = manually set ESC 1-4 power]")
    

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Error: Serial port must be specified.")
        exit()

    s_port = sys.argv[1]
    xbee = XBee.XBee(s_port)
    print_legend()

    while True:
        p = getch()
        
        if p == 'q':
            exit()
        elif p == '?':
            print_legend()
        elif p == ' ':
            print("stop")
            xbee.SendStr("SA", addr=address)
        elif p == 'a':
            print("auto")
            xbee.SendStr("AT", addr=address)
        elif p == 'h':
            sys.stdout.write("HB...")
            xbee.SendStr("HB", addr=address)
            Msg = xbee.Receive()
            if Msg:
                content = Msg[7:-1].decode('ascii')
                print(content)
        elif p == 'w':
            throttle += 10
            if throttle > 1900: throttle = 1900
            print("throttle+ " + str(throttle))
            xbee.SendStr("TH " + str(throttle), addr=address)
        elif p == 's':
            throttle -= 10
            if throttle < 1100: throttle = 1100
            print("throttle- " + str(throttle))
            xbee.SendStr("TH " + str(throttle), addr=address)
        elif p == '1':
            val = raw_input("esc 1 value: ")
            xbee.SendStr("M1 " + str(val), addr=address)           
        elif p == '2':
            val = raw_input("esc 2 value: ")
            xbee.SendStr("M2 " + str(val), addr=address)           
        elif p == '3':
            val = raw_input("esc 3 value: ")
            xbee.SendStr("M3 " + str(val), addr=address)           
        elif p == '4':
            val = raw_input("esc 4 value: ")
            xbee.SendStr("M4 " + str(val), addr=address)           
        elif p == 'p':
            val = raw_input("new Kp value: ")
            xbee.SendStr("KP " + str(val), addr=address)
        elif p == 'i':
            val = raw_input("new Ki value: ")
            xbee.SendStr("KI " + str(val), addr=address)
        elif p == 'd':
            val = raw_input("new Kd value: ")
            xbee.SendStr("KD " + str(val), addr=address)
