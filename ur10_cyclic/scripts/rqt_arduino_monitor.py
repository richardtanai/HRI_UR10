#!/usr/bin/env python3
import sys
from rqt_gui.main import Main

def main():
    main = Main()
    sys.exit(main.main(sys.argv, standalone='ur10_cyclic.arduino_rqt_monitor.ArduinoRqtMonitor'))

if __name__ == '__main__':
    main()
