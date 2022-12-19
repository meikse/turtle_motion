#!/usr/bin/env python3

from pyautogui import position 
from time import sleep

def main():
    try:
        while True:
            x, y = position()
            print (x,y, flush=True)
            sleep(.1)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()

