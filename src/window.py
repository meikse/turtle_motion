#!/usr/bin/env python3

import subprocess

def main():

    NAME = "TurtleSim"          # TODO -> make ros param 
    # NAME = "nvim window.py"   # for debugging

    shell_out= subprocess.Popen(["xwininfo","-name", NAME], 
                         stdout=subprocess.PIPE,
                         stderr = subprocess.STDOUT)

    out = shell_out.stdout.read()   # get shell output
    out = out.decode()              # convert from byte to string
    out = out.splitlines()          # delimiter = "\n"

    # find position in list for abs windows location in shell output
    parsed_out = [out[i].find("Absolute") for i, _ in enumerate(out)]
    # receive index of finding
    index = [i for i, val in enumerate(parsed_out) if val != -1]
    # filter values of from list via index
    win_pos = [int(''.join(filter(str.isdigit, out[i]))) for i in index]
    print(win_pos)

if __name__ == '__main__':
    main()

