#!/usr/bin/env python3
"""
Simple differential drive control for PiCar back wheels over SSH
"""

from .back_wheels import Back_Wheels  # Make sure this is your modified class
import sys

def main():
    bw = Back_Wheels(debug=True)  # debug=True prints motor actions
    bw.speed = 50
    print("PiCar Differential Drive Control")
    print("Commands: forward, backward, left, right, stop, exit")

    while True:
        cmd = input("Enter command: ").strip().lower()

        if cmd == "forward":
            bw.forward()
        elif cmd == "backward":
            bw.backward()
        elif cmd == "left":
            bw.spin_left()
        elif cmd == "right":
            bw.spin_right()
        elif cmd == "stop":
            bw.stop()
        elif cmd == "exit":
            bw.stop()
            print("Exiting, motors stopped")
            break
        else:
            print("Unknown command. Try: forward, backward, left, right, stop, exit")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected, stopping motors")
        bw.stop()
