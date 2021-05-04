#!/usr/bin/env python

import argparse
from typing import Union
from pathlib import Path
from collections import deque
from time import time, sleep
import re

import numpy as np
import gr
import serial


def get_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser()
    ap.add_argument("serial_port", type=Path, help="MCU TTY")
    ap.add_argument("baud_rate", type=int, help="Baud rate of serial connection")
    ap.add_argument(
        "-s",
        "--simulate-data",
        action="store_true",
        help="Generate simulated data instead of reading from an external source."
        " Requires a serial loopback device to be connected.",
    )
    ap.add_argument("--ymin", type=float, default=0, help="Fixed min value of y range")
    ap.add_argument(
        "--ymax", type=float, default=4096, help="Fixed max value of y range"
    )
    ap.add_argument(
        "--tick-spacing", type=float, default=500, help="Spacing between major ticks"
    )
    ap.add_argument(
        "--column",
        type=int,
        default=0,
        help="Select index of data to plot if multiple values are streamed in a line.",
    )
    return ap.parse_args()


def init_plot_window(xmin: float, xmax: float, ymin: float, ymax: float):
    gr.clearws()
    gr.setwsviewport(0.0, 0.2, 0.0, 0.2)  # Desktop window extents in meters
    gr.setviewport(0.15, 0.95, 0.15, 0.95)
    gr.setwindow(xmin, xmax, ymin, ymax)


def wrap(x, a, b):
    """Return x wrapped into the interval [a, b)

    Parameters
    ----------
    x : scalar or ndarray
    a, b : floats

    Returns
    -------
    scalar or ndarray
    """
    return a + (x - a) % (b - a)


def draw_axes(
    x_tick_spacing,
    y_tick_spacing,
    x_origin,
    y_origin,
    x_major=5,
    y_major=5,
    x_title="x",
    y_title="y",
):
    """
    Parameters
    ----------
    x_tick_spacing, y_tick_spacing : float
        Distance between ticks in data units
    x_origin, y_origin : float
        Location of plot origin in data units
    x_major, y_major : int
        Every x_major-th tick will be a labeled major tick on the x axis. Same for y.
        The ticks in between are unlabeled minor ticks.
    """
    gr.setlinewidth(1)
    gr.axes(x_tick_spacing, y_tick_spacing, x_origin, y_origin, x_major, y_major, -0.01)
    midway = 0.54
    gr.textext(midway, 0.02, "x")
    gr.setcharup(-1, 0)  # Vertical, end-up
    gr.textext(0.05, midway, "y")
    gr.setcharup(0, 1)  # Back to horizontal


def linecolor(r: float, g: float, b: float):
    """
    Parameters
    ----------
    r, g, b: Color intensities in [0.0, 1.0]
    """
    gr.setlinecolorind(gr.inqcolorfromrgb(r, g, b))


class FakeDataGenerator:
    def __init__(self):
        self.x = 0

    def simulate_data(self, ser: serial.Serial):
        self.x += np.random.randint(low=0, high=101) - 50
        self.x = wrap(self.x, 0, 4095)
        ser.write((str(self.x) + "\n").encode("utf-8"))


def read_data(ser: serial.Serial, pattern: None) -> Union[str, None]:
    line = ser.readline()
    try:
        line = line.decode()
    except UnicodeDecodeError:
        print("Unable to decode to UTF-8:", line)
        return None
    if len(line) == 0:
        return None
    if pattern not in line:
        print("Skipping line:", line)
        return None
    return line.strip()


def main():
    args = get_args()
    assert (
        args.serial_port.is_char_device()
    ), f"Not a valid character device: {str(args.serial_port)}"

    if args.simulate_data:
        generator = FakeDataGenerator()

    ser = serial.Serial(str(args.serial_port), args.baud_rate, timeout=1)

    timestep = 0.001

    init_plot_window(0, 1, 0, 1)

    queue_size = 1000
    t = deque(maxlen=queue_size)
    y1 = deque(maxlen=queue_size)

    counter = 0
    value = 0
    t0 = time()

    while True:
        start = time()

        if args.simulate_data:
            generator.simulate_data(ser)

        line = read_data(ser, "AS5600")
        if line is None:
            print("Skipping serial data")
            continue

        # Find all numbers in line, whether int or float
        s = re.findall(r"-?\d+\.?\d*", line.split("AS5600")[-1])

        try:
            value = float(s[args.column])
        except e:
            print(e)
            print("Skipping line:", line)
            continue

        print(line, value)

        t.append(time() - t0)
        y1.append(value)

        if counter > 0:
            xmin, xmax = t[0], t[-1]
            gr.clearws()
            gr.setwindow(xmin, xmax, args.ymin, args.ymax)

            gr.setlinewidth(2)
            linecolor(0, 0, 1.0)
            gr.polyline(t, y1)

            gr.setlinewidth(1)
            linecolor(0, 0, 0)
            draw_axes(1.0, args.tick_spacing, xmin, args.ymin, x_major=2, y_major=2)
            gr.updatews()
        counter += 1
        sleep(max(timestep - (time() - start), 0.0))


main()
