#!/usr/bin/env python
"""Usage:
  arduirc.py [options] send ( <section> <command> | <timings> )
  arduirc.py analyze <timings_csv>
  arduirc.py automation ( enable | disable )
  arduirc.py alarm ( enable | disable )

Options:
  -h --help               show this help and exit
  -s --server=HOSTNAME    the MQTT server to connect to [default: localhost].
  -u --duty-high=USEC     the high part of the duty cycle in usec (0-255) [default: 26]
  -l --duty-low=USEC      the low part of the duty cycle in usec (0-255) [default: 0]
  -v --version            show version and exit
  -t --timings-file=FILE  specify the timings file to use [default: timings.yml]
  -w --wait               wait two seconds for the Arduino to reset.
  -r --repeat=REPEAT      repeat the command REPEAT times [default: 3]
  -d --delay=DELAY        delay between repeats (in usec) [default: 10000]
  -p --pin=PIN            which GPIO pin to write to [default: 4]
"""

import csv
import sys
import yaml
import struct

from docopt import docopt
import paho.mqtt.client as mqtt


def mqtt_send(server, payload):
    # chr(1) is the command (command 1, send timings).
    client = mqtt.Client()
    client.connect(server)
    client.publish("esplights_command", bytearray(payload))


def send(arguments):
    # Get the timings.
    pin = int(arguments["--pin"])
    repeat = int(arguments["--repeat"])
    delay = int(round(int(arguments["--delay"]) / 100.0))
    duty_high = max(1, int(arguments["--duty-high"]) + 1)
    duty_low = max(1, int(arguments["--duty-low"]) + 1)

    if arguments.get("<section>") and arguments.get("<command>"):
        section = arguments["<section>"]
        command = arguments["<command>"]

        try:
            timing_dict = yaml.load(open(arguments["--timings-file"]))
        except IOError:
            sys.exit("Error opening timings file.")

        if section not in timing_dict:
            sys.exit("Unknown section.")

        if command not in timing_dict[section]["timings"]:
            sys.exit("Unknown command.")

        raw_timings = timing_dict[section]["timings"][command]

        # Override the command line with the parameters in the file.
        params = timing_dict[section].get("parameters", {})
        pin = params.get("pin", pin)
        duty_high = params.get("duty_high", duty_high)
        duty_low = params.get("duty_low", duty_low)
        repeat = params.get("repeat", repeat)
        delay = params.get("delay", delay)

    elif arguments.get("<timings>"):
        raw_timings = arguments["<timings>"]

    timings = [int(timing) for timing in raw_timings.split()]

    output = struct.pack("!" + ("h" * len(timings)), *timings)
    output = output.replace(chr(0), chr(1))  # Can't have null bytes.

    if delay > 255 or delay < 1:
        sys.exit("Delay must be between 100 and 25500.")

    if pin > 13 or pin < 0:
        sys.exit("Pin must be between 0 and 13.")

    if repeat > 255 or repeat < 1:
        sys.exit("Repeat must be between 1 and 255.")

    output = chr(1) + chr(duty_high) + chr(duty_low) + chr(pin) + chr(repeat) + chr(delay) + output + chr(0)
    mqtt_send(arguments["--server"], output)

    print("Command sent.")


def automation(enable):
    if arguments["enable"]:
        mqtt_send(arguments["--server"], "auto on")
    else:
        mqtt_send(arguments["--server"], "auto off")
    print("Command sent.")


def alarm(enable):
    if arguments["enable"]:
        mqtt_send(arguments["--server"], "alarm on")
    else:
        mqtt_send(arguments["--server"], "alarm off")
    print("Command sent.")


def analyze(arguments):
    """
    Read a CSV file from a logic analyzer and print out a send-compatible
    timings list.
    """
    reader = csv.reader(open(arguments["<timings_csv>"]))

    # Read and convert seconds to microseconds.
    timings = [int(1000000 * float(x[0])) for x in reader]

    # Normalize to 0 as the first value.
    timings = [x - timings[0] for x in timings]

    # Calculate the differences.
    diffs = [x[0] - x[1] for x in zip(timings[1:], timings)]
    print "Timings: %s" % " ".join([str(x) for x in diffs])


def main(arguments):
    if arguments["send"]:
        send(arguments)
    elif arguments["automation"]:
        automation(arguments)
    elif arguments["alarm"]:
        alarm(arguments)
    elif arguments["analyze"]:
        analyze(arguments)


if __name__ == "__main__":
    arguments = docopt(__doc__, version="0.1.0")
    main(arguments)
