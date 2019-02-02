import sys
import time
import resource
import numpy as np
import pickle as pkl
from rtmidi import RtMidiIn, RtMidiOut
from threading import Thread, Timer
from gesture import Gesture, Collecture, randomize
from representations import data_matrix

class Server(Thread):
    """Thread-based server for midi input, output, and gesture processing"""
    def __init__(self, in_device, in_port, out_device, out_port):
        Thread.__init__(self)
        self.setDaemon(True)
        # ports
        self.in_port = in_port
        self.in_device = in_device
        self.in_name = in_device.getPortName(in_port)
        self.out_port = out_port
        self.out_device = out_device
        self.out_name = out_device.getPortName(out_port)
        # gesture
        self.gesture = None
        self.g_start = None
        self.g_last = None
        self.g_ongoing = False
        self.g_break = 1.0
        # state
        self.sustained = [False] * 120
        self.pedal = False
        self.quit = False

    def run(self):
        """The server's main loop"""
        Xs = []
        self.in_device.openPort(self.in_port)
        self.out_device.openPort(self.out_port)
        self.in_device.ignoreTypes(True, False, True)
        while True:
            if self.quit:
                return

            msg = self.in_device.getMessage()
            if msg:
                self.add_to_gesture(msg)

            if self.g_ongoing and not any(self.sustained) and not self.pedal and time.time()-self.g_last > self.g_break:
                self.g_ongoing = False
                c = Collecture(self.gesture)
                c = randomize(c)
                c.play(self.out_device)
                # compress and decompress gesture
                # g = Gesture(self.gesture.dist_vector(), "dist")
                # play decompressed vector
                # g.play(self.out_device)

    def add_to_gesture(self, msg):
        """Add messages to ongoing gesture"""
        # start a new gesture if one is not ongoing
        if not self.g_ongoing:
            self.gesture = Gesture()
            self.g_start = time.time()
            self.g_ongoing = True
        # set sustain pedal control
        if msg.isController() and msg.getControllerNumber() == 64:
            if msg.getControllerValue() == 127:
                self.pedal = True
            else:
                self.pedal = False
        # mark note as sustained or completed
        if msg.isNoteOn():
            self.sustained[msg.getNoteNumber()] = True
        if msg.isNoteOff():
            self.sustained[msg.getNoteNumber()] = False

        # add midi message and time since start to gesture
        self.gesture.add_message(msg, time.time()-self.g_start)
        # record event time as last to inform gesture ending
        self.g_last = time.time()

        print_message(msg, self.in_name)

    def end(self):
        self.quit = True


def print_message(msg, port):
    """Print midi messages"""
    if msg.isNoteOn():
        print '%s: ON: ' % port, msg.getNoteNumber(), "=", msg.getMidiNoteName(msg.getNoteNumber()), msg.getVelocity()
    elif msg.isNoteOff():
        print '%s: OFF:' % port, msg.getNoteNumber(), "=", msg.getMidiNoteName(msg.getNoteNumber())
    elif msg.isController():
        print '%s: CONTROLLER' % port, msg.getControllerNumber(), msg.getControllerValue()


if __name__ == '__main__':
    # increase stack size to avoid segfault
    resource.setrlimit(
        resource.RLIMIT_CORE,
        (resource.RLIM_INFINITY, resource.RLIM_INFINITY)
        )

    midi_in = RtMidiIn()
    midi_out = RtMidiOut()

    print "AVAILABLE PORTS:"
    for i in range(midi_in.getPortCount()):
        print "  ", i, midi_in.getPortName(i)

    in_port  = raw_input("INPUT PORT  (1): ")
    in_port  = 1 if in_port.strip() == "" else int(in_port)
    out_port = raw_input("OUTPUT PORT (0): ")
    out_port = 0 if out_port.strip() == "" else int(out_port)

    server = Server(midi_in, in_port, midi_out, out_port)
    server.start()

    print 'HIT ENTER TO EXIT'
    sys.stdin.read(1)
    server.end()
