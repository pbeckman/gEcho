import sys
import time
from rtmidi import RtMidiIn, RtMidiOut, MidiMessage
from threading import Thread, Timer
import numpy as np

def normal(mu, sigma):
    if sigma <= 0:
        return mu
    else:
        return np.random.normal(mu, sigma)


class Gesture():
    def __init__(self, vector=None, vec_type="dist"):
        self.messages = []
        self.times = []
        # if a vector is provided, generate from it
        if vector is not None:
            if vec_type == "dist":
                starting_pitch, num_notes, vel_mean, vel_std, int_mean, int_std, \
                diff_mean, diff_std, len_mean, len_std = vector

                # sligthly randomize starting pitch
                # starting_pitch = int(round(np.random.normal(starting_pitch, 4)))

                prev_nn = 0 # midi note number for previous note on
                prev_time = 0 # start time for previous note on
                # random generate a similar gesture based on distributions
                for i in range(num_notes):
                    # if no messages have been written, use starting pitch
                    if self.messages == []:
                        self.messages.append(
                            MidiMessage.noteOn(1, starting_pitch, int(round(normal(vel_mean, vel_std))))
                            )
                        self.times.append(0.0)

                        self.messages.append(
                            MidiMessage.noteOff(1, starting_pitch)
                            )
                        self.times.append(normal(len_mean, len_std))

                        prev_nn = starting_pitch
                        prev_time = 0.0
                    else:
                        nn = prev_nn + int(round(normal(int_mean, int_std)))
                        time = prev_time + normal(diff_mean, diff_std)
                        self.messages.append(
                            MidiMessage.noteOn(1, nn, int(round(normal(vel_mean, vel_std))))
                            )
                        self.times.append(time)

                        self.messages.append(
                            MidiMessage.noteOff(1, nn)
                            )
                        self.times.append(time + normal(len_mean, len_std))

                        prev_nn = nn
                        prev_time = time


    def add_message(self, message, time):
        self.messages.append(message)
        self.times.append(time)

    def dist_vector(self):
        starting_pitch = 0
        num_notes = 0
        velocities = []
        intervals = []
        diffs = [] # time between note onsets
        lengths = []

        prev_nn = 0 # midi note number for previous note on
        prev_time = 0 # start time for previous note on
        starts = [0] * 120 # keep track of note start times
        for (msg, time) in zip(self.messages, self.times):
            if msg.isNoteOn():
                nn = msg.getNoteNumber()
                if starting_pitch == 0:
                    # assign starting pitch
                    starting_pitch = nn
                else:
                    # record interval
                    intervals.append(nn - prev_nn)
                    # record diff
                    diffs.append(time - prev_time)
                # update last note number and start time
                prev_nn = nn
                prev_time = time
                # record velocity
                velocities.append(msg.getVelocity())
                # count note
                num_notes += 1
                # mark note as sustained starting at its start time
                starts[nn] = time
            if msg.isNoteOff():
                nn = msg.getNoteNumber()
                # record length
                lengths.append(time - starts[nn])

        if num_notes == 1:
            return [
                starting_pitch,
                1,
                velocities[0],
                0,
                None,
                None,
                None,
                None,
                lengths[0],
                0
            ]
        else:
            return [
                starting_pitch,
                num_notes,
                np.mean(velocities),
                np.std(velocities),
                np.mean(intervals),
                np.std(intervals),
                np.mean(diffs),
                np.std(diffs),
                np.mean(lengths),
                np.std(lengths)
            ]

    def play(self, device):
        # print self.times
        # print self.messages
        for (msg, time) in zip(self.messages, self.times):
            # print time
            t = Timer(time, device.sendMessage, (msg,))
            t.start()


class Server(Thread):
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
        self.quit = False

    def run(self):
        self.in_device.openPort(self.in_port)
        self.out_device.openPort(self.out_port)
        self.in_device.ignoreTypes(True, False, True)
        while True:
            if self.quit:
                return

            msg = self.in_device.getMessage()
            if msg:
                self.add_to_gesture(msg)

            if self.g_ongoing and not any(self.sustained) and time.time()-self.g_last > self.g_break:
                self.g_ongoing = False
                print self.gesture.dist_vector()
                # compress and decompress gesture
                g = Gesture(self.gesture.dist_vector(), "dist")
                # print g.dist_vector()
                # play decompressed vector
                g.play(self.out_device)

    def add_to_gesture(self, msg):
        if msg.isNoteOn():
            if not self.g_ongoing:
                self.gesture = Gesture()
                self.g_start = time.time()
                self.g_ongoing = True

            self.gesture.add_message(msg, time.time()-self.g_start)
            self.g_last = time.time()
            self.sustained[msg.getNoteNumber()] = True
        if msg.isNoteOff():
            self.gesture.add_message(msg, time.time()-self.g_start)
            self.g_last = time.time()
            self.sustained[msg.getNoteNumber()] = False

        print_message(msg, self.in_name)

    def end(self):
        self.quit = True


def print_message(msg, port):
    if msg.isNoteOn():
        print '%s: ON: ' % port, msg.getNoteNumber(), "=", msg.getMidiNoteName(msg.getNoteNumber()), msg.getVelocity()
    elif msg.isNoteOff():
        print '%s: OFF:' % port, msg.getNoteNumber(), "=", msg.getMidiNoteName(msg.getNoteNumber())
    elif msg.isController():
        print '%s: CONTROLLER' % port, msg.getControllerNumber(), msg.getControllerValue()


if __name__ == '__main__':
    midi_in = RtMidiIn()
    midi_out = RtMidiOut()

    print "AVAILABLE PORTS:"
    for i in range(midi_in.getPortCount()):
        print "  ", i, midi_in.getPortName(i)

    in_port = raw_input("INPUT PORT: ")
    in_port = 1 if in_port.strip() == "" else int(in_port)
    out_port = raw_input("OUTPUT PORT: ")
    out_port = 0 if out_port.strip() == "" else int(out_port)

    server = Server(midi_in, in_port, midi_out, out_port)
    server.start()

    print 'HIT ENTER TO EXIT'
    sys.stdin.read(1)
    server.end()
