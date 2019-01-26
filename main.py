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
        # midi messages and times
        self.messages = []
        self.msg_times = []
        # note basic data
        self.notes = []
        self.velocities = []
        self.note_times = []
        self.lengths = []
        # note differential data
        self.intervals = []
        self.diffs = [] # time between note onsets
        # whether or not note data has been generated
        self.collected = False

        # if a vector is provided, load from it
        if vector is not None:
            if vec_type == "dist":
                self.load_dist_vec(vector)

    def add_message(self, message, time):
        self.messages.append(message)
        self.msg_times.append(time)

    def collect(self):
        """
        Convert midi messages into interperable note data.
        """
        prev_nn = 0 # midi note number for previous note on
        prev_time = 0 # start time for previous note on
        starts = [0] * 120 # keep track of note start times
        pedal = False # whether the sustain pedal is down
        sustained = [False] * 120 # keep track of notes held only by sustain pedal

        for (msg, time) in zip(self.messages, self.msg_times):
            if msg.isController() and msg.getControllerNumber() == 64:
                val = msg.getControllerValue()
                # set sustain pedal appropriately
                if val == 127:
                    pedal = True
                else:
                    pedal = False
                    # record lengths of all sustained notes, which have just ended
                    for nn in [i for (i, sus) in enumerate(sustained) if sus]:
                        # record length
                        self.lengths.append(time - starts[nn])
            if msg.isNoteOn():
                nn = msg.getNoteNumber()
                # record note number
                self.notes.append(nn)
                # record velocity
                self.velocities.append(msg.getVelocity())
                # record note start time
                self.note_times.append(time)
                # mark note as pressed starting at its start time
                starts[nn] = time
            if msg.isNoteOff():
                nn = msg.getNoteNumber()
                if pedal:
                    # don't record length, just mark as sustained
                    sustained[nn] = True
                else:
                    # record length
                    self.lengths.append(time - starts[nn])

        # compute intervals
        self.intervals = [self.notes[i+1]-self.notes[i] for i in range(len(self.notes)-1)]
        # compute start time differentials
        self.diffs = [self.note_times[i+1]-self.note_times[i] for i in range(len(self.note_times)-1)]

        # note data assembled
        self.collected = True

    def dist_vector(self):
        # if notes have
        if not self.collected:
            self.collect()

        if len(self.notes) == 1:
            return [
                self.notes[0],
                1,
                self.velocities[0],
                0,
                None,
                None,
                None,
                None,
                self.lengths[0],
                0
            ]
        else:
            return [
                self.notes[0],
                len(self.notes),
                np.mean(self.velocities),
                np.std(self.velocities),
                np.mean(self.intervals),
                np.std(self.intervals),
                np.mean(self.diffs),
                np.std(self.diffs),
                np.mean(self.lengths),
                np.std(self.lengths)
            ]

    def load_dist_vec(self, vector):
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
                self.msg_times.append(0.0)

                self.messages.append(
                    MidiMessage.noteOff(1, starting_pitch)
                    )
                self.msg_times.append(normal(len_mean, len_std))

                prev_nn = starting_pitch
                prev_time = 0.0
            else:
                nn = prev_nn + int(round(normal(int_mean, int_std)))
                time = prev_time + normal(diff_mean, diff_std)
                self.messages.append(
                    MidiMessage.noteOn(1, nn, int(round(normal(vel_mean, vel_std))))
                    )
                self.msg_times.append(time)

                self.messages.append(
                    MidiMessage.noteOff(1, nn)
                    )
                self.msg_times.append(time + normal(len_mean, len_std))

                prev_nn = nn
                prev_time = time

    def play(self, device):
        for (msg, time) in zip(self.messages, self.msg_times):
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
        self.pedal = False
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

            if self.g_ongoing and not any(self.sustained) and not self.pedal and time.time()-self.g_last > self.g_break:
                self.g_ongoing = False
                print self.gesture.dist_vector()
                # compress and decompress gesture
                g = Gesture(self.gesture.dist_vector(), "dist")
                # play decompressed vector
                g.play(self.out_device)

    def add_to_gesture(self, msg):
        if not self.g_ongoing:
            self.gesture = Gesture()
            self.g_start = time.time()
            self.g_ongoing = True
        if msg.isController() and msg.getControllerNumber() == 64:
            if msg.getControllerValue() == 127:
                self.pedal = True
            else:
                self.pedal = False
        if msg.isNoteOn():
            self.sustained[msg.getNoteNumber()] = True
        if msg.isNoteOff():
            self.sustained[msg.getNoteNumber()] = False

        self.gesture.add_message(msg, time.time()-self.g_start)
        self.g_last = time.time()

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
