import numpy as np

class Gesture():
    """General class for musical gestures generated from midi data"""
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
        # whether or not midi data has been dollected into note data
        self.collected = False

        # if a vector is provided, load from it
        if vector is not None:
            if vec_type == "dist":
                self.load_dist_vec(vector)

    def add_message(self, message, time):
        """Add midi message to gesture"""
        self.messages.append(message)
        self.msg_times.append(time)

    def collect(self):
        """Convert midi messages into interperable note data"""
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

    def play(self, device):
        """Spawn a series of timer threads to play back gesture on the given device"""
        for (msg, time) in zip(self.messages, self.msg_times):
            t = Timer(time, device.sendMessage, (msg,))
            t.start()
