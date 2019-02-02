import numpy as np
from rtmidi import MidiMessage
from threading import Timer
from sklearn.cluster import DBSCAN

class Gesture():
    """General class for musical gestures generated from midi data"""
    def __init__(self):
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

    def add_message(self, message, time):
        """Add midi message to gesture"""
        self.messages.append(message)
        self.msg_times.append(time)

    def collect(self):
        """Collect and convert midi messages into interperable note data"""
        prev_nn = 0 # midi note number for previous note on
        prev_time = 0 # start time for previous note on
        attrs = [(0,0)] * 120 # keep track of note start times and velocities
        pedal = False # whether the sustain pedal is down
        sustained = [False] * 120 # keep track of notes held only by sustain pedal

        for (msg, time) in zip(self.messages, self.msg_times):
            if msg.isController() and msg.getControllerNumber() == 64:
                # set sustain pedal
                val = msg.getControllerValue()
                if val == 127:
                    pedal = True
                else:
                    pedal = False
                    # record attributes of all sustained notes, which have just ended
                    for nn in [i for (i, sus) in enumerate(sustained) if sus]:
                        self.notes.append(nn)
                        self.velocities.append(attrs[nn][1])
                        self.note_times.append(attrs[nn][0])
                        self.lengths.append(time - attrs[nn][0])
                        sustained[nn] = False
            if msg.isNoteOn():
                nn = msg.getNoteNumber()
                # save note start time and velocity
                # wait until note end to record in class attributes to keep everything aligned
                attrs[nn] = (time, msg.getVelocity())
            if msg.isNoteOff():
                nn = msg.getNoteNumber()
                if pedal:
                    # don't record length, just mark as sustained
                    sustained[nn] = True
                else:
                    # record completed note attributes
                    self.notes.append(nn)
                    self.velocities.append(attrs[nn][1])
                    self.note_times.append(attrs[nn][0])
                    self.lengths.append(time - attrs[nn][0])

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

class Collecture():
    def __init__(self, gesture):
        self.subgestures = []
        self.times = []

        # use clustering to subdivide gesture and compute distributions separately
        X = data_matrix(gesture)
        labels = DBSCAN(eps=0.3, min_samples=3).fit_predict(X)
        # labels = [0 if l==-1 else l for l in labels]
        print labels

        l_set = set(labels)
        for l in l_set:
            l_inds = [i for i in range(len(labels)) if labels[i] == l]
            subgesture_X = X.take(l_inds, axis=0)
            g = from_data_matrix(subgesture_X)
            # print g.note_times
            self.subgestures.append(g)
            self.times.append(min(subgesture_X[:, 2]))

    def play(self, device):
        """Spawn a series of timer threads to play back subgestures on the given device"""
        for (subgesture, time) in zip(self.subgestures, self.times):
            # print subgesture.notes, subgesture.note_times
            t = Timer(time, subgesture.play, (device,))
            t.start()

def data_matrix(gesture, scaled=True):
    """Representation of gesture as a real-valued matrix
       Each row is a note with midi number, velocity, start time, and length"""
    # if midi messages have not been collected into notes do so
    if not gesture.collected:
      gesture.collect()

    mat = np.array([[n, v, t, l] for (n, v, t, l) in zip(
          gesture.notes,
          gesture.velocities,
          gesture.note_times,
          gesture.lengths
          )])

    if scaled:
      # linearly interpolate rough possible input ranges to [0, 1]
      return np.column_stack((
          np.interp(mat[:,0], [21, 108], [0, 1]),
          np.interp(mat[:,1], [0, 127], [0, 1]),
          np.interp(mat[:,2], [0, 20], [0, 1]),
          np.interp(mat[:,3], [0.05, 10], [0, 1]),
      ))
    else:
      return mat

def from_data_matrix(mat, scaled=True):
    """Create gesture using data matrix respresentation"""
    g = Gesture()
    # gesture start time
    t_start = min(mat[:, 2])
    for (n, v, t, l) in mat:
        # shift messages to start at time zero if they dont already
        t -= t_start

        if scaled:
            n = np.interp(n, [0, 1], [21, 108])
            v = np.interp(v, [0, 1], [0, 127])
            t = np.interp(t, [0, 1], [0, 20])
            l = np.interp(l, [0, 1], [0.05, 10])

        g.messages.append(
          MidiMessage.noteOn(1, int(n), int(v))
          )
        g.msg_times.append(t)

        g.messages.append(
          MidiMessage.noteOff(1, int(n))
          )
        g.msg_times.append(t + l)

    g.collect()

    return g

def dist_vector(gesture):
    """Representation of gesture as a set of distribution parameters
       A random draw from the corresponding distributions constructs
       a variation on the gesture"""
    # if midi messages have not been collected into notes do so
    if not gesture.collected:
      gesture.collect()

    if len(gesture.notes) == 1:
      return [
          gesture.notes[0],
          1,
          gesture.velocities[0],
          0,
          None,
          None,
          None,
          None,
          gesture.lengths[0],
          0
      ]
    else:
      return [
          gesture.notes[0],
          len(gesture.notes),
          np.mean(gesture.velocities),
          np.std(gesture.velocities),
          np.mean(gesture.intervals),
          np.std(gesture.intervals),
          np.mean(gesture.diffs),
          np.std(gesture.diffs),
          np.mean(gesture.lengths),
          np.std(gesture.lengths)
      ]

def from_dist_vec(vector):
    """Create gesture by drawing a random instance from
       the distribution vector representation"""
    starting_pitch, num_notes, vel_mean, vel_std, int_mean, int_std, \
    diff_mean, diff_std, len_mean, len_std = vector

    g = Gesture()

    prev_nn = 0 # midi note number for previous note on
    prev_time = 0 # start time for previous note on
    # randomize number of notes using a poisson distribution
    num_notes = max(np.random.poisson(num_notes), 1)
    # random generate a similar gesture based on attribute normal distributions
    for i in range(num_notes):
      # if no messages have been written, use starting pitch
      if g.messages == []:
          g.messages.append(
              MidiMessage.noteOn(1, starting_pitch, int(round(normal(vel_mean, vel_std))))
              )
          g.msg_times.append(0.0)

          g.messages.append(
              MidiMessage.noteOff(1, starting_pitch)
              )
          g.msg_times.append(normal(len_mean, len_std))

          prev_nn = starting_pitch
          prev_time = 0.0
      else:
          nn = prev_nn + int(round(normal(int_mean, int_std)))
          time = prev_time + normal(diff_mean, diff_std)
          g.messages.append(
              MidiMessage.noteOn(1, nn, int(round(normal(vel_mean, vel_std))))
              )
          g.msg_times.append(time)

          g.messages.append(
              MidiMessage.noteOff(1, nn)
              )
          g.msg_times.append(time + normal(len_mean, len_std))

          prev_nn = nn
          prev_time = time

    g.collect()

    return g

def normal(mu, sigma):
    """Extension of numpy's normal to cover trivial 0 variance case"""
    if sigma <= 0:
        return mu
    else:
        return np.random.normal(mu, sigma)
