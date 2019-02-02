import numpy as np
from gesture import Gesture

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

def load_dist_vec(gesture, vector):
    """Load distribution vector representation by drawing a random instance"""
    starting_pitch, num_notes, vel_mean, vel_std, int_mean, int_std, \
    diff_mean, diff_std, len_mean, len_std = vector

    prev_nn = 0 # midi note number for previous note on
    prev_time = 0 # start time for previous note on
    # random generate a similar gesture based on distributions
    for i in range(num_notes):
      # if no messages have been written, use starting pitch
      if gesture.messages == []:
          gesture.messages.append(
              MidiMessage.noteOn(1, starting_pitch, int(round(normal(vel_mean, vel_std))))
              )
          gesture.msg_times.append(0.0)

          gesture.messages.append(
              MidiMessage.noteOff(1, starting_pitch)
              )
          gesture.msg_times.append(normal(len_mean, len_std))

          prev_nn = starting_pitch
          prev_time = 0.0
      else:
          nn = prev_nn + int(round(normal(int_mean, int_std)))
          time = prev_time + normal(diff_mean, diff_std)
          gesture.messages.append(
              MidiMessage.noteOn(1, nn, int(round(normal(vel_mean, vel_std))))
              )
          gesture.msg_times.append(time)

          gesture.messages.append(
              MidiMessage.noteOff(1, nn)
              )
          gesture.msg_times.append(time + normal(len_mean, len_std))

          prev_nn = nn
          prev_time = time


def normal(mu, sigma):
    """Extension of numpy's normal to cover trivial 0 variance case"""
    if sigma <= 0:
        return mu
    else:
        return np.random.normal(mu, sigma)
