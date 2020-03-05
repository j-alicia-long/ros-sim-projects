# The MovingAverage implements the concept of window in a set of measurements.

# The window_size is the number of most recent measurements used in average.

# The measurements are continuously added using the method add.

# The get_average method must return the average of the last window_size measurements.

class MovingAverage():

    def __init__(self, window_size):
        self.stored = []
        self.window_size = window_size

    # add a new measurement
    def add(self, val):
        self.stored.append(val)
        if len(self.stored) > self.window_size:
            self.stored.pop(0)
     
    # return the average of the last window_size measurements added 
    # or the average of all measurements if less than window_size were provided
    def get_average(self):
        avg = 0
        if len(self.stored) > 0:
            avg = sum(self.stored)/len(self.stored)
        return avg