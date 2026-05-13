import numpy as np

class EMAFilter:
    def __init__(self, alpha=0.15):
        """
        Exponential Moving Average (EMA) Filter.
        alpha: Smoothing factor between 0.0 and 1.0.
               Higher = more responsive, but more jitter.
               Lower  = smoother, but introduces lag.
               0.15 is a good starting point for webcams.
        """
        self.alpha = alpha
        self.previous_val = None

    def update(self, current_val):
        """Updates the filter with a new value (like an [x, y, z] coordinate)."""
        if current_val is None:
            return None
            
        current_val = np.array(current_val, dtype=float)
        
        # If this is the first frame, we have no history to smooth against
        if self.previous_val is None:
            self.previous_val = current_val
            return current_val

        # The Magic Math: Blend the new value with the old value
        smoothed_val = (self.alpha * current_val) + ((1.0 - self.alpha) * self.previous_val)
        
        # Save this smoothed value for the next frame
        self.previous_val = smoothed_val
        
        return smoothed_val.tolist()