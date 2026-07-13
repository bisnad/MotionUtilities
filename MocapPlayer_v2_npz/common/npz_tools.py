import numpy as np
import re

class NPZ_Tools:

    def load(self, file_name):
        """Loads the raw Numpy dictionary."""
        npz_data = np.load(file_name)
        return npz_data

