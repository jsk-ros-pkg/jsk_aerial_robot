import numpy as np


class FIRDifferentiator:
    def __init__(self, coefficients, sampling_frequency):
        """
        Initializes the FIR Differentiator with the provided filter coefficients and sampling frequency.

        :param coefficients: The FIR filter coefficients.
        :param sampling_frequency: The sampling frequency of the input signal in Hz.
        """
        self.coefficients = np.array(coefficients)
        self.sampling_frequency = sampling_frequency
        self.buffer = np.zeros(len(coefficients))

    def apply_single(self, sample):
        """
        Applies the FIR differentiator to a single input sample.

        :param sample: Input sample (a single number).
        :return: Filtered output (a single number).
        """
        # Shift the buffer and add the new signal sample
        self.buffer[1:] = self.buffer[:-1]
        self.buffer[0] = sample

        # Calculate the output as the dot product of coefficients and buffer
        output = np.dot(self.coefficients, self.buffer)

        # Scale the output by the sampling frequency
        output *= self.sampling_frequency

        return output