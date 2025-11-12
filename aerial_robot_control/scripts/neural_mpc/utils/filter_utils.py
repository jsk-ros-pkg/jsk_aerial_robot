import numpy as np
from scipy.signal import butter, lfilter, filtfilt, freqz
import matplotlib.pyplot as plt


def moving_average_filter(data, window_size=5):
    """
    Apply a moving average filter to the specified dimensions of the input data.

    :param y: Input data array
    :param idx: Indices of the dimensions to filter
    :param window_size: Size of the moving average window (must be odd)
    """
    N = data.shape[0]
    D = data.shape[1]
    data_filtered = np.empty_like(data)
    pad_size = window_size // 2

    for dim in range(D):
        padded_data = np.pad(data[:, dim], ((pad_size, pad_size)), mode="edge")
        for i in range(N):
            data_filtered[i, dim] = np.mean(padded_data[i : i + window_size])

    return data_filtered


def butter_lowpass(cutoff, fs, order=5):
    return butter(order, cutoff, fs=fs, btype="lowpass", analog=False)


def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order=order)
    # y = lfilter(b, a, data)
    y = filtfilt(b, a, data)
    return y


def low_pass_filter(data, order=6, fs=30.0, cutoff=3.667):
    """
    Low-pass Butterworth filter.

    :param data: Input data array
    :param order: Order of the filter
    :param fs: Sampling frequency [Hz]
    :param cutoff: Cutoff frequency [Hz]
    :return: Filtered data array
    """
    return butter_lowpass_filter(data, cutoff, fs, order)


if __name__ == "__main__":
    """
    Test the low-pass filter.
    """
    # Filter requirements.
    order = 6
    fs = 30.0  # sample rate, Hz
    cutoff = 3.667  # desired cutoff frequency of the filter, Hz

    # Test data
    T = 5.0  # seconds
    n = int(T * fs)  # total number of samples
    t = np.linspace(0, T, n, endpoint=False)
    # "Noisy" data.  We want to recover the 1.2 Hz signal from this.
    data = np.sin(1.2 * 2 * np.pi * t) + 1.5 * np.cos(9 * 2 * np.pi * t) + 0.5 * np.sin(12.0 * 2 * np.pi * t)

    # Get the filter coefficients so we can check its frequency response.
    b, a = butter_lowpass(cutoff, fs, order)

    # Plot the frequency response.
    w, h = freqz(b, a, fs=fs, worN=8000)
    plt.subplot(2, 1, 1)
    plt.plot(w, np.abs(h), "b")
    plt.plot(cutoff, 0.5 * np.sqrt(2), "ko")
    plt.axvline(cutoff, color="k")
    plt.xlim(0, 0.5 * fs)
    plt.title("Lowpass Filter Frequency Response")
    plt.xlabel("Frequency [Hz]")
    plt.grid()

    # Demonstrate the use of the filter.
    # First make some data to be filtered.
    T = 5.0  # seconds
    n = int(T * fs)  # total number of samples
    t = np.linspace(0, T, n, endpoint=False)
    # "Noisy" data.  We want to recover the 1.2 Hz signal from this.
    data = np.sin(1.2 * 2 * np.pi * t) + 1.5 * np.cos(9 * 2 * np.pi * t) + 0.5 * np.sin(12.0 * 2 * np.pi * t)

    # Filter the data, and plot both the original and filtered signals.
    y = butter_lowpass_filter(data, cutoff, fs, order)

    plt.subplot(2, 1, 2)
    plt.plot(t, data, "b-", label="data")
    plt.plot(t, y, "g-", linewidth=2, label="filtered data")
    plt.xlabel("Time [sec]")
    plt.grid()
    plt.legend()

    plt.subplots_adjust(hspace=0.35)
    plt.show()
