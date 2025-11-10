import numpy as np


def moving_average_filter(y, window_size=5):
    """
    Apply a moving average filter to the specified dimensions of the input data.

    :param y: Input data array
    :param idx: Indices of the dimensions to filter
    :param window_size: Size of the moving average window (must be odd)
    """
    N = y.shape[0]
    D = y.shape[1]
    y_filtered = np.empty_like(y)
    pad_size = window_size // 2

    for dim in range(D):
        padded_data = np.pad(y[:, dim], ((pad_size, pad_size)), mode="edge")
        for i in range(N):
            y_filtered[i, dim] = np.mean(padded_data[i : i + window_size])

    return y_filtered
