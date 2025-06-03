import numpy as np

def euclidean_dist(x, y, thresh=None):
    """
    Measures the euclidean distance between points x and y. If a threshold value is provided, this function returns True
    if the calculated distance is smaller than the threshold, or False otherwise. If no threshold is provided, this
    function returns the computed distance.

    :param x: n-dimension point
    :param y: n-dimension point
    :param thresh: threshold
    :type thresh: float
    :return: If thresh is not None: whether x and y are closer to each other than the threshold.
    If thresh is None: the distance between x and y
    """

    dist = np.sqrt(np.sum((x - y) ** 2))

    if thresh is None:
        return dist

    return dist < thresh