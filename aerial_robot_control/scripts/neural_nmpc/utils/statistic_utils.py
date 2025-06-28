import numpy as np
import matplotlib.pyplot as plt

def prune_dataset(x, y, x_cap, n_bins, thresh, plot=False, labels=None):
    """
    Prunes the collected model error dataset with two filters based on the linear velocity.
    First, remove values where the input values (velocities) exceed 10.
    Second, create a histogram for each of the three axial velocity errors (y) with the specified number of
    bins and remove any data where the total amount of samples in that bin is less than the specified threshold ratio.
    :param x: Dataset of (velocity) input features. Dimensions N x n (N entries and n dimensions)
    :param y: Dataset of errors (in velocity). Dimensions N x m (N entries and m dimensions)
    :param x_cap: Remove values from dataset if x > x_cap or x < -x_cap
    :param n_bins: Number of bins used for histogram
    :param thresh: Threshold ratio below which data from that bin will be removed
    :param plot: Make a plot of the pruning
    :param labels: Labels to use for the plot
    :return: The indices to keep after pruning
    """
    pruned_idx_unique = np.zeros(0, dtype=int)

    # === 1st pruning filter ===
    # Prune by extremum: Discard samples where the absolute value of the input exceeds x_cap
    # Makes most sense for velocities: max axial velocity = vel_cap m/s
    if x_cap is not None:
        for i in range(x.shape[1]):
            pruned_idx = np.where(np.abs(x[:, i]) > x_cap)[0]
            pruned_idx_unique = np.unique(np.append(pruned_idx, pruned_idx_unique))

    # === 2nd pruning filter ===
    # Prune by error histogram dimension wise: Discard bins with in each dimension less than thresh (e.g., 1%) of the data
    for i in range(y.shape[1]):
        h, bins = np.histogram(y[:, i], bins=n_bins)
        for j in range(len(h)):
            # Check if ratio of samples in bin j to total number of samples is below threshold
            if h[j] / np.sum(h) < thresh:
                # Get indices of samples in bin j
                pruned_idx = np.where(np.logical_and(bins[j + 1] >= y[:, i], y[:, i] >= bins[j]))
                pruned_idx_unique = np.unique(np.append(pruned_idx, pruned_idx_unique))

    # === 3rd pruning filter ===
    # Prune by error histogram norm: Discard bins with the norm less than thresh (e.g., 1%) of the data
    y_norm = np.linalg.norm(y, axis=1)
    h, norm_bins = np.histogram(y_norm, bins=n_bins)
    # TODO following doesnt make sense since h will already be normalized
    for j in range(len(h)):
        if h[j] / np.sum(h) < thresh:
            pruned_idx = np.where(np.logical_and(norm_bins[j + 1] >= y_norm, y_norm >= norm_bins[j]))
            pruned_idx_unique = np.unique(np.append(pruned_idx, pruned_idx_unique))

    # === Plotting (original) ===
    if plot:
        plt.figure()
        plot_bins = []
        # Error histograms for each dimension
        for i in range(y.shape[1]):
            plt.subplot(y.shape[1] + 1, 1, i + 1)
            h, bins = np.histogram(y[:, i], bins=n_bins)
            plot_bins.append(bins)
            plt.bar(bins[:-1], h, np.ones_like(h) * (bins[1] - bins[0]), align='edge', label='discarded')
            if labels is not None:
                plt.ylabel(labels[i])
        # Normalized error histogram
        plt.subplot(y.shape[1] + 1, 1, y.shape[1] + 1)
        h, norm_bins = np.histogram(y_norm, bins=n_bins)
        h = h / np.sum(h)
        plt.bar(norm_bins[:-1], h, np.ones_like(h) * (norm_bins[1] - norm_bins[0]), align='edge', label='discarded')

    # === Pruning ===
    # Remove pruned indices from the dataset
    y = np.delete(y, pruned_idx_unique, axis=0)

    # === Plotting (pruned) ===
    if plot:
        # Error histograms for each dimension
        for i in range(y.shape[1]):
            plt.subplot(y.shape[1] + 1, 1, i + 1)
            h, bins = np.histogram(y[:, i], bins=plot_bins[i])
            plt.bar(bins[:-1], h, np.ones_like(h) * (bins[1] - bins[0]), align='edge', label='kept')
            plt.legend()

        # Normalized error histogram
        plt.subplot(y.shape[1] + 1, 1, y.shape[1] + 1)
        h, bins = np.histogram(np.linalg.norm(y, axis=1), bins=norm_bins)
        h = h / np.sum(h)
        plt.bar(bins[:-1], h, np.ones_like(h) * (bins[1] - bins[0]), align='edge', label='kept')
        plt.ylabel('Error norm')
        plt.show()

    kept_idx = np.delete(np.arange(0, x.shape[0]), pruned_idx_unique)
    return kept_idx