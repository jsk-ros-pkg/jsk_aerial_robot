#!/usr/bin/env python3
import csv
import matplotlib.pyplot as plt
def main():
    body_size = 0.25 #[radius]
    figure, axes = plt.subplots()
    with open("real.csv", 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            x = float(row[1])
            y = float(row[2])
            r = float(row[8])+body_size
            draw_circle = plt.Circle((x, y), r)
            axes.add_artist(draw_circle)
    plt.title('ObstacleMap')
    plt.xlim([-0.2, 4.2])
    plt.ylim([-1.5, 1.5])
    plt.show()
if __name__ == "__main__":
    main()