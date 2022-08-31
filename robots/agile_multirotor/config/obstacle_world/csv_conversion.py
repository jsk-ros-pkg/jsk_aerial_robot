#!/usr/bin/env python

import pandas as pd
import numpy as np

if __name__=="__main__":
    df = pd.read_csv("world1.csv", header=None)
    for i in range(len(df)):
        name = 'obj' + str(i+1)
        df.loc[i, 1:3] = np.array(df.loc[i, 1:3].tolist())/5
    df.to_csv("world1_new.csv", header=False,index=False)
