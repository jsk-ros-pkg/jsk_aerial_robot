### README

1. tracking nmpc, pure disturbance rejection
```
python sim_impedance_no_mhe.py 0 -e 0 -p 4
```

```bash
    # position
    pMxy: 1.5
    pMz: 1.5
    Qv_xy: 10  # pDxy
    Qv_z: 10  # pDz
    Qp_xy: 6  # pKxy
    Qp_z: 6  # pKz

    # orientation
    oMxy: 0.9
    oMz: 0.9
    Qw_xy: 10  # oDxy
    Qw_z: 10  # oDz
    Qq_xy: 15  # oKxy
    Qq_z: 15  # oKz
```

2. impedance nmpc, enlarge factor = 1

```bash
python sim_impedance_no_mhe.py 1 -e 0 -p 4
```

3. impedance nmpc, enlarge factor = 8

```bash
python sim_impedance_no_mhe.py 1 -e 0 -p 4
```

4. impedance nmpc, enlarge factor = 16

```bash
python sim_impedance_no_mhe.py 1 -e 0 -p 4
```
