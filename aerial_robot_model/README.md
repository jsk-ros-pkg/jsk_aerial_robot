# Robot model for aerial robot

## Robot model (under ./robots)
- hydrus3: 3rd generation hydrus which has 4 links
- hydrusx: 4rd generation hydrus which has links more than 4 links. E.g. penta with 5 links, hex with 6 links.
- dragon2: prototype of dragon

## Usage
- hyrus3:
```
$ roslaunch aerial_robot_model aerial_robot_model.launch
```

- hydrusx:
```
$ roslaunch aerial_robot_model aerial_robot_model.launch model:=hydrusx/hydrus3_hex
```

- dragon2:
```
$ roslaunch aerial_robot_model aerial_robot_model.launch model:=dragon2
```
We have to change the "Fix Frame" in rviz.
