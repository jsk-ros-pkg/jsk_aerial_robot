#!/bin/bash

if [ "`which gzsdf`" = "" ]; then
    GZSDF="gz sdf -p"
else
    GZSDF="gzsdf print"
fi
rm -rf meshes model.sdf model.urdf dropping_zone_box.dae
roseus box-to-collada.l
sed -i -e 's/nil/box/g' dropping_zone_box.dae
rosrun collada_urdf collada_to_urdf dropping_zone_box.dae -G -A --mesh_output_dir meshes --mesh_prefix "model://dropping_zone_box/meshes" -O model.urdf
$GZSDF model.urdf > model.sdf
sed -i -e 's/<mass>.*<\/mass>/<mass>20<\/mass>/g' model.sdf
sed -i -e 's@<ixx>.*</ixx>@<ixx>1</ixx>@g' model.sdf
sed -i -e 's@<iyy>.*</iyy>@<iyy>1</iyy>@g' model.sdf
sed -i -e 's@<izz>.*</izz>@<izz>1</izz>@g' model.sdf
