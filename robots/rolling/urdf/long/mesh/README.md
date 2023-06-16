```
ls *.STL | while read f; do meshlabserver -i $f -o `basename $f .STL`.stl -s filter.mlx -om fc; done
```