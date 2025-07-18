# How to contribute

## Contribute Process

1. fork the original repository from https://github.com/jsk-ros-pkg/jsk_aerial_robot
2. develop your own feature in your forked repository
3. once you finish your development, make a pull request to the original repository
4. the original repository maintainer will review your pull request and merge it if it is OK

## Code Style

We use Clang-format to format the C++ code, and use black to format the Python code.
Pre-commit, the format plugin, will run automatically before each commit. The specific function has been written in
.pre-commit-config.yaml. To activate this plugin, please run the following command in terminal and root directory:

```bash
    pip3 install pre-commit
    pre-commit install
    pre-commit run --files $(find path/to/folder -type f)
```

To turn off the pre-commit check locally, run:
```bash
    pre-commit uninstall
```

If you don't like the style formatted by black for Python, please add the following line to the top of your Python file:

```python
# fmt: off
# do your code here
# fmt: on
```
This will disable the black formatting for this file, and you can format it manually.

If you don't like the style formated by clang-format for C++, please add the following line to the top of your C++ file:

```cpp
// clang-format off
// do your code here
// clang-format on
```
This will disable the clang-format formatting for this file, and you can format it manually.
