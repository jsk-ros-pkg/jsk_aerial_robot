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
```
