# How to contribute

## Contribute Process

1. Fork the original repository from https://github.com/jsk-ros-pkg/jsk_aerial_robot
2. Develop your own feature in your forked repository
3. Once you finish your development, make a pull request to the original repository
4. The original repository maintainer will review your pull request and merge it if it is okay

## Code Style

We use the [clang-format](https://clang.llvm.org/docs/ClangFormat.html) to format C++ code, and use [black](https://github.com/psf/black) to format Python code. 
Pre-commit, the format plugin, will run automatically before each commit. The specific function has been written in 
.pre-commit-config.yaml. To activate this plugin, please run the following command in terminal and root directory:

```bash
    pip3 install pre-commit
    pre-commit install
```
