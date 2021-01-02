---
layout: default
---


## 1. Install bloom (if not installed)
Run `$ sudo apt install python-bloom`

## 2. Checkout to master branch (if not on master branch)
This operation should be done in master branch because the tag should be created in the original repository.
**Note: The commands refers `origin`, so please set `origin` remote to https://github.com/tongtybj/aerial_robot**

## 3. Generate CHANGELOG.rst
Run `$ catkin_generate_changelog` at root (aerial_robot/)
Then, you can edit each CHANGELOG.rst. Now, you shouldn't erase "forthcoming" section. It will be modified the after step.
After that, please `git add` all CHANGELOG.rst, then `git commit`.

**Note1: CHANGELOG.rst MUST NOT contain any 全角文字(ひらがな，カタカナ，漢字), so please use only English for commit message. In your .gitconfig, please set your username in English.**

## 4. Update package.xml
Run `$ catkin_prepare_release --bump {major,minor,patch}` at root (aerial_robot/).
By this command, the version is updated and your commit is pushed.
You should select option `major,minor,patch` properly. 
Ex., if you fixed a small bug, you should run `$ catkin_prepare_release --bump patch`

## Policy
version x.y.zとして<br>
x: リポジトリの構造の変更レベルの大変更<br>
y: 重要なバグフィックス，互換性のない変更(APIの名前，仕様の変更など）<br>
z: 互換性のある変更（単純な機能追加など）<br>
その他(タグを切らない): 些細な変更(スペルミスなど)

## Reference
http://wiki.ros.org/ja/bloom/Tutorials/ReleaseCatkinPackage