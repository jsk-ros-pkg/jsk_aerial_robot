---
layout: default
title: MCU
---

# Beginner Level
## SDK: True Studio
### Install:
Linux/Windows: https://atollic.com/resources/download/

### After Install:
1. Run SDK:
   1. Windows: nothing to say.
   2. Linux: 
       1. general way: `$ cd /opt/Atollic_TrueSTUDIO_for_ARM_x86_64_x.x.x/ide/ & ./TrueSTUDIO`
       2. another way: Add `/opt/Atollic_TrueSTUDIO_for_ARM_x86_64_x.x.x/ide/` to $PATH; move to your project directory where `.project` exists, and run `$ TrueSTUDIO .project`.
 
~~2. Emacs Key-bindings~~
   ~~1. Select "Help" -> "Install New Software", add following urls:~~
      ~~- http://www.mulgasoft.com/emacsplus/e4/update-site~~
      ~~- http://www.mulgasoft.com/emacsplus/update-site~~
   ~~2. change to emacs keybindings~~
      ~~"Window" -> "Reference" -> "General" -> "Keys" -> "Scheme". Select "Emacs+ Scheme"~~

### Tips:
1. General compile errors:
    1. `xxxx.su can not find`: Delete the "Debug/XXXX" directory in the explorer

# Intermediate Level
## Project Generator: STM32CubeMX 

### Install:
Linux/Windows: http://www.st.com/ja/development-tools/stm32cubemx.html#getsoftware-scroll
- Windows: nothing to say
- Linux: run `$ ./SetupSTM32CubeMX-x.xx.x.linux` after extraction. Then run `$ cd ~/STM32CubeMX & ./STM32CubeMX`

### Tips:
1. Stable version for d_board: Right now we only can use ```STM32Cube FW_F7 V1.3.1```. The later version can not work with CAN system.
2. Delete ``` main.c ``` from the ``` Users``` dir in TrueStudio
3. Linux: After generate project from STM32CubeMX in Ubuntu, please revert file: ```.project``` and ```.cproject```.
```$ git checkout -- .cproject  .project```  

## SDK: TrueStudio

### Initialization  Process for first project building
**note**: `.cproject` is modified from TrueStudio, which has no business with STM32CubeMX. But, `.project` is influenced by STM32CubeMX.
1. ADD line: `"<nature>org.eclipse.cdt.core.ccnature</nature>"` in `.project` to enable cpp files compile; 
2. Change the `<location>` to the relative path in .project, if necessary.
2. Add source files(*.c, *.cpp) to project: drag the related folders directly into the "Application" in the Project Explorer. (e.g. "rosserial_stm/test/ros_lib", "Jsk_Lib"). Also note that please check to the "link to files and folders"
3. Copy main.c to main.cpp and also drag into the `"Application"/"User"` in the Project Explorer. 
    Delete mian.c in the Project Explorer.  Or you can directly edit `.project` to change from `main.c` to `main.cpp`
4. Edit the properties of the project: right click the project in the Project Explorer, select "Properties". Select "C/C++ Build" / "Settings". 
   1. change to `arm-atollic-eabi-g++ -c` in "Assembler" and  "C++ Compiler", please **remain** `arm-atollic-eabi-gcc -c` for "C Compiler", and change to `arm-atollic-eabi-g++` for  "C++ linker". Please read [this](http://kaworu.jpn.org/cpp/g++) for the meaning of option `-c`.
   2. Change to "gnu++11(C++11 + nug extensions)" for `C++ Comipler/General`.
   3. Add related symbols to "Symbols" of  "C Compiler",  (e.g. __USE_C99_MATH, __arm). Copy all the symbols to "C++ Compiler". Also note that do not add "__cplus_cplus" which is the default symbols when running `g++`.
   4. add include path in "Directories" of  "C Compiler".  (e.g.: ../../../ros_lib/rosseial_stm/test/ros_lib, ../../../Jsk_Lib). Copy **all** the paths to "C++ Compiler". 
   5. change the optimize for debugging (-Og) in `Optimization` 
   6. Can also add lib in "C++ Linker", please check "jsk_ugv"
   7. Please use `extern C` to avoid the conflict C funtion in C++ files. Please read [this](https://aki-yam.hatenablog.com/entry/20081019/1224386522) carefully.



