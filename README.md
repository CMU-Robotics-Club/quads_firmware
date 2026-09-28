# Setup for Mac (Windows and linux coming)

Run:
```
brew install --cask gcc-arm-embedded
brew install cmake ninja
brew install stlink openocd
brew install llvm
```

While that's installing, get the [STM32Cube App](https://www.st.com/en/development-tools/stm32cubemx.html) 

#### For an existing project (DO THIS ONE)

Run the following:

```
mkdir -p ~/Documents/QuadsSTMFirmware && cd ~/Documents/QuadsSTMFirmware/
git clone https://github.com/CMU-Robotics-Club/quads_firmware .
```

Next, open up your project by selecting your QuadsSTMFirmware folder in the STM32Cube App.

#### For a new project: 

Use QuadsSTMFirmware for the project name, and select STM32H753ZIT6 as the MCU (SUBJECT TO CHANGE). Select your home directory's Documents folder for saving for compatibility with the aliases we'll make later.

### Compiling and more setup

Once both those steps are complete, go to the project directory in your terminal. That would be in the QuadsSTMFirmware directory wherever you put the project. Run the following:

```
cmake -B build -G Ninja -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
ln -s build/compile_commands.json .
```

Next, open up your `.zshrc` or `.bashrc` file and add the following aliases:

```
alias makestm="cmake -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON && cmake --build build"
alias flashstm="arm-none-eabi-objcopy -O binary build/QuadsSTMFirmware.elf build/QuadsSTMFirmware.bin && st-flash --reset write build/QuadsSTMFirmware.bin 0x08000000"
```

Resource/restart your terminal. Then, in the project's root dir, run `makestm`, which should compile the project. Then with the STM connected to your computer with USB, run `flashstm`, and that's it!

Remember to always run `makestm` to update your compilations before running `flashstm`.
