# Setup for Mac and linux (Windows coming)

Run the following for mac:
```
brew install --cask gcc-arm-embedded
brew install cmake ninja
brew install stlink openocd
brew install llvm
```
or this for linux:
```
sudo apt update
sudo apt install -y \
  cmake \
  ninja-build \
  stlink-tools \
  openocd \
  llvm \
  clang \
  gcc-arm-none-eabi \
  libnewlib-arm-none-eabi \
  gdb-arm-none-eabi \
  binutils-arm-none-eabi \
  gdb-multiarch
sudo ln -s /usr/bin/gdb-multiarch /usr/local/bin/arm-none-eabi-gdb

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

Use QuadsSTMFirmware for the project name, and select STM32H723ZGT6 as the MCU (SUBJECT TO CHANGE). Select your home directory's Documents folder for saving for compatibility with the aliases we'll make later.

Once the project has been created, go to Project Manager -> Project -> Toolchain/IDE Select in the dropdown CMake.

### Compiling and more setup

Once both those steps are complete, go to the project directory in your terminal. That would be in the QuadsSTMFirmware directory wherever you put the project. Run the following:

```
cmake -B build -G Ninja -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
ln -s build/compile_commands.json .
```

Next, open up your `.zshrc` or `.bashrc` file and add the following aliases:

```
alias ocd="openocd \
  -f interface/stlink-dap.cfg \
  -c 'transport select dapdirect_swd' \
  -f target/stm32h7x.cfg \
  -c 'init' \
  -c 'stm32h7x.swo configure -protocol uart -traceclk 275000000 -pin-freq 2500000 -output /dev/stdout' \
  -c 'stm32h7x.swo enable' \
  -c 'itm ports on' \
  | python3 -u -c '
import sys
while True:
    header = sys.stdin.buffer.read(1)
    if not header:
        break
    if header[0] == 0x01:
        char = sys.stdin.buffer.read(1)
        if char:
            sys.stdout.buffer.write(char)
            sys.stdout.flush()'"
alias flashstm="makestm && arm-none-eabi-gdb --batch build/QuadsSTMFirmware.elf \
  -ex 'target extended-remote :3333' \
  -ex 'load' \
  -ex 'monitor reset run' \
  -ex 'quit'"
alias debugstm="arm-none-eabi-gdb build/QuadsSTMFirmware.elf \
  -ex 'target extended-remote :3333' \
  -ex 'monitor reset halt'"

```

Resource/restart your terminal. Then, in the project's root dir, with the STM connected to your computer with USB, run `flashstm`, and that's it!

### Debugging

To debug our code, we will use the `gdb` debugger. Follow the below instructions to start the debugger. If you have any questions on what to do with it, search up GDB's documentation. 

Before you start debugging, ensure the following:
1. You have successfully flashed your code onto the MCU using `flashstm`
2. Your device is currently connected to the board

Open two terminal sessions.

Terminal 1: Run `ocd`, and leave it running in the background.
> This will start a `gdb` server on port 3333 that translates our GDB commands into ST-Link operations (allowing us to communicate with the MCU).

Terminal 2: Run the following command:

```
debugstm
```

> `gdb-multiarch` and `arm-none-eabi-gdb` allows us to use GDB for different systems.

This will connect to the GDB server and begin debugging at the start of your `main` function.

Basic GDB usage:
- Set a breakpoint: `b <line #>` or `break <line #>`
- Set a breakpoint: `b <function>` or `break <function>`
- Delete a breakpoint: `d <breakpoint #>` or `delete <breakpoint #>`
- Step over: `n` or `next`
- Continue: `c` or `continue
- Print variable: `p <variable name>` or `print <variable name>`
  - When printing, you may need to type "up" to go up the call stack to go to the main where your variables are defined and in-scope
- Track variable: `display <variable name>`
  - Prints the value of a variable each time you hit a breakpoint or you step over to another line
