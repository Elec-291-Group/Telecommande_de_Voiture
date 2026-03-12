# Build & Flash

Requires: MSYS2 MINGW64 with `arm-none-eabi` toolchain, CMake, and `stm32flash.exe`.

**First time:** run cmake from Git Bash once to configure the build directory:
```bash
mkdir build && cd build && cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/gcc-arm-none-eabi.cmake
```

**Build / Flash (PowerShell):**
```powershell
.\build.ps1              # build and generate .bin
.\build.ps1 flash        # build + flash via UART bootloader (BOOT0=1) on COM3
.\build.ps1 flash -Port COM5 -FlashTool "D:\path\to\stm32flash.exe"
.\build.ps1 clean
```
