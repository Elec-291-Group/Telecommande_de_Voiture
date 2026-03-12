# =============================================================================
# build.ps1 - Build & flash script for STM32CubeMX CMake projects
# Usage:
#   .\build.ps1 build                    # make + elf->bin
#   .\build.ps1 flash -Port COM3         # flash existing .bin
#   .\build.ps1 clean                    # clean build directory
#
# NOTE: Run cmake once manually from Git Bash before first build:
#   cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
# =============================================================================

param(
    [string]$Target    = "build",
    [string]$Port      = "COM12",
    [string]$Baud      = "115200",
    [string]$FlashTool = "D:\stm32flash\stm32flash.exe"
)

$ErrorActionPreference = "Stop"

# --- Auto-detect project name from CMakeLists.txt ----------------------------
$ProjectRoot = $PSScriptRoot
$CmakeLists  = Get-Content "$ProjectRoot\CMakeLists.txt" -Raw
if ($CmakeLists -match 'set\s*\(\s*CMAKE_PROJECT_NAME\s+([\w_]+)') {
    $ProjectName = $Matches[1]
} else {
    throw "Cannot find CMAKE_PROJECT_NAME in CMakeLists.txt"
}

# --- Derived paths -----------------------------------------------------------
$BuildDir = "$ProjectRoot\build"
$ElfFile  = "$BuildDir\$ProjectName.elf"
$BinFile  = "$BuildDir\$ProjectName.bin"

# --- Auto-find make.exe ------------------------------------------------------
$Make = (Get-Command make -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Source)
if (-not $Make) {
    $candidates = @(
        "D:\MSYS2\usr\bin\make.exe", "D:\Git\usr\bin\make.exe",
        "C:\MSYS2\usr\bin\make.exe", "C:\msys64\usr\bin\make.exe"
    )
    $Make = $candidates | Where-Object { Test-Path $_ } | Select-Object -First 1
}
if (-not $Make) { throw "make.exe not found. Add MSYS2 usr/bin to PATH." }

# --- Fix MinGW linker temp file issue ----------------------------------------
$env:TMP  = "$env:USERPROFILE\AppData\Local\Temp"
$env:TEMP = "$env:USERPROFILE\AppData\Local\Temp"

# --- Targets -----------------------------------------------------------------
switch ($Target) {

    "build" {
        if (-not (Test-Path "$BuildDir\Makefile")) {
            throw "build/Makefile not found. Run cmake first from Git Bash:`n  cmake -B build -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake"
        }
        Write-Host "[make] Building '$ProjectName'..." -ForegroundColor Cyan
        & $Make -C $BuildDir
        if ($LASTEXITCODE -ne 0) { throw "make failed" }

        Write-Host "[objcopy] $ProjectName.elf -> $ProjectName.bin" -ForegroundColor Cyan
        arm-none-eabi-objcopy -O binary $ElfFile $BinFile
        Write-Host "[done] $BinFile" -ForegroundColor Green
    }

    "flash" {
        if (-not (Test-Path $BinFile))   { throw "$ProjectName.bin not found - run build first" }
        if (-not (Test-Path $FlashTool)) { throw "stm32flash not found at: $FlashTool" }
        Write-Host "[flash] Writing to $Port at $Baud baud..." -ForegroundColor Cyan
        & $FlashTool -b $Baud -w $BinFile -v -g 0x08000000 $Port
        if ($LASTEXITCODE -ne 0) { throw "Flash failed" }
        Write-Host "[done] Flash successful" -ForegroundColor Green
    }

    "clean" {
        if (Test-Path $BuildDir) {
            Remove-Item -Recurse -Force $BuildDir
            Write-Host "[done] build directory removed" -ForegroundColor Green
        } else {
            Write-Host "[skip] build directory does not exist" -ForegroundColor Yellow
        }
    }

    default {
        Write-Host "Unknown target: $Target"
        Write-Host "Valid targets: build (default), flash, clean"
    }
}
