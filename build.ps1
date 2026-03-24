param(
    [string]$Target    = "build",
    [string]$Port      = "COM21",   
    [string]$Baud      = "115200",
    [string]$FlashTool = "C:\Users\Owner\Downloads\STM32L051 (1)\stm32flash\stm32flash.exe"
)

$ErrorActionPreference = "Stop"

# --- Project root ---
$ProjectRoot = $PSScriptRoot

# --- Auto-detect stm32flash ---
if (-not $FlashTool) {
    $cmd = Get-Command stm32flash.exe -ErrorAction SilentlyContinue
    if ($cmd) {
        $FlashTool = $cmd.Source
    } else {
        $candidates = @(
            "D:\stm32flash\stm32flash.exe",
            "C:\stm32flash\stm32flash.exe",
            "$ProjectRoot\stm32flash.exe",
            "$env:USERPROFILE\Downloads\stm32flash.exe"
        )
        $FlashTool = $candidates | Where-Object { $_ -and (Test-Path $_) } | Select-Object -First 1
    }
}

# --- Detect project name ---
$CmakeLists  = Get-Content "$ProjectRoot\CMakeLists.txt" -Raw
if ($CmakeLists -match 'set\s*\(\s*CMAKE_PROJECT_NAME\s+([\w_]+)') {
    $ProjectName = $Matches[1]
} else {
    throw "Cannot find CMAKE_PROJECT_NAME in CMakeLists.txt"
}

# --- Paths ---
$BuildDir = "$ProjectRoot\build"
$ElfFile  = "$BuildDir\$ProjectName.elf"
$BinFile  = "$BuildDir\$ProjectName.bin"

# --- Fix temp path ---
$env:TMP  = "$env:USERPROFILE\AppData\Local\Temp"
$env:TEMP = "$env:USERPROFILE\AppData\Local\Temp"

# --- Targets ---
switch ($Target) {

    "build" {
        if (-not (Test-Path $BuildDir)) {
            throw "build/ not found. Run cmake first."
        }

        Write-Host "[cmake] Building '$ProjectName'..." -ForegroundColor Cyan

        Push-Location $ProjectRoot
        cmake --build build
        $exitCode = $LASTEXITCODE
        Pop-Location

        if ($exitCode -ne 0) { throw "Build failed" }

        Write-Host "[objcopy] ELF -> BIN" -ForegroundColor Cyan
        arm-none-eabi-objcopy -O binary $ElfFile $BinFile

        if (-not (Test-Path $BinFile)) {
            throw "BIN generation failed"
        }

        Write-Host "[done] $BinFile" -ForegroundColor Green
    }

    "flash" {

        # --- Port check ---
        $availablePorts = [System.IO.Ports.SerialPort]::GetPortNames()

        if ($Port -notin $availablePorts) {
            Write-Host "[error] Port '$Port' not found." -ForegroundColor Red
            Write-Host "Available COM ports:" -ForegroundColor Yellow

            if ($availablePorts.Count -eq 0) {
                Write-Host "  (none detected)" -ForegroundColor DarkGray
            } else {
                ($availablePorts | Sort-Object) | ForEach-Object {
                    Write-Host "  $_" -ForegroundColor White
                }
            }
            exit 1
        }

        # --- File checks ---
        if (-not (Test-Path $BinFile)) {
            throw "$ProjectName.bin not found - run build first"
        }

        if (-not $FlashTool -or -not (Test-Path $FlashTool)) {
            throw "stm32flash not found. Pass -FlashTool path or install it."
        }

        Write-Host "[flash] Using: $FlashTool" -ForegroundColor Cyan
        Write-Host "[flash] Writing to $Port @ $Baud..." -ForegroundColor Cyan

        & $FlashTool -b $Baud -w $BinFile -v -g 0x08000000 $Port

        if ($LASTEXITCODE -ne 0) {
            throw "Flash failed"
        }

        Write-Host "[done] Flash successful 🚀" -ForegroundColor Green
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
        Write-Host "Valid targets: build, flash, clean"
    }
}