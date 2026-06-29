#Requires -Version 5.1
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

# ==============================
# User Configuration Variables
# ==============================
$MUJOCO_VERSION      = "3.10.0"
$MUJOCO_ZIP          = "mujoco-$MUJOCO_VERSION-windows-x86_64.zip"
$MUJOCO_URL          = "https://github.com/google-deepmind/mujoco/releases/download/$MUJOCO_VERSION/$MUJOCO_ZIP"
$WIN_RESOURCES_DIR   = "src\main\resources\mujoco\windows-x86_64"
$INSTALL_DIR         = "install\mujoco"

# ==============================
# Download MuJoCo SDK
# ==============================
New-Item -ItemType Directory -Force -Path build | Out-Null
$zipPath = "build\$MUJOCO_ZIP"
if (-not (Test-Path $zipPath)) {
    Write-Host "Downloading $MUJOCO_ZIP ..."
    Invoke-WebRequest -Uri $MUJOCO_URL -OutFile $zipPath
}

$extractRoot = "build\mujoco-extract"
Remove-Item -Recurse -Force $extractRoot -ErrorAction SilentlyContinue
Expand-Archive -Path $zipPath -DestinationPath $extractRoot

# This zip extracts flat (no versioned subfolder) — bin/, include/, lib/ are at the root.
$mujocoRoot = $extractRoot

# ==============================
# Stage Headers + Import Library
# ==============================
New-Item -ItemType Directory -Force -Path "$INSTALL_DIR\include" | Out-Null
New-Item -ItemType Directory -Force -Path "$INSTALL_DIR\lib"     | Out-Null

Copy-Item -Recurse -Force "$mujocoRoot\include\mujoco" "$INSTALL_DIR\include\"
# mujoco.lib is the MSVC import library used by the JavaCPP linker step in wrap.ps1
Copy-Item -Force "$mujocoRoot\lib\mujoco.lib" "$INSTALL_DIR\lib\"

# ==============================
# Copy Runtime DLL to Resources
# ==============================
New-Item -ItemType Directory -Force -Path $WIN_RESOURCES_DIR | Out-Null
Copy-Item -Force "$mujocoRoot\bin\mujoco.dll" $WIN_RESOURCES_DIR

Write-Host ""
Write-Host "Done. MuJoCo $MUJOCO_VERSION Windows SDK staged."
Write-Host "  Headers : $INSTALL_DIR\include\mujoco\"
Write-Host "  Lib     : $INSTALL_DIR\lib\mujoco.lib"
Write-Host "  DLL     : $WIN_RESOURCES_DIR\mujoco.dll"
Write-Host ""
Write-Host "Next step: run wrap.ps1 from a Visual Studio Developer PowerShell to build jniMujoco.dll"
