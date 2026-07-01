#Requires -Version 5.1
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

# ==============================
# User Configuration Variables
# ==============================
$JAVACPP_VERSION     = "1.5.11"
$WIN_RESOURCES_DIR   = "src\main\resources\mujoco\windows-x86_64"
$GENERATED_JAVA_DIR  = "src\main\generated-java\us\ihmc\scs2\simulation\mujoco"

# Module root is the parent of native-build/
$moduleRoot  = Split-Path $PSScriptRoot

# ==============================
# Verify cl.exe is on PATH
# (run this script from a "Developer PowerShell for VS 20xx" or after calling vcvarsall.bat)
# ==============================
if (-not (Get-Command cl.exe -ErrorAction SilentlyContinue)) {
    Write-Error @"
cl.exe not found on PATH.
Run this script from a 'Developer PowerShell for VS 20xx' (search the Start menu), or first run:
    cmd /c "`"C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat`" && powershell"
"@
    exit 1
}

$buildJava   = "$PSScriptRoot\build\java"
$javacppJar  = "$buildJava\javacpp.jar"
$resourcesAbs = "$moduleRoot\$WIN_RESOURCES_DIR"

# ==============================
# Stage Java Sources for JavaCPP
# ==============================
New-Item -ItemType Directory -Force -Path $buildJava | Out-Null
Copy-Item -Recurse -Force "$moduleRoot\src\main\java\*" $buildJava

# ==============================
# Download javacpp.jar
# ==============================
if (-not (Test-Path $javacppJar)) {
    $javacppZipUrl = "https://github.com/bytedeco/javacpp/releases/download/$JAVACPP_VERSION/javacpp-platform-$JAVACPP_VERSION-bin.zip"
    $javacppZip = "$buildJava\javacpp.zip"
    Write-Host "Downloading javacpp-platform-$JAVACPP_VERSION-bin.zip ..."
    Invoke-WebRequest -Uri $javacppZipUrl -OutFile $javacppZip
    Add-Type -AssemblyName System.IO.Compression.FileSystem
    $zip = [System.IO.Compression.ZipFile]::OpenRead($javacppZip)
    $entry = $zip.Entries | Where-Object { $_.Name -eq "javacpp.jar" } | Select-Object -First 1
    if ($null -eq $entry) { throw "javacpp.jar not found inside the zip." }
    [System.IO.Compression.ZipFileExtensions]::ExtractToFile($entry, $javacppJar, $true)
    $zip.Dispose()
}

# ==============================
# Step 1: Parse the preset -> emit Mujoco.java
# JavaCPP resolves source paths relative to the process CWD, so use cmd to set it.
# ==============================
Write-Host "Generating Mujoco.java from preset ..."
cmd /c "cd /d `"$buildJava`" && java -jar javacpp.jar us/ihmc/scs2/simulation/mujoco/preset/MujocoInfoMapper.java"
if ($LASTEXITCODE -ne 0) { throw "JavaCPP preset parsing failed." }

# ==============================
# Step 2: Compile JNI shim -> jniMujoco.dll
# LINK=/DLL is required: JavaCPP's MSVC command does not pass /DLL to the linker by default;
# the MSVC linker reads the LINK env var as additional flags.
# ==============================
New-Item -ItemType Directory -Force -Path $resourcesAbs | Out-Null
Write-Host "Compiling jniMujoco.dll ..."
cmd /c "cd /d `"$buildJava`" && set LINK=/DLL && java -jar javacpp.jar us/ihmc/scs2/simulation/mujoco/Mujoco.java -d `"$resourcesAbs`" -Dplatform.compiler=cl `"-Dplatform.compiler.default=/O2 /std:c++17 /EHsc /MD`""
if ($LASTEXITCODE -ne 0) { throw "JavaCPP JNI compilation failed." }

# Remove MSVC build artifacts that land alongside the DLL
@("jniMujoco.obj","jniMujoco.exp","jniMujoco.lib","jnijavacpp.obj") | ForEach-Object {
    Remove-Item "$resourcesAbs\$_" -ErrorAction SilentlyContinue
}

# ==============================
# Copy Generated Java to generated-java source set
# ==============================
New-Item -ItemType Directory -Force -Path "$moduleRoot\$GENERATED_JAVA_DIR" | Out-Null
Copy-Item -Force "$buildJava\us\ihmc\scs2\simulation\mujoco\Mujoco.java" "$moduleRoot\$GENERATED_JAVA_DIR\"

Write-Host ""
Write-Host "Done."
Write-Host "  JNI DLL     : $WIN_RESOURCES_DIR\jniMujoco.dll"
Write-Host "  Mujoco.java : $GENERATED_JAVA_DIR\Mujoco.java"
