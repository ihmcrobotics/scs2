param(
   [string] $JavaCppVersion = "1.5.11"
)

$ErrorActionPreference = "Stop"

$windowsResourcesDir = "src/main/resources/mujoco/windows-x86_64"
$generatedJavaDir = "src/main/generated-java/us/ihmc/scs2/simulation/mujoco"
$cxxReleaseFlags = "/O2 /EHsc /MD /LD /std:c++17 /Zc:__cplusplus"

if (!(Get-Command java -ErrorAction SilentlyContinue))
{
   throw "java was not found on PATH."
}
if (!(Get-Command cl.exe -ErrorAction SilentlyContinue))
{
   throw "cl.exe was not found on PATH. Run this from a Visual Studio Developer PowerShell."
}
if ($env:VSCMD_ARG_TGT_ARCH -and $env:VSCMD_ARG_TGT_ARCH -ne "x64")
{
   throw "The active Visual Studio toolchain targets '$env:VSCMD_ARG_TGT_ARCH'. Open an x64 Visual Studio Developer PowerShell."
}
if (!(Test-Path "install/mujoco/include/mujoco"))
{
   throw "MuJoCo headers are missing. Run ./install.ps1 first."
}
if (!(Test-Path "install/mujoco/lib/mujoco.lib"))
{
   throw "MuJoCo import library is missing. Run ./install.ps1 first."
}

New-Item -ItemType Directory -Force -Path "build/java" | Out-Null
Copy-Item -Recurse -Force -Path "src/main/java/*" -Destination "build/java/"

$mujocoIncludePath = (Resolve-Path "install/mujoco/include").Path.Replace("\", "/")
$mujocoLibPath = (Resolve-Path "install/mujoco/lib").Path.Replace("\", "/")
$presetPath = "build/java/us/ihmc/scs2/simulation/mujoco/preset/MujocoInfoMapper.java"
$presetDir = Split-Path -Parent $presetPath
Copy-Item -Recurse -Force -Path "install/mujoco/include/mujoco" -Destination "build/java/"
Copy-Item -Recurse -Force -Path "install/mujoco/include/mujoco" -Destination $presetDir
$presetContent = Get-Content -Raw $presetPath
$presetContent = $presetContent.Replace("../../install/mujoco/include", ".")
$presetContent = $presetContent.Replace("../../install/mujoco/lib", $mujocoLibPath)
$presetContent = $presetContent.Replace('"mujoco/', '"./mujoco/')
Set-Content -Path $presetPath -Value $presetContent -NoNewline

Push-Location "build/java"
try
{
   if (!(Test-Path "javacpp.jar"))
   {
      Invoke-WebRequest `
         -Uri "https://github.com/bytedeco/javacpp/releases/download/$JavaCppVersion/javacpp-platform-$JavaCppVersion-bin.zip" `
         -OutFile "javacpp.zip"
      Expand-Archive -Path "javacpp.zip" -DestinationPath "." -Force
      $jar = Get-ChildItem -Recurse -Filter "javacpp.jar" | Select-Object -First 1
      if ($null -eq $jar)
      {
         throw "javacpp.jar was not found in the downloaded archive."
      }
      Copy-Item -Force $jar.FullName "javacpp.jar"
   }

   & java -jar javacpp.jar `
      -properties windows-x86_64 `
      "-Dplatform.includepath=$mujocoIncludePath" `
      "-Dplatform.linkpath=$mujocoLibPath" `
      us/ihmc/scs2/simulation/mujoco/preset/MujocoInfoMapper.java
   if ($LASTEXITCODE -ne 0) { throw "JavaCPP preset parsing failed." }

   & java -jar javacpp.jar `
      -properties windows-x86_64 `
      us/ihmc/scs2/simulation/mujoco/Mujoco.java `
      -d "../../$windowsResourcesDir" `
      "-Dplatform.includepath=$mujocoIncludePath" `
      "-Dplatform.linkpath=$mujocoLibPath" `
      "-Dplatform.compiler=cl.exe" `
      "-Dplatform.compiler.default=$cxxReleaseFlags"
   if ($LASTEXITCODE -ne 0) { throw "JavaCPP JNI build failed." }
}
finally
{
   Pop-Location
}

Remove-Item -Force `
   "$windowsResourcesDir/*.obj", `
   "$windowsResourcesDir/*.lib", `
   "$windowsResourcesDir/*.exp", `
   "$windowsResourcesDir/*.cpp" `
   -ErrorAction SilentlyContinue

New-Item -ItemType Directory -Force -Path $generatedJavaDir | Out-Null
Remove-Item -Force "$generatedJavaDir/Mujoco.java" -ErrorAction SilentlyContinue
Copy-Item -Force "build/java/us/ihmc/scs2/simulation/mujoco/Mujoco.java" "$generatedJavaDir/"
