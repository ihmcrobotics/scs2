param(
   [string] $MujocoVersion = "3.2.7",
   [string] $MujocoSdkPath = ""
)

$ErrorActionPreference = "Stop"

$mujocoArchive = "mujoco-$MujocoVersion-windows-x86_64.zip"
$mujocoUrl = "https://github.com/google-deepmind/mujoco/releases/download/$MujocoVersion/$mujocoArchive"
$resourcesDir = "src/main/resources/mujoco/windows-x86_64"
$installDir = "install/mujoco"

New-Item -ItemType Directory -Force -Path "build" | Out-Null

if ([string]::IsNullOrWhiteSpace($MujocoSdkPath))
{
   $localSdkPath = "mujoco-$MujocoVersion-windows-x86_64"
   $buildSdkPath = "build/mujoco-$MujocoVersion-windows-x86_64"

   if (Test-Path $localSdkPath)
   {
      $MujocoSdkPath = $localSdkPath
   }
   else
   {
      $archivePath = Join-Path "build" $mujocoArchive
      if (!(Test-Path $archivePath))
      {
         Invoke-WebRequest -Uri $mujocoUrl -OutFile $archivePath
      }

      if (Test-Path $buildSdkPath)
      {
         Remove-Item -Recurse -Force $buildSdkPath
      }
      Expand-Archive -Path $archivePath -DestinationPath "build" -Force
      if (!(Test-Path $buildSdkPath))
      {
         $alternateBuildSdkPath = "build/mujoco-$MujocoVersion"
         if (Test-Path $alternateBuildSdkPath)
         {
            $buildSdkPath = $alternateBuildSdkPath
         }
      }
      $MujocoSdkPath = $buildSdkPath
   }
}

if (!(Test-Path (Join-Path $MujocoSdkPath "include/mujoco")))
{
   throw "MuJoCo SDK include directory not found under '$MujocoSdkPath'."
}
if (!(Test-Path (Join-Path $MujocoSdkPath "lib/mujoco.lib")))
{
   throw "MuJoCo import library not found under '$MujocoSdkPath/lib'."
}
if (!(Test-Path (Join-Path $MujocoSdkPath "bin/mujoco.dll")))
{
   throw "MuJoCo runtime DLL not found under '$MujocoSdkPath/bin'."
}

New-Item -ItemType Directory -Force -Path "$installDir/include", "$installDir/lib", $resourcesDir | Out-Null

Copy-Item -Recurse -Force (Join-Path $MujocoSdkPath "include/mujoco") "$installDir/include/"
Copy-Item -Force (Join-Path $MujocoSdkPath "lib/mujoco.lib") "$installDir/lib/"
Copy-Item -Force (Join-Path $MujocoSdkPath "bin/mujoco.dll") "$resourcesDir/"

if (Test-Path (Join-Path $MujocoSdkPath "bin/mujoco_plugin"))
{
   Copy-Item -Recurse -Force (Join-Path $MujocoSdkPath "bin/mujoco_plugin") "$resourcesDir/"
}
