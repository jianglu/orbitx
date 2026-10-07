# Import MSVC x64 toolchain into the current PowerShell process (INCLUDE/LIB/PATH).
# Usage (from repo or orbitx/):
#   . .\tools\enter-msvc.ps1
# Then: cargo test -p orbitx-dynamics --test ffi_oracle

$ErrorActionPreference = 'Stop'

function Find-VcVars64 {
    $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
    if (Test-Path $vswhere) {
        $install = & $vswhere -latest -products * `
            -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 `
            -property installationPath 2>$null
        if ($install) {
            $candidate = Join-Path $install 'VC\Auxiliary\Build\vcvars64.bat'
            if (Test-Path $candidate) { return $candidate }
        }
    }
    $fallbacks = @(
        'D:\VisualStudio\2022\VC\Auxiliary\Build\vcvars64.bat',
        "${env:ProgramFiles}\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat",
        "${env:ProgramFiles}\Microsoft Visual Studio\2022\Professional\VC\Auxiliary\Build\vcvars64.bat",
        "${env:ProgramFiles}\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
    )
    foreach ($p in $fallbacks) {
        if (Test-Path $p) { return $p }
    }
    throw 'vcvars64.bat not found. Install VS 2022 with "Desktop development with C++".'
}

$vcvars = Find-VcVars64
cmd /c "`"$vcvars`" >nul && set" | ForEach-Object {
    if ($_ -match '^(.*?)=(.*)$') {
        [System.Environment]::SetEnvironmentVariable($matches[1], $matches[2], 'Process')
    }
}

$cl = Get-Command cl.exe -ErrorAction SilentlyContinue
if (-not $cl) {
    throw 'cl.exe still not on PATH after vcvars64.'
}
Write-Host "MSVC ready: $($cl.Source)"
Write-Host "INCLUDE set=$([bool]$env:INCLUDE)  LIB set=$([bool]$env:LIB)"
