param(
    [Parameter(Mandatory=$true)][string]$ZigExe,
    [string]$BuildDir='build/linux64',
    [ValidateSet('all','fixedwing','fixedwing_att')][string]$Runtime='all',
    [string]$MakeProgram='mingw32-make',
    [int]$Jobs=2
)
# Windows -> Linux x86_64, glibc <= 2.35 (Ubuntu 22.04). No Linux execution here.
$ErrorActionPreference='Stop'
Push-Location (Split-Path -Parent $PSScriptRoot)
try {
    $iaZig=(Resolve-Path -LiteralPath $ZigExe).Path
    $iaMake=(Get-Command $MakeProgram -ErrorAction Stop).Source
    $iaBuild=[IO.Path]::GetFullPath($BuildDir)
    New-Item -ItemType Directory -Path $iaBuild -Force | Out-Null
    $iaWrapper=Join-Path $iaBuild 'zig-cxx.cmd'
    $iaPython=(Get-Command python -ErrorAction Stop).Source
    $iaDriver=Join-Path $PSScriptRoot 'zig_cxx_driver.py'
    Set-Content -LiteralPath $iaWrapper -Value ('@"'+$iaPython+'" "'+$iaDriver+'" "'+$iaZig+'" %*') -Encoding ASCII
    $iaPriorCache=$env:ZIG_GLOBAL_CACHE_DIR
    $env:ZIG_GLOBAL_CACHE_DIR=Join-Path $iaBuild 'zig-cache'
    try {
        $iaOld=if($Runtime -eq 'fixedwing_att') {'OFF'} else {'ON'}
        $iaAtt=if($Runtime -eq 'fixedwing') {'OFF'} else {'ON'}
        & cmake -S . -B $iaBuild -G 'MinGW Makefiles' '-DCMAKE_SYSTEM_NAME=Linux' `
            '-DCMAKE_SYSTEM_PROCESSOR=x86_64' "-DCMAKE_CXX_COMPILER=$iaWrapper" `
            "-DCMAKE_MAKE_PROGRAM=$iaMake" '-DCMAKE_BUILD_TYPE=Release' `
            "-DAVANTI_BUILD_FIXEDWING=$iaOld" "-DAVANTI_BUILD_FIXEDWING_ATT=$iaAtt" `
            '-DAVANTI_BUILD_TESTS=ON' '-DAVANTI_STAGE_LIBS=ON'
        if($LASTEXITCODE -ne 0) {throw 'Linux cross configure failed'}
        & cmake --build $iaBuild --parallel $Jobs
        if($LASTEXITCODE -ne 0) {throw 'Linux cross build failed'}
        Write-Output 'ELF libraries built. Run dist ABI executables and ctypes smoke on Ubuntu; cross CMake caches are host-specific.'
    } finally { $env:ZIG_GLOBAL_CACHE_DIR=$iaPriorCache }
} finally { Pop-Location }
