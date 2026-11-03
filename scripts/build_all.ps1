param(
    [string]$BuildDir='build/win64',
    [ValidateSet('all','fixedwing','fixedwing_att')][string]$Runtime='all',
    [string]$Compiler='g++',
    [string]$MakeProgram='',
    [int]$Jobs=4
)
$ErrorActionPreference='Stop'
Push-Location (Split-Path -Parent $PSScriptRoot)
try {
    $iaCompiler=(Get-Command $Compiler -ErrorAction Stop).Source
    if(-not $MakeProgram) {
        $iaSibling=Join-Path (Split-Path -Parent $iaCompiler) 'mingw32-make.exe'
        if(Test-Path -LiteralPath $iaSibling) { $MakeProgram=$iaSibling }
        else { $MakeProgram=(Get-Command mingw32-make -ErrorAction Stop).Source }
    }
    $iaOld=if($Runtime -eq 'fixedwing_att') {'OFF'} else {'ON'}
    $iaAtt=if($Runtime -eq 'fixedwing') {'OFF'} else {'ON'}
    & cmake -S . -B $BuildDir -G 'MinGW Makefiles' '-DCMAKE_BUILD_TYPE=Release' `
        "-DCMAKE_CXX_COMPILER=$iaCompiler" "-DCMAKE_MAKE_PROGRAM=$MakeProgram" `
        "-DAVANTI_BUILD_FIXEDWING=$iaOld" "-DAVANTI_BUILD_FIXEDWING_ATT=$iaAtt" `
        '-DAVANTI_BUILD_TESTS=ON' '-DAVANTI_STAGE_LIBS=ON'
    if($LASTEXITCODE -ne 0) {throw 'CMake configure failed'}
    & cmake --build $BuildDir --parallel $Jobs
    if($LASTEXITCODE -ne 0) {throw 'CMake build failed'}
    & ctest --test-dir $BuildDir --output-on-failure
    if($LASTEXITCODE -ne 0) {throw 'CTest failed'}
    if($Runtime -ne 'fixedwing') {
        $iaSmokeArgs=@('--library',(Join-Path $BuildDir 'dist/avanti2_att.dll'))
        if($Runtime -eq 'fixedwing_att') { $iaSmokeArgs+='--skip-old-runtime' }
        & python tests/regression/test_att_runtime.py @iaSmokeArgs
        if($LASTEXITCODE -ne 0) {throw 'Attitude runtime smoke failed'}
    }
} finally { Pop-Location }
