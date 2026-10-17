param([string]$Python = 'python')

$ErrorActionPreference = 'Stop'
$toolRoot = $PSScriptRoot
$buildRoot = Join-Path $toolRoot 'build'
$releaseRoot = Join-Path $buildRoot 'release'
$specPath = Join-Path $toolRoot 'DAPLinkFlasher.spec'
& $Python -B -X utf8 -m PyInstaller --noconfirm --distpath $releaseRoot --workpath $buildRoot $specPath
if ($LASTEXITCODE -ne 0) { throw 'DAPLink Flasher build failed; the existing EXE was kept.' }
Write-Output ('Built candidate: ' + (Join-Path $releaseRoot 'DAPLinkFlasher.exe'))
Write-Output 'Check with tools/tests/smoke_dap_flasher_exe.py before replacing tools/DAPLinkFlasher.exe.'
