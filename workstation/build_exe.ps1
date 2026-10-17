param([string]$Python = 'python')

$ErrorActionPreference = 'Stop'
$consoleRoot = $PSScriptRoot
$distPath = Join-Path $consoleRoot 'dist'
$buildPath = Join-Path $consoleRoot 'build'
$specPath = Join-Path $consoleRoot 'RPBotStudio.spec'

& $Python -X utf8 -m PyInstaller --noconfirm --distpath $distPath --workpath $buildPath $specPath
if ($LASTEXITCODE -ne 0) { throw 'RPBot Studio EXE build failed.' }
$exePath = Join-Path $distPath 'RPBotStudio.exe'
$exeFile = Get-Item -LiteralPath $exePath
Write-Output ('Built {0} ({1:N1} MiB)' -f $exeFile.FullName, ($exeFile.Length / 1MB))
Get-FileHash -LiteralPath $exePath -Algorithm SHA256
