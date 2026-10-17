$ErrorActionPreference = 'Stop'
$consoleRoot = $PSScriptRoot
$pythonPath = (& python -c 'import sys; print(sys.executable)').Trim()
if ($LASTEXITCODE -ne 0) { throw 'Python 3 with tkinter is required.' }
$pythonwPath = Join-Path (Split-Path -Parent $pythonPath) 'pythonw.exe'
if (-not (Test-Path -LiteralPath $pythonwPath)) { throw "Missing $pythonwPath" }
$desktopPath = [Environment]::GetFolderPath('Desktop')
$shortcutPath = Join-Path $desktopPath 'RPBot Console.lnk'
$shellObject = New-Object -ComObject WScript.Shell
$shortcut = $shellObject.CreateShortcut($shortcutPath)
$shortcut.TargetPath = $pythonwPath
$shortcut.Arguments = '"' + (Join-Path $consoleRoot 'launch_rpbot.pyw') + '"'
$shortcut.WorkingDirectory = $consoleRoot
$shortcut.Description = 'RPBot mapping, RViz and diagnostics'
$shortcut.Save()
Write-Output "Created $shortcutPath"
