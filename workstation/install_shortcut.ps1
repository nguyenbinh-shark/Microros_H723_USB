$ErrorActionPreference = 'Stop'
$consoleRoot = $PSScriptRoot
$exePath = Join-Path $consoleRoot 'dist\RPBotStudio.exe'
if (Test-Path -LiteralPath $exePath) {
    $targetPath = $exePath
    $targetArguments = ''
    $iconPath = "$exePath,0"
} else {
    $pythonPath = (& python -c 'import sys; print(sys.executable)').Trim()
    if ($LASTEXITCODE -ne 0) { throw 'Build RPBotStudio.exe or install Python 3 with tkinter.' }
    $targetPath = Join-Path (Split-Path -Parent $pythonPath) 'pythonw.exe'
    if (-not (Test-Path -LiteralPath $targetPath)) { throw "Missing $targetPath" }
    $targetArguments = '"' + (Join-Path $consoleRoot 'launch_rpbot.pyw') + '"'
    $iconPath = Join-Path $consoleRoot 'assetspbot-studio.ico'
}
$desktopPath = [Environment]::GetFolderPath('Desktop')
$shortcutPath = Join-Path $desktopPath 'RPBot Console.lnk'
$shellObject = New-Object -ComObject WScript.Shell
$shortcut = $shellObject.CreateShortcut($shortcutPath)
$shortcut.TargetPath = $targetPath
$shortcut.Arguments = $targetArguments
$shortcut.WorkingDirectory = $consoleRoot
$shortcut.Description = 'RPBot Studio by Tran Nguyen Binh - Mapping, Navigation and diagnostics'
$shortcut.IconLocation = $iconPath
$shortcut.Save()
Write-Output "Created $shortcutPath"
