param(
    [string]$PiAddress = '192.168.0.120',
    [int]$PortBase = 7400,
    [int]$Domain = 0,
    [switch]$Apply
)
$ErrorActionPreference = 'Stop'
$creatorId = '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}'
$principal = New-Object Security.Principal.WindowsPrincipal([Security.Principal.WindowsIdentity]::GetCurrent())
$isAdmin = $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
Write-Output "Administrator=$isAdmin"
Get-NetFirewallHyperVVMSetting -PolicyStore ActiveStore | Format-Table Name,DefaultInboundAction
Get-NetFirewallHyperVRule -VMCreatorId $creatorId |
    Where-Object { $_.Name -like '*DDS*' -or $_.Name -like 'RPBot*' } |
    Select-Object Name,Direction,Action,Protocol,LocalPorts,RemoteAddresses | ConvertTo-Json -Depth 3
if (-not $Apply) { return }
if (-not $isAdmin) { throw 'Run this script in an Administrator PowerShell window with -Apply.' }
$piIp = [System.Net.IPAddress]::Parse($PiAddress)
if ($piIp.AddressFamily -ne [System.Net.Sockets.AddressFamily]::InterNetwork) { throw 'Use the Pi IPv4 address.' }
$firstPort = $PortBase + 250 * $Domain
$lastPort = $firstPort + 125
if ($firstPort -lt 1024 -or $lastPort -gt 65535) { throw 'Invalid DDS port range.' }
$portRange = "$firstPort-$lastPort"
$ruleName = 'RPBot-WSL-DDS-In'
if (Get-NetFirewallHyperVRule -Name $ruleName -ErrorAction SilentlyContinue) {
    Set-NetFirewallHyperVRule -Name $ruleName -Direction Inbound -Action Allow -Protocol UDP -LocalPorts $portRange -RemoteAddresses $PiAddress | Out-Null
} else {
    New-NetFirewallHyperVRule -Name $ruleName -DisplayName 'RPBot DDS from robot to WSL' -Direction Inbound -Action Allow -VMCreatorId $creatorId -Protocol UDP -LocalPorts $portRange -RemoteAddresses $PiAddress | Out-Null
}
if (Get-NetFirewallRule -Name $ruleName -ErrorAction SilentlyContinue) {
    Set-NetFirewallRule -Name $ruleName -Enabled True -Direction Inbound -Action Allow -Protocol UDP -LocalPort $portRange -RemoteAddress $PiAddress | Out-Null
} else {
    New-NetFirewallRule -Name $ruleName -DisplayName 'RPBot DDS from robot to WSL' -Enabled True -Direction Inbound -Action Allow -Protocol UDP -LocalPort $portRange -RemoteAddress $PiAddress | Out-Null
}
Write-Output "Allowed UDP $portRange only from Pi $PiAddress. Other firewall settings unchanged."
