# One-time set-up so the car can reach the Mosquitto broker on this laptop.
# Run once from an administrator PowerShell:
#   powershell -ExecutionPolicy Bypass -File tools\mqtt_broker_setup.ps1
#
# 1. Makes the Mosquitto service listen on all addresses, port 1883, with
#    anonymous clients allowed (Mosquitto 2 only listens on localhost by
#    default).
# 2. Restarts the service.
# 3. Opens TCP 1883 in Windows Firewall.
# 4. Prints this laptop's IP addresses: enter the hotspot one on the car's
#    page under More controls -> MQTT broker.
#
# Anonymous access is fine on a phone hotspot for testing. Run with -Undo to
# remove the firewall rule and the listener lines again.

param([switch] $Undo)

$ErrorActionPreference = 'Stop'

$principal = New-Object Security.Principal.WindowsPrincipal(
    [Security.Principal.WindowsIdentity]::GetCurrent())
if (-not $principal.IsInRole(
        [Security.Principal.WindowsBuiltInRole]::Administrator))
{
    Write-Host 'Run this from an administrator PowerShell.' -ForegroundColor Red
    exit 1
}

$conf     = 'C:\Program Files\mosquitto\mosquitto.conf'
$begin    = '# --- SuperCar (tools/mqtt_broker_setup.ps1) ---'
$end      = '# --- end SuperCar ---'
$ruleName = 'Mosquitto MQTT 1883 (SuperCar)'

if (-not (Test-Path $conf))
{
    Write-Host "Mosquitto not found at $conf. Install it first:" `
        -ForegroundColor Red
    Write-Host '  winget install --id EclipseFoundation.Mosquitto -e'
    exit 1
}

# Drop any earlier SuperCar block, so running this twice is harmless.
$text = Get-Content $conf -Raw
$pattern = '(?s)\r?\n?' + [regex]::Escape($begin) + '.*?' +
           [regex]::Escape($end) + '\r?\n?'
$text = [regex]::Replace($text, $pattern, "`r`n")

if ($Undo)
{
    Set-Content -Path $conf -Value $text.TrimEnd() -Encoding ascii
    Remove-NetFirewallRule -DisplayName $ruleName -ErrorAction SilentlyContinue
    Restart-Service mosquitto
    Write-Host 'Removed: Mosquitto is back to localhost only.'
    exit 0
}

$block = @"

$begin
listener 1883 0.0.0.0
allow_anonymous true
$end
"@
Set-Content -Path $conf -Value ($text.TrimEnd() + "`r`n" + $block) `
    -Encoding ascii
Restart-Service mosquitto
Write-Host 'Mosquitto now listens on all addresses, port 1883.'

if (-not (Get-NetFirewallRule -DisplayName $ruleName `
            -ErrorAction SilentlyContinue))
{
    New-NetFirewallRule -DisplayName $ruleName -Direction Inbound `
        -Protocol TCP -LocalPort 1883 -Action Allow -Profile Any | Out-Null
}
Write-Host 'Firewall: TCP 1883 allowed in.'

Write-Host ''
Write-Host 'This laptop''s addresses (use the one on the car''s hotspot):'
Get-NetIPAddress -AddressFamily IPv4 |
    Where-Object { $_.IPAddress -notlike '127.*' -and
                   $_.IPAddress -notlike '169.254.*' } |
    ForEach-Object { '  {0,-15} {1}' -f $_.IPAddress, $_.InterfaceAlias }
