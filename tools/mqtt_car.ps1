# Connect this laptop to the car over MQTT, in one go.
#
# From PowerShell in the repo folder, run:   .\mqtt
# (or: powershell -ExecutionPolicy Bypass -File tools\mqtt_car.ps1)
#
# It:
#  1. checks the Mosquitto broker is installed, running and open to the
#     network (if not, run tools\mqtt_broker_setup.ps1 once as admin);
#  2. finds this laptop's address on the hotspot and tells the car to use
#     it as its broker (over Wi-Fi, as the page's MQTT broker box does);
#  3. opens a second window that shows what the car sends and saves it
#     all to %USERPROFILE%\mqtt_log.txt (UTF-8, added to the end);
#  4. lets you type commands for the car here: start, stop, ft1111, ...
#
# The car also keeps trying the broker in src/config.h (MQTT_BROKER_HOST)
# by itself, every 5 s, whenever it is on the hotspot.

param(
    [switch] $Watch,                            # Run as the watch window.
    [string] $Car = 'supercar-aai14.local'      # The car's name or IP.
)

$Mosquitto = 'C:\Program Files\mosquitto'
$Prefix    = 'inf2004/aai14/supercar'
$LogPath   = Join-Path $env:USERPROFILE 'mqtt_log.txt'
$Sub       = Join-Path $Mosquitto 'mosquitto_sub.exe'
$Pub       = Join-Path $Mosquitto 'mosquitto_pub.exe'

# ---------------------------------------------------------------------
# The watch window: show the car's messages and save them all.
# ---------------------------------------------------------------------
if ($Watch)
{
    $Host.UI.RawUI.WindowTitle = 'SuperCar MQTT - what the car sends'
    $utf8   = New-Object System.Text.UTF8Encoding $false
    $writer = New-Object System.IO.StreamWriter($LogPath, $true, $utf8)
    $writer.AutoFlush = $true
    $script:shown = 0

    Write-Host "Saving everything to $LogPath" -ForegroundColor DarkGray
    Write-Host 'Telemetry is shown once a second; the file has all of it.' `
        -ForegroundColor DarkGray
    Write-Host 'Close this window (or Ctrl+C) to stop.' -ForegroundColor DarkGray
    Write-Host ''

    & $Sub -h localhost -t "$Prefix/#" -v | ForEach-Object {
        $time = Get-Date -Format 'HH:mm:ss.fff'
        $writer.WriteLine("$time $_")

        $space = $_.IndexOf(' ')
        if ($space -lt 0) { return }
        $topic = $_.Substring(0, $space).Replace("$Prefix/", '')
        $body  = $_.Substring($space + 1)

        switch ($topic)
        {
            'telemetry'
            {
                $script:shown++
                if (($script:shown % 4) -ne 1) { return }
                try
                {
                    $d = $body | ConvertFrom-Json
                    $line = '{0} {1,-13} motors {2,3} {3,3}  range {4,5}' +
                            '  pitch {5,5:N1}  roll {6,5:N1}  height {7,4} mm'
                    Write-Host ($line -f $time, $d.st, $d.ml, $d.mr, $d.rg,
                                ($d.pt / 10), ($d.rl / 10), $d.hh) `
                        -ForegroundColor Gray
                }
                catch { Write-Host "$time telemetry $body" }
            }
            'heartbeat' { Write-Host "$time heartbeat $body" -ForegroundColor DarkGray }
            'online'
            {
                if ($body -eq '1') { Write-Host "$time CAR ONLINE" -ForegroundColor Green }
                else { Write-Host "$time CAR OFFLINE" -ForegroundColor Red }
            }
            'cmd'     { Write-Host "$time sent command: $body" -ForegroundColor Cyan }
            'cmd/ack' { Write-Host "$time car took: $body" -ForegroundColor Cyan }
            default   { Write-Host "$time $topic $body" -ForegroundColor Yellow }
        }
    }
    $writer.Close()
    return
}

# ---------------------------------------------------------------------
# The main window.
# ---------------------------------------------------------------------
$Host.UI.RawUI.WindowTitle = 'SuperCar MQTT - commands'

# 1. The broker.
if (-not (Test-Path $Sub))
{
    Write-Host 'Mosquitto is not installed. Install it with:' -ForegroundColor Red
    Write-Host '  winget install --id EclipseFoundation.Mosquitto -e'
    exit 1
}
$service = Get-Service mosquitto -ErrorAction SilentlyContinue
if ((-not $service) -or ($service.Status -ne 'Running'))
{
    Write-Host 'The Mosquitto service is not running. Start it (as admin):' `
        -ForegroundColor Red
    Write-Host '  Start-Service mosquitto'
    exit 1
}
$conf = Join-Path $Mosquitto 'mosquitto.conf'
if (-not (Select-String -Path $conf -Pattern 'listener 1883 0.0.0.0' -Quiet))
{
    Write-Host 'Mosquitto only listens on this laptop, so the car cannot' `
        'reach it. Run this once from an admin PowerShell:' -ForegroundColor Red
    Write-Host '  powershell -ExecutionPolicy Bypass -File tools\mqtt_broker_setup.ps1'
    exit 1
}
Write-Host 'Broker: Mosquitto is running and open to the hotspot.' `
    -ForegroundColor Green

# 2. This laptop's address on the hotspot (an iPhone hotspot: 172.20.10.x).
$addresses = Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue |
    Where-Object { $_.IPAddress -notlike '127.*' -and
                   $_.IPAddress -notlike '169.254.*' }
$ip = ($addresses | Where-Object { $_.IPAddress -like '172.20.10.*' } |
       Select-Object -First 1).IPAddress
if (-not $ip)
{
    $ip = ($addresses | Where-Object { $_.InterfaceAlias -like '*Wi-Fi*' } |
           Select-Object -First 1).IPAddress
}
if (-not $ip)
{
    Write-Host 'Could not find this laptop on a Wi-Fi network. Join the' `
        'hotspot first.' -ForegroundColor Red
    exit 1
}
Write-Host "This laptop on the hotspot: $ip"

# 3. Tell the car (the same as the page's MQTT broker box).
function Send-Broker
{
    try
    {
        $url = "http://$Car/cmd?c=mb$ip"
        Invoke-WebRequest -Uri $url -UseBasicParsing -TimeoutSec 5 | Out-Null
        Write-Host "Car told to use $ip as its broker." -ForegroundColor Green
    }
    catch
    {
        Write-Host "Could not reach the car at $Car. It keeps trying the" `
            'broker in config.h by itself; if that is not this laptop, set' `
            "it on the car's page (More controls -> MQTT broker)." `
            -ForegroundColor Yellow
    }
}
Send-Broker

# 4. The watch window.
Start-Process powershell -ArgumentList @(
    '-NoExit', '-ExecutionPolicy', 'Bypass',
    '-File', "`"$PSCommandPath`"", '-Watch')
Write-Host 'Opened the watch window (what the car sends).'

# 5. Commands.
Write-Host ''
Write-Host 'Type a command for the car and press Enter:' -ForegroundColor Cyan
Write-Host '  start          START: calibrate on the line, then follow it'
Write-Host '  stop           stop at once'
Write-Host '  ftLABH         feature test, each 1/0: line, avoid, barcode, hump'
Write-Host '                 (ft1111 = all on, ft0001 = humps only)'
Write-Host '  calibrate      calibrate the IMU (car still and level)'
Write-Host '  bt, scan, wheel, clear   (as on the page)'
Write-Host '  broker         tell the car this laptop''s address again'
Write-Host '  q              quit (the watch window keeps running)'
Write-Host ''

while ($true)
{
    $word = (Read-Host 'car').Trim().ToLower()
    if ($word -eq '') { continue }
    if (($word -eq 'q') -or ($word -eq 'quit') -or ($word -eq 'exit')) { break }
    if ($word -eq 'broker') { Send-Broker; continue }
    & $Pub -h localhost -t "$Prefix/cmd" -m $word
}
