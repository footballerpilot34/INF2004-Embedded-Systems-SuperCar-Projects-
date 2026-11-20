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
#  3. opens two windows:
#     - OUTPUT: what the car says - its log (state changes, Wi-Fi,
#       barcodes, obstacles, humps, ...), events, online/offline and
#       command replies. It also saves EVERYTHING (pings included) to
#       %USERPROFILE%\mqtt_log.txt (UTF-8, added to the end);
#     - PINGS: the constant telemetry and heartbeat, once a second;
#  4. keeps this window for typing commands: start, stop, ft1111, set A
#     left, letters, ...
#
# The car also keeps trying the broker in src/config.h (MQTT_BROKER_HOST)
# by itself, every 5 s, whenever it is on the hotspot.

param(
    [string] $View = '',                        # '', 'output' or 'pings'.
    [string] $Car  = 'supercar-aai14.local'     # The car's name or IP.
)

$Mosquitto = 'C:\Program Files\mosquitto'
$Prefix    = 'inf2004/aai14/supercar'
$LogPath   = Join-Path $env:USERPROFILE 'mqtt_log.txt'
$Sub       = Join-Path $Mosquitto 'mosquitto_sub.exe'
$Pub       = Join-Path $Mosquitto 'mosquitto_pub.exe'

# mosquitto_sub writes UTF-8 (e.g. the curly apostrophe in an iPhone's
# hotspot name); read it as such, not in the console's old code page.
[Console]::OutputEncoding = New-Object System.Text.UTF8Encoding $false

# Split "topic payload" from mosquitto_sub -v into the short topic and body.
function Split-Message ([string] $Line)
{
    $space = $Line.IndexOf(' ')
    if ($space -lt 0) { return $null }
    return @($Line.Substring(0, $space).Replace("$Prefix/", ''),
             $Line.Substring($space + 1))
}

# ---------------------------------------------------------------------
# OUTPUT window: what the car says. Saves everything to the log file.
# ---------------------------------------------------------------------
if ($View -eq 'output')
{
    $Host.UI.RawUI.WindowTitle = 'SuperCar MQTT - OUTPUT'
    # Shared, so another window (an older OUTPUT window, a manual logger)
    # can have the file open too; if it still cannot be opened, use a new
    # file rather than stop showing messages.
    $utf8   = New-Object System.Text.UTF8Encoding $false
    $writer = $null
    foreach ($path in @($LogPath,
                        (Join-Path $env:USERPROFILE ('mqtt_log_{0}.txt' -f
                            (Get-Date -Format 'yyyyMMdd_HHmmss')))))
    {
        try
        {
            $stream = New-Object System.IO.FileStream($path,
                [System.IO.FileMode]::Append, [System.IO.FileAccess]::Write,
                [System.IO.FileShare]::ReadWrite)
            $writer = New-Object System.IO.StreamWriter($stream, $utf8)
            $writer.AutoFlush = $true
            $LogPath = $path
            break
        }
        catch
        {
            Write-Host "Cannot open $path ($($_.Exception.Message))" `
                -ForegroundColor Yellow
        }
    }
    if ($null -eq $writer)
    {
        Write-Host 'Not saving to a file; showing messages only.' -ForegroundColor Red
    }
    $colours = @{ 'system' = 'Gray'; 'obstacle' = 'Yellow';
                  'line' = 'White'; 'state' = 'Cyan'; 'servo' = 'DarkGray' }

    Write-Host "Saving everything (pings too) to $LogPath" -ForegroundColor DarkGray
    Write-Host 'Close this window (or Ctrl+C) to stop.' -ForegroundColor DarkGray
    Write-Host ''

    & $Sub -h localhost -t "$Prefix/#" -v | ForEach-Object {
        $time = Get-Date -Format 'HH:mm:ss.fff'
        if ($null -ne $writer) { $writer.WriteLine("$time $_") }
        $parts = Split-Message $_
        if ($null -eq $parts) { return }
        $topic = $parts[0]
        $body  = $parts[1]

        switch ($topic)
        {
            'telemetry' { }                         # The PINGS window.
            'heartbeat' { }
            'log'
            {
                try
                {
                    $d = $body | ConvertFrom-Json
                    $colour = $colours[[string] $d.c]
                    if (-not $colour) { $colour = 'Gray' }
                    Write-Host "$time $($d.m)" -ForegroundColor $colour
                }
                catch { Write-Host "$time $body" }
            }
            'online'
            {
                if ($body -eq '1') { Write-Host "$time === CAR ONLINE ===" -ForegroundColor Green }
                else { Write-Host "$time === CAR OFFLINE ===" -ForegroundColor Red }
            }
            'barcode'
            {
                try
                {
                    $d = $body | ConvertFrom-Json
                    Write-Host "$time >> BARCODE '$($d.msg)' -> $($d.cmd)" -ForegroundColor Magenta
                }
                catch { Write-Host "$time >> BARCODE $body" -ForegroundColor Magenta }
            }
            'obstacle'
            {
                try
                {
                    $d = $body | ConvertFrom-Json
                    $text = ("$time >> OBSTACLE {0} mm ahead, {1} mm wide " +
                             '(edges {2} / {3} mm, {4} / {5})') -f `
                        $d.front, $d.width, $d.left, $d.right, $d.lst, $d.rst
                    Write-Host $text -ForegroundColor Magenta
                }
                catch { Write-Host "$time >> OBSTACLE $body" -ForegroundColor Magenta }
            }
            'hump'
            {
                try
                {
                    $d = $body | ConvertFrom-Json
                    $text = ("$time >> HUMP peak {0} mm, tilt {1:N1} deg " +
                             '(highest this run {2} mm, {3} humps)') -f `
                        $d.height_mm, ($d.peak_tilt_x10 / 10), $d.run_height_mm, $d.n
                    Write-Host $text -ForegroundColor Magenta
                }
                catch { Write-Host "$time >> HUMP $body" -ForegroundColor Magenta }
            }
            'cmd'     { Write-Host "$time sent: $body" -ForegroundColor DarkCyan }
            'cmd/ack' { Write-Host "$time car took: $body" -ForegroundColor DarkCyan }
            default   { Write-Host "$time $topic $body" }
        }
    }
    if ($null -ne $writer) { $writer.Close() }
    return
}

# ---------------------------------------------------------------------
# PINGS window: the constant telemetry (once a second) and heartbeat.
# ---------------------------------------------------------------------
if ($View -eq 'pings')
{
    $Host.UI.RawUI.WindowTitle = 'SuperCar MQTT - PINGS'
    $script:shown = 0
    Write-Host 'Telemetry once a second (the OUTPUT window saves all of it).' `
        -ForegroundColor DarkGray
    Write-Host ''

    & $Sub -h localhost -t "$Prefix/telemetry" -t "$Prefix/heartbeat" -v |
        ForEach-Object {
        $time  = Get-Date -Format 'HH:mm:ss'
        $parts = Split-Message $_
        if ($null -eq $parts) { return }
        try
        {
            $d = $parts[1] | ConvertFrom-Json
            if ($parts[0] -eq 'telemetry')
            {
                $script:shown++
                if (($script:shown % 4) -ne 1) { return }
                $line = '{0} {1,-13} motors {2,3} {3,3}  IR {4}{5}{6}' +
                        '  range {7,5}  pitch {8,5:N1}  roll {9,5:N1}' +
                        '  height {10,4} mm  servo {11,3}'
                Write-Host ($line -f $time, $d.st, $d.ml, $d.mr, $d.l, $d.r,
                            $d.b, $d.rg, ($d.pt / 10), ($d.rl / 10), $d.hh,
                            $d.sv) -ForegroundColor Gray
            }
            else
            {
                $text = ('{0} heartbeat #{1}  up {2} s  battery {3} mV  ' +
                         'sent {4}  dropped {5}  connections {6}') -f `
                    $time, $d.seq, $d.up, $d.vb, $d.sent, $d.drop, $d.conn
                Write-Host $text -ForegroundColor DarkGray
            }
        }
        catch { Write-Host "$time $_" }
    }
    return
}

# ---------------------------------------------------------------------
# INPUT window (this one).
# ---------------------------------------------------------------------
$Host.UI.RawUI.WindowTitle = 'SuperCar MQTT - INPUT'

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

# 4. The OUTPUT and PINGS windows.
foreach ($name in @('pings', 'output'))
{
    Start-Process powershell -ArgumentList @(
        '-NoExit', '-ExecutionPolicy', 'Bypass',
        '-File', "`"$PSCommandPath`"", '-View', $name)
}
Write-Host 'Opened the OUTPUT and PINGS windows.'

# 5. Commands.
function Send-Command ([string] $Word)
{
    & $Pub -h localhost -t "$Prefix/cmd" -m $Word
}

function Show-Help
{
    Write-Host ''
    Write-Host 'Type a command for the car and press Enter:' -ForegroundColor Cyan
    Write-Host '  start              START: calibrate on the line, then follow it'
    Write-Host '  stop               stop at once'
    Write-Host '  ftLABH             feature test, each 1/0: line, avoid, barcode, hump'
    Write-Host '                     (ft1111 = all on, ft0001 = humps only)'
    Write-Host '  calibrate          calibrate the IMU (car still and level)'
    Write-Host '  set <letters> <command>   what a barcode letter does at the next'
    Write-Host '                     crossroad; one or several letters at once:'
    Write-Host '    set A left         turn left       (or l)'
    Write-Host '    set Z right        turn right      (or r)'
    Write-Host '    set C straight     go straight on  (or s, straight ahead)'
    Write-Host '    set D uturn        U-turn          (or u, u turn, u-turn)'
    Write-Host '    set Q none         no command: only a message  (or n)'
    Write-Host '    set AEI right      A, E and I all turn right'
    Write-Host '  letters            show what every letter does'
    Write-Host '  letters reset      back to the table in config.h'
    Write-Host '  bt, scan, wheel, clear   (as on the page)'
    Write-Host '  broker             tell the car this laptop''s address again'
    Write-Host '  help               this list'
    Write-Host '  q                  quit (the other windows keep running)'
    Write-Host ''
    Write-Host 'Letter changes last until the car restarts.' -ForegroundColor DarkGray
    Write-Host ''
}

# The command after "set <letters>", spaces and dashes taken out first, so
# "u turn", "u-turn" and "straight ahead" all work.
$Actions = @{ 'l' = 'l'; 'left' = 'l'; 'turnleft' = 'l';
              'r' = 'r'; 'right' = 'r'; 'turnright' = 'r';
              's' = 's'; 'straight' = 's'; 'straightahead' = 's';
              'ahead' = 's'; 'forward' = 's'; 'gostraight' = 's';
              'u' = 'u'; 'uturn' = 'u'; 'turnaround' = 'u';
              'n' = 'n'; 'none' = 'n'; 'message' = 'n'; 'nothing' = 'n' }

Show-Help
while ($true)
{
    $text  = (Read-Host 'car').Trim().ToLower()
    $words = @($text -split '\s+' | Where-Object { $_ -ne '' })
    if ($words.Count -eq 0) { continue }

    switch ($words[0])
    {
        { $_ -in @('q', 'quit', 'exit') } { return }
        'help'   { Show-Help }
        'broker' { Send-Broker }
        'letters'
        {
            if (($words.Count -gt 1) -and ($words[1] -eq 'reset'))
            { Send-Command 'bcreset' }
            else { Send-Command 'bclist' }
        }
        'set'
        {
            $action = ''
            if ($words.Count -ge 3)
            {
                $action = (($words[2..($words.Count - 1)]) -join '') -replace '-', ''
            }
            if (($words.Count -lt 3) -or ($words[1] -notmatch '^[a-z]+$') -or
                (-not $Actions.ContainsKey($action)))
            {
                Write-Host 'Use: set <letters> <left|right|straight|uturn|none>,' `
                    'e.g. set A left, set C straight, set D uturn' -ForegroundColor Yellow
            }
            else
            {
                foreach ($letter in $words[1].ToCharArray())
                {
                    Send-Command "bc$letter$($Actions[$action])"
                }
            }
        }
        default { Send-Command $text }
    }
}
