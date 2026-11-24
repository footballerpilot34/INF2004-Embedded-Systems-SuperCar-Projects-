# Decoder and encoder for the SuperCar's binary MQTT frames, dot-sourced by
# tools/mqtt_car.ps1. Tested against tools/mqtt_frame.py (the reference).
#
# ---------------------------------------------------------------------
# Binary frames (change requirement 1). The car sends every payload except
# "online" as a compact binary frame, not JSON: byte 0 is the header (type
# in bits 0-3, version 0xA in bits 4-7), then the fields packed LSB first
# with no padding. The layouts are in app_program/comms/mqtt_frame.h; this
# decoder, tools/mqtt_frame.py and mqtt_frame.c must stay in step.
# ---------------------------------------------------------------------
$FrameTypes = @{ 1 = 'telemetry'; 2 = 'heartbeat'; 3 = 'barcode'; 4 = 'obstacle';
                 5 = 'hump'; 6 = 'log'; 7 = 'cmd'; 8 = 'cmd/ack' }
# Name, bits, signed.
$Layouts = @{
    'telemetry' = @(@('t_msec', 32, $false), @('state', 4, $false),
        @('bypass_phase', 4, $false), @('motor_left', 8, $true),
        @('motor_right', 8, $true), @('enc_left', 16, $false),
        @('enc_right', 16, $false), @('line_left', 1, $false),
        @('line_right', 1, $false), @('barcode', 1, $false),
        @('range_mm', 12, $false), @('pitch_x10', 12, $true),
        @('roll_x10', 12, $true), @('servo_deg', 8, $false),
        @('hump_x10', 11, $true), @('accel_mg', 13, $false),
        @('lift_mm_s', 12, $true), @('pending_turn', 3, $false),
        @('faults', 8, $false))
    'heartbeat' = @(@('seq', 16, $false), @('up_s', 24, $false),
        @('vsys_mv', 13, $false), @('sent', 16, $false), @('dropped', 16, $false),
        @('connects', 8, $false), @('wifi_mode', 3, $false),
        @('hotspot', 3, $false), @('ip', 32, $false), @('rssi_dbm', 8, $true),
        @('ping_ok', 1, $false), @('ping_msec', 10, $false),
        @('ping_age_s', 8, $false), @('link_drops', 8, $false),
        @('restart', 8, $false), @('faults', 8, $false),
        @('loop_max_us', 16, $false), @('overruns', 16, $false))
    'obstacle' = @(@('t_msec', 32, $false), @('front_mm', 12, $false),
        @('left_mm', 13, $true), @('right_mm', 13, $true), @('mid_mm', 13, $true),
        @('width_mm', 12, $false), @('geometric', 1, $false),
        @('left_status', 3, $false), @('right_status', 3, $false))
    'hump' = @(@('t_msec', 32, $false), @('height_x10', 11, $true),
        @('tilt_x10', 12, $true), @('run_height_x10', 11, $true),
        @('run_tilt_x10', 12, $true), @('count', 8, $false))
}
$Code39   = '0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%*'
$States   = @('READY', 'FOLLOW_LINE', 'PROBE', 'HUMP', 'BLOCKED', 'STOPPED',
              'LINE_CAL', 'TURN', 'FEATURE_TEST', 'BARCODE_STATIC',
              'OBSTACLE_SCAN', 'BYPASS')
$Phases   = @('idle', 'pivot', 'out', 're-scan', 'pass', 'return', 'advance',
              'align', 'finish', 'brake', 'plan', 'line-nudge', 'back-up')
$Turns    = @('MESSAGE', 'TURN LEFT', 'TURN RIGHT', 'U-TURN', 'GO STRAIGHT')
$Edges    = @('edge not observed', 'missing echoes', 'unreliable observations',
              'echo-bracketed', 'estimated at scan limit',
              'estimated distance jump')
$LogCats  = @('system', 'obstacle', 'line', 'state', 'servo')
$Faults   = @('motor stall', 'encoder', 'IMU', 'no echo', 'Wi-Fi loss',
              'MQTT down', 'battery low', 'loop overrun')
$WifiModes = @('off', 'joining', 'joined', 'rejoining', 'own network')
$Restarts = @('unknown', 'power-on or brown-out', 'reset button',
              'debugger restart', 'software reset', 'watchdog (froze)',
              'CPU hard fault', 'network stack check failed')
$Stages   = @('unknown', 'Wi-Fi chip/network polling', 'web server',
              'hotspot joining', 'battery reading', 'buttons', 'serial keys',
              'running a command', 'servo', 'ultrasonic', 'IMU', 'barcode',
              'vehicle control', 'obstacle scan test', 'IR diag', 'buzzer',
              'telemetry', 'MQTT', 'waiting for the next tick', 'fault check')
$Words    = @('stop', 'start', 'calibrate', 'wheel', 'diag', 'black', 'white',
              'bt', 'bs', 'scan', 'v', 'mark', 'clear', 'bclist', 'bcreset',
              'bc', 'ft')
$CmdActions = 'nlrsu'

function ConvertFrom-Hex ([string] $Hex)
{
    $bytes = New-Object byte[] ($Hex.Length / 2)
    for ($i = 0; $i -lt $bytes.Length; $i++)
    {
        $bytes[$i] = [Convert]::ToByte($Hex.Substring(2 * $i, 2), 16)
    }
    return , $bytes
}

# Read one LSB-first field; $State.Pos moves on.
function Read-Bits ($State, [int] $Width, [bool] $Signed = $false)
{
    [long] $value = 0
    for ($i = 0; $i -lt $Width; $i++)
    {
        $bit = $State.Pos + $i
        $index = [int][math]::Floor($bit / 8)
        if ($index -ge $State.Data.Length) { throw 'frame too short' }
        if ((($State.Data[$index] -shr ($bit % 8)) -band 1) -ne 0)
        {
            $value = $value -bor ([long] 1 -shl $i)
        }
    }
    $State.Pos += $Width
    if ($Signed -and (($value -band ([long] 1 -shl ($Width - 1))) -ne 0))
    {
        $value -= ([long] 1 -shl $Width)
    }
    return $value
}

function Get-Name ($Table, [long] $Index)
{
    if (($Index -ge 0) -and ($Index -lt $Table.Count)) { return $Table[$Index] }
    return "?$Index"
}

function Get-FaultText ([long] $Mask)
{
    $names = @(0..7 | Where-Object { ($Mask -band (1 -shl $_)) -ne 0 } |
               ForEach-Object { $Faults[$_] })
    if ($names.Count -eq 0) { return 'none' }
    return ($names -join ', ')
}

# Decode one payload: an object with Kind, F (the fields) and Text.
function ConvertFrom-Frame ([byte[]] $Data)
{
    if (($Data.Length -eq 0) -or (($Data[0] -shr 4) -ne 0xA) -or
        (-not $FrameTypes.ContainsKey([int]($Data[0] -band 0xF))))
    {
        throw 'not a frame'
    }
    $kind  = $FrameTypes[[int]($Data[0] -band 0xF)]
    $state = @{ Data = $Data; Pos = 8 }
    $f     = @{}
    if ($Layouts.ContainsKey($kind))
    {
        foreach ($field in $Layouts[$kind])
        {
            $f[$field[0]] = Read-Bits $state $field[1] $field[2]
        }
    }
    elseif ($kind -eq 'barcode')
    {
        $f.t_msec  = Read-Bits $state 32
        $f.command = Read-Bits $state 3
        $count     = Read-Bits $state 5
        $text      = ''
        for ($i = 0; $i -lt $count; $i++)
        {
            $c = Read-Bits $state 6
            if ($c -lt $Code39.Length) { $text += $Code39[[int]$c] } else { $text += '?' }
        }
        $f.message = $text
    }
    elseif ($kind -eq 'log')
    {
        $f.category = Read-Bits $state 3
        $f.text = [Text.Encoding]::UTF8.GetString($Data, 2, $Data.Length - 2)
    }
    else
    {
        $opcode = Read-Bits $state 5
        $word = Get-Name $Words $opcode
        if ($word -eq 'bc')
        {
            $letter = Read-Bits $state 5
            $action = Read-Bits $state 3
            $word = 'bc' + [char](97 + $letter) + $CmdActions[[int]$action]
        }
        elseif ($word -eq 'ft')
        {
            for ($i = 0; $i -lt 4; $i++) { $word += [string](Read-Bits $state 1) }
        }
        $f.word = $word
    }
    return [pscustomobject]@{ Kind = $kind; F = $f; Text = (Get-FrameText $kind $f) }
}

function Get-FrameText ([string] $Kind, $f)
{
    switch ($Kind)
    {
        'telemetry'
        {
            $st = Get-Name $States $f.state
            if ($st -eq 'BYPASS') { $st += '/' + (Get-Name $Phases $f.bypass_phase) }
            $range = if ($f.range_mm -eq 0) { '-' } else { $f.range_mm }
            $text = ('{0,-13} motors {1,4} {2,4}  IR {3}{4}{5}  range {6,5}' +
                     '  pitch {7,5:N1}  roll {8,5:N1}  height {9,4:N1} mm' +
                     '  servo {10,3}') -f $st, $f.motor_left, $f.motor_right,
                     $f.line_left, $f.line_right, $f.barcode, $range,
                     ($f.pitch_x10 / 10), ($f.roll_x10 / 10),
                     ($f.hump_x10 / 10), $f.servo_deg
            if ($f.pending_turn -ne 0) { $text += '  next ' + (Get-Name $Turns $f.pending_turn) }
            if ($f.faults -ne 0) { $text += '  FAULTS: ' + (Get-FaultText $f.faults) }
            return $text
        }
        'heartbeat'
        {
            $ip = (0, 8, 16, 24 | ForEach-Object { ($f.ip -shr $_) -band 0xFF }) -join '.'
            $kind = $f.restart -band 7
            $why = Get-Name $Restarts $kind
            if (($kind -eq 5) -or ($kind -eq 6)) { $why += ' in ' + (Get-Name $Stages ($f.restart -shr 3)) }
            $ping = if ($f.ping_ok -ne 0) { "$($f.ping_msec) ms" } else { '-' }
            return ('heartbeat #{0}  up {1} s  battery {2} mV  sent {3}  dropped {4}' +
                    '  connections {5}  Wi-Fi {6} #{7} {8} {9} dBm ping {10}' +
                    '  loop max {11} us ({12} overruns)  faults: {13}' +
                    '  last restart: {14}') -f $f.seq, $f.up_s, $f.vsys_mv,
                    $f.sent, $f.dropped, $f.connects,
                    (Get-Name $WifiModes $f.wifi_mode), ($f.hotspot + 1), $ip,
                    $f.rssi_dbm, $ping, $f.loop_max_us, $f.overruns,
                    (Get-FaultText $f.faults), $why
        }
        'barcode' { return "BARCODE '$($f.message)' -> $(Get-Name $Turns $f.command)" }
        'obstacle'
        {
            $none = { param($v, $n) if ($v -eq $n) { '-' } else { "$v" } }
            return ('OBSTACLE {0} mm ahead, {1} mm wide (edges {2} / {3} mm, {4} / {5})' -f
                    (& $none $f.front_mm 0), (& $none $f.width_mm 0),
                    (& $none $f.left_mm -4096), (& $none $f.right_mm -4096),
                    (Get-Name $Edges $f.left_status), (Get-Name $Edges $f.right_status))
        }
        'hump'
        {
            return ('HUMP peak {0:N1} mm, tilt {1:N1} deg (highest this run {2:N1} mm, {3} humps)' -f
                    ($f.height_x10 / 10), ($f.tilt_x10 / 10),
                    ($f.run_height_x10 / 10), $f.count)
        }
        'log' { return $f.text }
        default { return $f.word }
    }
}

# Build the binary command frame for a web-page command word ($null if it
# is not one). Bits are collected LSB first, then packed into bytes.
function ConvertTo-CommandFrame ([string] $Word)
{
    $word = $Word.Trim().ToLower().Replace(' ', '')
    $bits = New-Object System.Collections.Generic.List[int]
    $put = { param($value, $width) for ($i = 0; $i -lt $width; $i++) { $bits.Add(($value -shr $i) -band 1) } }
    & $put 7 4
    & $put 0xA 4
    if (($word.Length -eq 4) -and $word.StartsWith('bc') -and
        ($word[2] -match '[a-z]') -and ($CmdActions.IndexOf($word[3]) -ge 0) -and
        ($Words -notcontains $word))
    {
        & $put ([array]::IndexOf($Words, 'bc')) 5
        & $put ([int][char]$word[2] - 97) 5
        & $put ($CmdActions.IndexOf($word[3])) 3
    }
    elseif ($word.StartsWith('ft') -and (($word.Length -eq 5) -or ($word.Length -eq 6)) -and
            ($word.Substring(2) -match '^[01]+$'))
    {
        $digits = $word.Substring(2)
        if ($digits.Length -eq 3) { $digits += '1' } # Older ft111: humps on.
        & $put ([array]::IndexOf($Words, 'ft')) 5
        foreach ($d in $digits.ToCharArray()) { & $put ([int][string]$d) 1 }
    }
    elseif (($Words -contains $word) -and ($word -ne 'bc') -and ($word -ne 'ft'))
    {
        & $put ([array]::IndexOf($Words, $word)) 5
    }
    else
    {
        return $null
    }
    $bytes = New-Object byte[] ([int][math]::Ceiling($bits.Count / 8))
    for ($i = 0; $i -lt $bits.Count; $i++)
    {
        if ($bits[$i] -ne 0) { $bytes[[int][math]::Floor($i / 8)] = $bytes[[int][math]::Floor($i / 8)] -bor (1 -shl ($i % 8)) }
    }
    return , $bytes
}
