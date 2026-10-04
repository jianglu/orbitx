$ErrorActionPreference = "Stop"
$inv = [Globalization.CultureInfo]::InvariantCulture

function Fmt([double]$x) {
    return $x.ToString('G6', $inv)
}

function Get-CylInertia([double]$m, [double]$r, [double]$h) {
    if ($m -le 0) { return @(0.0, 0.0, 0.0) }
    $iAx = 0.5 * $m * $r * $r
    $iTr = $m / 12.0 * (3.0 * $r * $r + $h * $h)
    return @($iTr, $iAx, $iTr)
}

function Migrate-Text([string]$text) {
    $parts = [regex]::Split($text, '(\[\[stages\]\])')
    $sb = New-Object System.Text.StringBuilder
    for ($i = 0; $i -lt $parts.Length; $i++) {
        if ($parts[$i] -ne '[[stages]]') {
            [void]$sb.Append($parts[$i])
            continue
        }
        [void]$sb.Append($parts[$i])
        $i++
        if ($i -ge $parts.Length) { break }
        $body = $parts[$i]
        $dryM = [regex]::Match($body, '(?m)^dry_mass\s*=\s*([0-9.eE+-]+)')
        $fuelM = [regex]::Match($body, '(?m)^fuel_mass\s*=\s*([0-9.eE+-]+)')
        $lenM = [regex]::Match($body, '(?m)^length\s*=\s*([0-9.eE+-]+)')
        $radM = [regex]::Match($body, '(?m)^radius\s*=\s*([0-9.eE+-]+)')
        $dry = if ($dryM.Success) { [double]$dryM.Groups[1].Value } else { 0.0 }
        $length = if ($lenM.Success) { [double]$lenM.Groups[1].Value } else { 10.0 }
        $radius = if ($radM.Success) { [double]$radM.Groups[1].Value } else { 1.0 }
        $body = [regex]::Replace($body, '(?m)^fuel_mass\s*=\s*[^\r\n]+\r?\n?', '')
        $body = [regex]::Replace($body, '(?m)^inertia\s*=\s*[^\r\n]+\r?\n?', '')
        if ($fuelM.Success) {
            $fuel = [double]$fuelM.Groups[1].Value
            if (($dry + $fuel) -gt 0 -and $fuel -gt 0) {
                $yDry = -2.0 * $fuel / ($dry + $fuel)
                $yFuel = 2.0 * $dry / ($dry + $fuel)
            } else {
                $yDry = 0.0
                $yFuel = 0.0
            }
            $di = Get-CylInertia $dry $radius $length
            $ti = Get-CylInertia $fuel $radius ($length * 0.7)
            $insert = "dry_center = [0.0, $(Fmt $yDry), 0.0]`r`ndry_inertia = [$(Fmt $di[0]), $(Fmt $di[1]), $(Fmt $di[2])]`r`n"
            $body = [regex]::Replace($body, '(?m)^(dry_mass\s*=\s*[^\r\n]+\r?\n)', "`$1$insert", 1)
            $tank = "`r`n[[stages.tanks]]`r`nid = 0`r`nmax_mass = $fuel`r`nmass = $fuel`r`npos = [0.0, $(Fmt $yFuel), 0.0]`r`ninertia = [$(Fmt $ti[0]), $(Fmt $ti[1]), $(Fmt $ti[2])]`r`nefficiency = 1.0`r`n`r`n"
            $re = New-Object System.Text.RegularExpressions.Regex('\[\[stages\.thrusters\]\]')
            if ($re.IsMatch($body)) {
                $body = $re.Replace($body, ($tank + '[[stages.thrusters]]'), 1)
            } else {
                $body = $body.TrimEnd() + "`r`n" + $tank
            }
        } elseif ($body -notmatch 'dry_center') {
            $di = Get-CylInertia $dry $radius $length
            $insert = "dry_center = [0.0, 0.0, 0.0]`r`ndry_inertia = [$(Fmt $di[0]), $(Fmt $di[1]), $(Fmt $di[2])]`r`n"
            $body = [regex]::Replace($body, '(?m)^(dry_mass\s*=\s*[^\r\n]+\r?\n)', "`$1$insert", 1)
        }
        [void]$sb.Append($body)
    }
    return $sb.ToString()
}

$dir = "e:\Project\SimRocket\orbitx\crates\orbitx-config\presets"
Get-ChildItem $dir -Filter *.toml | ForEach-Object {
    if ($_.Name -match 'scenario') { return }
    $bytes = [System.IO.File]::ReadAllBytes($_.FullName)
    $text = [System.Text.Encoding]::UTF8.GetString($bytes)
    if ($text -notmatch 'fuel_mass') {
        Write-Host "skip $($_.Name)"
        return
    }
    $new = Migrate-Text $text
    $utf8NoBom = New-Object System.Text.UTF8Encoding $false
    [System.IO.File]::WriteAllText($_.FullName, $new, $utf8NoBom)
    Write-Host "migrated $($_.Name)"
}
