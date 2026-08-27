$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $ScriptDir
$ProjectRoot = (Resolve-Path (Join-Path $ScriptDir "..")).Path
$ProjectMainPath = [IO.Path]::GetFullPath((Join-Path $ScriptDir "main.exe"))

$ForceClose = $args -contains "--force-close"

$Profile = "mcu-fast"
$UserDefines = @()
for ($i = 0; $i -lt $args.Count; $i++) {
    if ($args[$i] -eq "--profile" -and ($i + 1) -lt $args.Count) {
        $Profile = $args[$i + 1]
        $i++
    } elseif ($args[$i] -eq "--define" -and ($i + 1) -lt $args.Count) {
        $UserDefines += $args[$i + 1]
        $i++
    }
}
$KnownProfiles = @("mcu-fast")
if ($KnownProfiles -notcontains $Profile) {
    throw "Unknown profile '$Profile'. Use mcu-fast."
}
$DriverDir = "..\Driver"
$BuildDir = "build"
$ObjSuffixParts = @()

foreach ($Define in $UserDefines) {
    $SafeDefine = $Define -replace '[^A-Za-z0-9_=-]', '_'
    $ObjSuffixParts += "D_$SafeDefine"
}
$ObjProfile = if ($ObjSuffixParts.Count -gt 0) { "$Profile-$($ObjSuffixParts -join '-')" } else { $Profile }
$ObjDir = Join-Path "build\obj" $ObjProfile

$CC = "gcc"
$CFlags = @("-O2", "-I$DriverDir")
$HostScanCFlags = @("-Os")
foreach ($Define in $UserDefines) {
    if ($Define -notmatch '^[A-Za-z_][A-Za-z0-9_]*(=.*)?$') {
        throw "Invalid define '$Define'. Use NAME or NAME=VALUE."
    }
    $CFlags += "-D$Define"
}
$LinkFlags = @("-static-libgcc")

function Invoke-Checked {
    param([string[]]$CommandLine)

    Write-Host ("[run] " + ($CommandLine -join " "))
    if ($CommandLine.Count -gt 1) {
        & $CommandLine[0] @($CommandLine[1..($CommandLine.Count - 1)])
    } else {
        & $CommandLine[0]
    }
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed with exit code $LASTEXITCODE"
    }
}

function Test-NeedsBuild {
    param(
        [string]$Output,
        [string[]]$Inputs
    )

    if (-not (Test-Path $Output)) {
        return $true
    }

    $OutTime = (Get-Item $Output).LastWriteTimeUtc
    foreach ($InputPath in $Inputs) {
        if ((Get-Item $InputPath).LastWriteTimeUtc -gt $OutTime) {
            return $true
        }
    }
    return $false
}

function Test-OutputHashMatches {
    param(
        [string]$Output,
        [string]$HashStamp
    )

    if (-not (Test-Path $Output) -or -not (Test-Path $HashStamp)) {
        return $false
    }
    $ExpectedHash = (Get-Content -Raw -LiteralPath $HashStamp).Trim()
    $ActualHash = (Get-FileHash -LiteralPath $Output -Algorithm SHA256).Hash
    return $ExpectedHash -eq $ActualHash
}

function Update-OutputHashStamp {
    param(
        [string]$Output,
        [string]$HashStamp
    )

    $OutputHash = (Get-FileHash -LiteralPath $Output -Algorithm SHA256).Hash
    Set-Content -LiteralPath $HashStamp -Encoding ASCII -NoNewline -Value $OutputHash
}

function Compile-Object {
    param(
        [string]$Source,
        [string]$Object,
        [string[]]$Headers,
        [string[]]$ExtraFlags = @()
    )

    $Inputs = @($Source) + $Headers
    if (Test-NeedsBuild -Output $Object -Inputs $Inputs) {
        Invoke-Checked (@($CC, "-c", $Source, "-o", $Object) + $CFlags + $ExtraFlags)
    } else {
        Write-Host "[skip] $([IO.Path]::GetFileName($Object)) is up to date"
    }
}

if ($ForceClose) {
    Write-Warning "[force-close] Only project processes are eligible; verify unsaved work before continuing."

    # Do not terminate every process with a generic name.  A locked output is
    # normally held by this checkout's main.exe or demo.py; unrelated user
    # programs must remain untouched.
    Get-Process -Name "main" -ErrorAction SilentlyContinue |
        Where-Object {
            try {
                $_.Path -and ([IO.Path]::GetFullPath($_.Path) -ieq $ProjectMainPath)
            } catch {
                $false
            }
        } |
        Stop-Process -Force -ErrorAction SilentlyContinue

    try {
        Get-CimInstance Win32_Process -Filter "Name = 'python.exe' OR Name = 'pythonw.exe'" |
            Where-Object {
                $_.CommandLine -and
                $_.CommandLine.IndexOf($ProjectRoot, [StringComparison]::OrdinalIgnoreCase) -ge 0
            } |
            ForEach-Object {
                Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue
            }
    } catch {
        Write-Warning "[force-close] Could not inspect Python command lines; close demo.py manually if needed."
    }
}

Write-Host "[info] Build profile: $Profile"
if ($UserDefines.Count -gt 0) { Write-Host "[info] Extra defines: $($UserDefines -join ', ')" }
New-Item -ItemType Directory -Force -Path $ObjDir | Out-Null
New-Item -ItemType Directory -Force -Path $BuildDir | Out-Null
$CompileStamp = Join-Path $ObjDir "compile_identity.stamp"

$Headers = @(
    Get-ChildItem -Path $DriverDir -Filter "*.h" | ForEach-Object { $_.FullName }
)
$Headers += Join-Path $ScriptDir "pc_flash.h"
$CompileIdentity = @(
    "cc=$CC"
    "cflags=$($CFlags -join ' ')"
    "sokoban_scan_cflags=$($HostScanCFlags -join ' ')"
) -join "`n"
$ExistingCompileIdentity = if (Test-Path $CompileStamp) { Get-Content -Raw -LiteralPath $CompileStamp } else { $null }
if ($ExistingCompileIdentity -ne $CompileIdentity) {
    Set-Content -LiteralPath $CompileStamp -Encoding ASCII -NoNewline -Value $CompileIdentity
}
$CompileInputs = $Headers + @($CompileStamp)
$Sources = [ordered]@{
    "main" = Join-Path $ScriptDir "pc_main.c"
    "pc_flash" = Join-Path $ScriptDir "pc_flash.c"
    "astar" = Join-Path $DriverDir "astar.c"
    "sokoban_flash" = Join-Path $DriverDir "sokoban_flash.c"
    "sokoban_scan" = Join-Path $DriverDir "sokoban_scan.c"
    "sokoban_solver" = Join-Path $DriverDir "sokoban_solver.c"
    "sokoban_recovery" = Join-Path $DriverDir "sokoban_recovery.c"
}

$Objects = @{}
foreach ($Name in $Sources.Keys) {
    $Objects[$Name] = Join-Path $ObjDir "$Name.o"
    $ExtraFlags = @()
    if ($Name -eq "sokoban_flash") {
        $ExtraFlags += @("-include", (Join-Path $ScriptDir "pc_flash.h"))
    }
    if ($Name -eq "sokoban_scan") {
        # This is a host-GCC size proxy. MCU optimization is owned by the Keil project,
        # never by compiler-specific pragmas in the portable Driver source.
        $ExtraFlags += $HostScanCFlags
    }
    Compile-Object -Source $Sources[$Name] -Object $Objects[$Name] -Headers $CompileInputs -ExtraFlags $ExtraFlags
}

function Update-LinkStamp {
    param(
        [string]$Stamp,
        [string]$Identity
    )

    $ExistingIdentity = if (Test-Path $Stamp) { Get-Content -Raw -LiteralPath $Stamp } else { $null }
    if ($ExistingIdentity -ne $Identity) {
        Set-Content -LiteralPath $Stamp -Encoding ASCII -NoNewline -Value $Identity
    }
}

$MainExe = "main.exe"
$MainTmp = Join-Path $BuildDir "main.exe.tmp"
$MainObjects = @($Objects["main"], $Objects["pc_flash"], $Objects["astar"], $Objects["sokoban_flash"], $Objects["sokoban_scan"], $Objects["sokoban_solver"], $Objects["sokoban_recovery"])
$MainLinkStamp = Join-Path $BuildDir "main.link_identity.stamp"
$MainLinkIdentity = @(
    "target=main.exe"
    "profile=$Profile"
    "obj_profile=$ObjProfile"
    "cc=$CC"
    "cflags=$($CFlags -join ' ')"
    "linkflags=$($LinkFlags -join ' ')"
    "objects=$($MainObjects -join '|')"
) -join "`n"
Update-LinkStamp -Stamp $MainLinkStamp -Identity $MainLinkIdentity
$MainInputs = $MainObjects + @($MainLinkStamp)
if (Test-NeedsBuild -Output $MainExe -Inputs $MainInputs) {
    Invoke-Checked (@($CC, "-o", $MainTmp) + $MainObjects + $LinkFlags)
    if (Test-Path $MainExe) {
        Remove-Item -Force $MainExe
    }
    Move-Item -Force $MainTmp $MainExe
} else {
    Write-Host "[skip] main.exe is up to date"
}

$Dll = "sokoban_solver.dll"
$DllTmp = Join-Path $BuildDir "sokoban_solver.dll.tmp"
$Def = "sokoban_solver.def"
$DllObjects = @($Objects["pc_flash"], $Objects["astar"], $Objects["sokoban_flash"], $Objects["sokoban_scan"], $Objects["sokoban_solver"], $Objects["sokoban_recovery"])
$DllLinkStamp = Join-Path $BuildDir "sokoban_solver.link_identity.stamp"
$DllOutputHashStamp = Join-Path $BuildDir "sokoban_solver.output_hash.stamp"
$DllLinkIdentity = @(
    "target=sokoban_solver.dll"
    "profile=$Profile"
    "obj_profile=$ObjProfile"
    "cc=$CC"
    "cflags=$($CFlags -join ' ')"
    "linkflags=$($LinkFlags -join ' ')"
    "objects=$($DllObjects -join '|')"
    "def=$Def"
) -join "`n"
Update-LinkStamp -Stamp $DllLinkStamp -Identity $DllLinkIdentity
$DllInputs = $DllObjects + @($Def, $DllLinkStamp)
$DllNeedsBuild = (Test-NeedsBuild -Output $Dll -Inputs $DllInputs) -or
    -not (Test-OutputHashMatches -Output $Dll -HashStamp $DllOutputHashStamp)
if ($DllNeedsBuild) {
    Invoke-Checked (@($CC, "-shared", "-o", $DllTmp) + $DllObjects + @($Def) + $LinkFlags)
    if (Test-Path $Dll) {
        Remove-Item -Force $Dll
    }
    Move-Item -Force $DllTmp $Dll
    Update-OutputHashStamp -Output $Dll -HashStamp $DllOutputHashStamp
} else {
    Write-Host "[skip] sokoban_solver.dll is up to date"
}

