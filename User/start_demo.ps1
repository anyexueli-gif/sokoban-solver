$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RootDir = Split-Path -Parent $ScriptDir
$Python = (Get-Command python -ErrorAction Stop).Source

Push-Location $RootDir
try {
    & $Python (Join-Path $ScriptDir "demo.py") --attached
    exit $LASTEXITCODE
}
finally {
    Pop-Location
}
