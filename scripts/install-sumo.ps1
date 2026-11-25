$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$runtimeTarget = Join-Path $projectRoot 'build/sumo-runtime'
python -m pip install --target $runtimeTarget --requirement (Join-Path $projectRoot 'requirements-sumo.txt')
if ($LASTEXITCODE -ne 0) { throw 'SUMO runtime installation failed.' }
& (Join-Path $runtimeTarget 'sumo/bin/sumo.exe') --version
if ($LASTEXITCODE -ne 0) { throw 'Installed SUMO runtime could not start.' }
