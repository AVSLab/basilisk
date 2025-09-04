param(
  [Parameter(Mandatory=$true)][string]$Wheel,
  [Parameter(Mandatory=$true)][string]$DestDir
)

$ErrorActionPreference = 'Stop'
$Project = $env:CIBW_PROJECT_DIR
$BuildBin = Join-Path $Project 'build\bin'
$DistBsk  = Join-Path $Project 'dist3\Basilisk'

Write-Host "=== ENV ==="
Write-Host "Wheel:      $Wheel"
Write-Host "DestDir:    $DestDir"
Write-Host "ProjectDir: $Project"
Write-Host "BuildBin:   $BuildBin"
Write-Host "DistBsk:    $DistBsk"
Write-Host "PATH(before)=" $env:PATH

Write-Host "=== LIST DLL SOURCES (dist3\\Basilisk, build\\bin) ==="
if (Test-Path $DistBsk)  { Get-ChildItem $DistBsk  -Recurse -Filter *.dll | ForEach-Object { $_.FullName } }
if (Test-Path $BuildBin) { Get-ChildItem $BuildBin -Recurse -Filter *.dll | ForEach-Object { $_.FullName } }

Write-Host "=== WHEEL CONTENTS (DLL/PYD) BEFORE REPAIR ==="
python - << 'PY'
import sys, zipfile, os
w = r"""%WHEEL%""".replace('%WHEEL%', r'%WHEEL%')  # placeholder, we'll replace below
w = r'''__WHEEL__'''
with zipfile.ZipFile(w) as z:
    names = z.namelist()
    dlls = [n for n in names if n.lower().endswith('.dll')]
    pyds = [n for n in names if n.lower().endswith('.pyd')]
    for title, items in (('DLLs', dlls), ('PYDs', pyds)):
        print(f'[{title}] {len(items)}')
        for s in sorted(items):
            print('  ', s)
PY | ForEach-Object { $_ -replace '__WHEEL__', $Wheel }

Write-Host "=== IMPORTS FROM KEY PYD (dump via pefile) ==="
python - << 'PY'
import sys, zipfile, io
w = r'''__WHEEL__'''
target = 'Basilisk/architecture/_alg_contain.pyd'
try:
    import pefile
except ImportError:
    import subprocess; subprocess.check_call([sys.executable, '-m', 'pip', 'install', 'pefile'])
    import pefile
with zipfile.ZipFile(w) as z:
    data = z.read(target)
    pe = pefile.PE(data=data)
    if hasattr(pe, 'DIRECTORY_ENTRY_IMPORT'):
        for entry in pe.DIRECTORY_ENTRY_IMPORT:
            print(entry.dll.decode('ascii', errors='ignore'))
    else:
        print('(no import directory)')
PY | ForEach-Object { $_ -replace '__WHEEL__', $Wheel }

Write-Host "=== DELVEWHEEL SHOW (with and without add-path) ==="
python -m pip show delvewheel
python -m delvewheel show -v $Wheel
$env:PATH = "$DistBsk;$BuildBin;$env:PATH"
Write-Host "PATH(after)=$($env:PATH)"
python -m delvewheel show -v --add-path "$DistBsk;$BuildBin" $Wheel

Write-Host "=== REPAIR (keep extracted tree) ==="
python -m delvewheel repair -v `
  --extract-dir (Join-Path $DestDir 'extract') `
  --add-path "$DistBsk;$BuildBin" `
  -w $DestDir `
  $Wheel

Write-Host "=== LIST REPAIRED WHEEL CONTENTS (DLLs) ==="
$repaired = Get-ChildItem $DestDir -Filter *.whl | Select-Object -First 1
python - << 'PY'
import sys, zipfile, os
w = r'''__WHEEL__'''
with zipfile.ZipFile(w) as z:
    libs = [n for n in z.namelist() if n.endswith('.libs/') or n.lower().endswith('.dll')]
    for s in sorted(libs):
        print(s)
PY | ForEach-Object { $_ -replace '__WHEEL__', $repaired.FullName }
