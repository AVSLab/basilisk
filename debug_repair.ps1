param(
  [Parameter(Mandatory = $true)] [string] $Wheel,
  [Parameter(Mandatory = $true)] [string] $DestDir
)

$ErrorActionPreference = 'Stop'

# ---- Paths we care about ----
$Project  = $env:CIBW_PROJECT_DIR
$BuildBin = Join-Path $Project 'build\bin'
$DistBsk  = Join-Path $Project 'dist3\Basilisk'

# ---- Helper to run an inline Python snippet safely in PowerShell ----
function Invoke-Py {
  param(
    [Parameter(Mandatory = $true)] [string] $Code,
    [string[]] $Args = @()
  )
  $tmp = [System.IO.Path]::ChangeExtension([System.IO.Path]::GetTempFileName(), '.py')
  try {
    # Use UTF8 w/o BOM so Python won't choke on the encoding marker
    [System.IO.File]::WriteAllText($tmp, $Code, (New-Object System.Text.UTF8Encoding($false)))
    & python $tmp @Args
  }
  finally {
    Remove-Item -Force -ErrorAction SilentlyContinue $tmp
  }
}

Write-Host "=== ENV ==="
Write-Host "Wheel:      $Wheel"
Write-Host "DestDir:    $DestDir"
Write-Host "ProjectDir: $Project"
Write-Host "BuildBin:   $BuildBin"
Write-Host "DistBsk:    $DistBsk"
Write-Host "PATH(before)= $($env:PATH)"
Write-Host ""

Write-Host "=== LIST DLL SOURCES (dist3\Basilisk, build\bin) ==="
if (Test-Path $DistBsk)  { Get-ChildItem $DistBsk  -Recurse -Filter *.dll | ForEach-Object { $_.FullName } }
if (Test-Path $BuildBin) { Get-ChildItem $BuildBin -Recurse -Filter *.dll | ForEach-Object { $_.FullName } }
Write-Host ""

# ---- Wheel contents BEFORE repair ----
Write-Host "=== WHEEL CONTENTS (DLL/PYD) BEFORE REPAIR ==="
$pyWheelList = @'
import sys, zipfile
w = sys.argv[1]
with zipfile.ZipFile(w) as z:
    names = z.namelist()
    dlls = [n for n in names if n.lower().endswith(".dll")]
    pyds = [n for n in names if n.lower().endswith(".pyd")]
    for title, items in (("DLLs", dlls), ("PYDs", pyds)):
        print(f"[{title}] {len(items)}")
        for s in sorted(items):
            print("  ", s)
'@
Invoke-Py -Code $pyWheelList -Args @($Wheel)
Write-Host ""

# ---- Imports from the key PYD, via pefile ----
Write-Host "=== IMPORTS FROM KEY PYD (dump via pefile) ==="
$pyDumpImports = @'
import sys, zipfile, pefile
w = sys.argv[1]
target = "Basilisk/architecture/_alg_contain.pyd"
with zipfile.ZipFile(w) as z:
    data = z.read(target)
    pe = pefile.PE(data=data)
    if hasattr(pe, "DIRECTORY_ENTRY_IMPORT"):
        for entry in pe.DIRECTORY_ENTRY_IMPORT:
            dll = entry.dll
            if isinstance(dll, bytes):
                dll = dll.decode("ascii", errors="ignore")
            print(dll)
    else:
        print("(no import directory)")
'@
# ensure pefile is present
python - <<#[
try:
    import pefile  # noqa
except ImportError:
    import sys, subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "pefile"])
]# 2>$null
Invoke-Py -Code $pyDumpImports -Args @($Wheel)
Write-Host ""

# ---- Delvewheel diagnostics ----
Write-Host "=== DELVEWHEEL SHOW (baseline) ==="
python -m pip show delvewheel
python -m delvewheel show -v "$Wheel"
Write-Host ""

# Add DLL locations to PATH for discovery
$env:PATH = "$DistBsk;$BuildBin;$env:PATH"
Write-Host "PATH(after)= $($env:PATH)"
Write-Host ""

Write-Host "=== DELVEWHEEL SHOW (with --add-path) ==="
python -m delvewheel show -v --add-path "$DistBsk;$BuildBin" "$Wheel"
Write-Host ""

# ---- Repair ----
Write-Host "=== REPAIR (keep extracted tree) ==="
$ExtractDir = Join-Path $DestDir 'extract'
python -m delvewheel repair -v `
  --extract-dir "$ExtractDir" `
  --add-path "$DistBsk;$BuildBin" `
  -w "$DestDir" `
  "$Wheel"
Write-Host ""

# ---- Verify repaired wheel contents ----
Write-Host "=== LIST REPAIRED WHEEL CONTENTS (DLLs + .libs) ==="
$repaired = Get-ChildItem $DestDir -Filter *.whl | Select-Object -First 1
if (-not $repaired) {
  throw "No repaired wheel found in $DestDir"
}
$pyListRepaired = @'
import sys, zipfile
w = sys.argv[1]
with zipfile.ZipFile(w) as z:
    names = z.namelist()
    libs = [n for n in names if n.endswith(".libs/") or n.lower().endswith(".dll")]
    for s in sorted(libs):
        print(s)
'@
Invoke-Py -Code $pyListRepaired -Args @($repaired.FullName)
Write-Host ""
Write-Host "Repaired wheel: $($repaired.FullName)"
