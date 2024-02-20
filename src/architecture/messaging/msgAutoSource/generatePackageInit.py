import sys
from pathlib import Path

if __name__ == "__main__":
    moduleOutputPath = Path(sys.argv[1])
    moduleOutputPath.mkdir(parents=True, exist_ok=True)
    with (moduleOutputPath/'__init__.py').open('w') as mainImportFid:
        for file in moduleOutputPath.glob(f"*.py"):
            className = file.stem
            if className != "__init__":
                mainImportFid.write('from Basilisk.architecture.messaging.' + className + ' import *\n')