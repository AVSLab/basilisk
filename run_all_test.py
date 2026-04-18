import subprocess

subprocess.run(["ctest", "-C", "Release"], cwd="dist3", check=True)
subprocess.run(["pytest", "-n", "auto"], cwd="src", check=True)
