from os import system

system('cd dist3 && ctest -C Release')
system('cd src && pytest -n auto')
