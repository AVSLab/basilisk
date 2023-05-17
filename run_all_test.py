from os import system

system('cd src && pytest -n auto')
system('cd dist3 && ctest -C Release')
