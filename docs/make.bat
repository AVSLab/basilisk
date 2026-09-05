@ECHO OFF

pushd "%~dp0"

REM Command file for Sphinx documentation

if "%SPHINXBUILD%" == "" (
	set SPHINXBUILD=sphinx-build
)
set SOURCEDIR=source
set BUILDDIR=build

if "%1" == "" goto help

if /I "%1"=="clean" goto clean

if /I "%1"=="comparison-runtime-tables" (
	py -3 generateDynamicsComparisonRuntimeTables.py || python generateDynamicsComparisonRuntimeTables.py
	if errorlevel 1 exit /b 1
	goto end
)

%SPHINXBUILD% >NUL 2>NUL
if errorlevel 9009 (
	echo.
	echo.The 'sphinx-build' command was not found. Make sure you have Sphinx
	echo.installed, then set the SPHINXBUILD environment variable to point
	echo.to the full path of the 'sphinx-build' executable. Alternatively you
	echo.may add the Sphinx directory to PATH.
	echo.
	echo.If you don't have Sphinx installed, grab it from
	echo.http://sphinx-doc.org/
	exit /b 1
)

if /I "%1"=="html" (
	py -3 source\Support\bskReleaseNotesSnippets\_compile_release_notes_snippets.py || python source\Support\bskReleaseNotesSnippets\_compile_release_notes_snippets.py
	if errorlevel 1 exit /b 1
)

%SPHINXBUILD% -M %1 %SOURCEDIR% %BUILDDIR% %SPHINXOPTS% %O%
goto end

:help
%SPHINXBUILD% -M help %SOURCEDIR% %BUILDDIR% %SPHINXOPTS% %O%
goto end

:clean
if exist "%BUILDDIR%" (
	rmdir /S /Q "%BUILDDIR%"
	if errorlevel 1 goto clean_failed
)
if exist "%SOURCEDIR%\Documentation" (
	rmdir /S /Q "%SOURCEDIR%\Documentation"
	if errorlevel 1 goto clean_failed
)
if exist "%SOURCEDIR%\breathe.data" (
	del /F /Q "%SOURCEDIR%\breathe.data"
	if errorlevel 1 goto clean_failed
)
if exist "%SOURCEDIR%\externalTools" (
	rmdir /S /Q "%SOURCEDIR%\externalTools"
	if errorlevel 1 goto clean_failed
)
if exist "%SOURCEDIR%\examples" (
	rmdir /S /Q "%SOURCEDIR%\examples"
	if errorlevel 1 goto clean_failed
)
goto end

:clean_failed
set "CLEAN_EXIT_CODE=%ERRORLEVEL%"
echo Failed to clean generated documentation.
popd
exit /b %CLEAN_EXIT_CODE%

:end
popd
