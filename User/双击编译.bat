@echo off
setlocal
cd /d "%~dp0"

REM Fast incremental build. Use --force-close only after confirming a project process is locking the DLL.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0build_fast.ps1" %*
set "BUILD_RESULT=%ERRORLEVEL%"

echo.
if "%BUILD_RESULT%"=="0" (
    echo ============================================
    echo Build succeeded.
    echo Outputs: main.exe, sokoban_solver.dll
    echo Run test: python demo.py
    echo ============================================
) else (
    echo ============================================
    echo Build failed. See messages above.
    echo If sokoban_solver.dll is locked, close this project's demo.py or run:
    echo powershell -NoProfile -ExecutionPolicy Bypass -File .\build_fast.ps1 --force-close
    echo ============================================
)

echo.
pause
exit /b %BUILD_RESULT%
