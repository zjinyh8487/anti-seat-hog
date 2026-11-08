@echo off
set W=D:\Users\zhangjinyu\AppData\Local\Programs\Coopanion\data\home\companion\workspace\tasks\esp32\pc
cd /d "%W%"
where py >nul 2>&1 && (start "" py -3 dashboard_server.py 8899 & goto :open)
where python >nul 2>&1 && (start "" python dashboard_server.py 8899 & goto :open)
echo NO PYTHON FOUND
pause
exit /b 1
:open
timeout /t 2 /nobreak >nul
start "" http://127.0.0.1:8899/
