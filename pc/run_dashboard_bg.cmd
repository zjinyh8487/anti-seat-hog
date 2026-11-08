@echo off
rem 后台起大屏服务（无控制台窗口，关掉浏览器也不会死）
set W=D:\Users\zhangjinyu\AppData\Local\Programs\Coopanion\data\home\companion\workspace\tasks\esp32\pc
cd /d "%W%"
where pythonw >nul 2>&1 && (start "" pythonw dashboard_server.py 8899 & goto :open)
where py >nul 2>&1 && (start "" py -3w dashboard_server.py 8899 & goto :open)
where python >nul 2>&1 && (start "" python dashboard_server.py 8899 & goto :open)
echo NO PYTHON FOUND
pause
exit /b 1
:open
timeout /t 3 /nobreak >nul
start "" http://127.0.0.1:8899/
