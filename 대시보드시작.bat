@echo off
chcp 65001 >nul
rem ============================================
rem  DOA 대시보드 서버 시작 (더블클릭 실행용)
rem  - 이 창을 닫으면 서버도 꺼집니다
rem  - 브라우저: http://localhost:8765
rem ============================================
cd /d %~dp0
python dashboard\serve.py
pause
