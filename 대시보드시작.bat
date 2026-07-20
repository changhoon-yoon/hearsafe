@echo off
chcp 65001 >nul
rem ============================================
rem  DOA 대시보드 서버 시작 (기본: 무선 WiFi 모드)
rem  - ESP32가 pixar 2.4G 접속 후 doa-node.local:9000 에서 대기
rem  - 브라우저: http://localhost:8765
rem  - USB 유선으로 쓰려면 아랫줄의 "--net doa-node.local" 을 지우고 실행
rem ============================================
cd /d %~dp0
python dashboard\serve.py --net doa-node.local
pause
