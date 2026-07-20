@echo off
chcp 65001 >nul
rem ============================================
rem  DOA 대시보드 서버 - 무선(WiFi) 모드
rem  ESP32가 pixar 2.4G 접속 후 doa-node.local:9000 대기
rem  (이름 해석 실패 시 아래 주소를 보드 IP로 직접 수정:
rem   예: python dashboard\serve.py --net 192.168.0.52)
rem ============================================
cd /d %~dp0
python dashboard\serve.py --net doa-node.local
pause
