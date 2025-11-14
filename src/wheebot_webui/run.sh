#!/bin/bash
set -e

echo "Menjalankan backend (FastAPI) di port 8000..."
cd ~/wheebot_ws/src/wheebot_webui/backend
nohup python3 main.py > backend.log 2>&1 &
BACK_PID=$!

sleep 3

echo "Menjalankan frontend (HTTP server) di port 8080..."
cd ../frontend
nohup python3 -m http.server 8080 > frontend.log 2>&1 &
FRONT_PID=$!

echo ""
echo "✅ WheeBot WebUI berjalan:"
echo "🚀 Backend  → http://localhost:8000"
echo "🌐 Frontend → http://localhost:8080"
echo ""
echo "Tekan [Ctrl+C] untuk menghentikan keduanya."

trap "echo '🛑 Menghentikan semua proses...'; kill $BACK_PID $FRONT_PID" SIGINT

wait
