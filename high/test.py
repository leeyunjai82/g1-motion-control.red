#!/usr/bin/env python3
"""
G1 Control Center - dashboard.py(:50002) + run_motion.py(:50003) iframe 통합
"""
from fastapi import FastAPI
from fastapi.responses import HTMLResponse
import uvicorn

app = FastAPI()

HTML = """<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<title>G1 Control Center</title>
<style>
  *{box-sizing:border-box;margin:0;padding:0}
  html,body{height:100%;background:#0a0a0f;color:#eee;
            font-family:-apple-system,"Segoe UI",sans-serif}
  body{display:flex;flex-direction:column;padding:10px;gap:10px}
  header{color:#f9c300;font-size:14px;padding:4px 8px;font-weight:600}
  .row{display:flex;flex:1;gap:10px;min-height:0}
  .panel{flex:1;display:flex;flex-direction:column;
         background:#1c1c1c;border:1px solid #2a2a2a;border-radius:8px;
         overflow:hidden;min-width:0}
  .panel h2{font-size:13px;padding:8px 12px;background:#232323;
            border-bottom:1px solid #2a2a2a;color:#9cf;font-weight:500}
  iframe{flex:1;border:0;background:#000;min-height:0;width:100%}
</style>
</head>
<body>
  <header>🤖 G1 Control Center</header>
  <div class="row">
    <div class="panel">
      <h2>Dashboard — :50002/dashboard</h2>
      <iframe id="if-dash"></iframe>
    </div>
    <div class="panel">
      <h2>Motion Runner — :50003/robot-only</h2>
      <iframe id="if-motion"></iframe>
    </div>
  </div>
<script>
  // 접속한 호스트(PC의 IP)를 그대로 사용 → IP 바뀌어도 OK
  const host = window.location.hostname;
  document.getElementById('if-dash').src   = `http://${host}:50002/dashboard`;
  document.getElementById('if-motion').src = `http://${host}:50003/robot-only`;
</script>
</body>
</html>"""

@app.get("/")
def index():
    return HTMLResponse(HTML)

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8080)
