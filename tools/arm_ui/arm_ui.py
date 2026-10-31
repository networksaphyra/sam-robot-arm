"""Browser control panel for SAM. Keeps the ESP32's serial port open and serves a page at http://127.0.0.1:8765
with a slider per joint, nudge buttons, limp switches, home and rest poses, and saved-pose playback. Other scripts
can POST a firmware command (e.g. "s 1500") to /cmd.

Opening the serial port resets the ESP32, so every joint starts limp.

    pip install -r requirements.txt
    python3 arm_ui.py --port /dev/cu.usbserial-0001   # the ESP32's port on your machine
"""
import argparse
import json
import re
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import serial

JOINT = re.compile(r'^([ysewg])\s+GPIO(\d+)\s+(on|limp|rest)\s+at\s+(\d+)\s+target\s+(\d+)\s+limits\s+(\d+)\.\.\s*(\d+)\s+home\s+(\d+)')
POSE = re.compile(r'^pose\s+(\d+): y (\d+)\s+s (\d+)\s+e (\d+)\s+w (\d+)\s+g (\d+)')
SPEED = re.compile(r'^speed (\d+) us/s, dwell (\d+) ms')
NAMES = {'y': 'Base yaw', 's': 'Shoulder', 'e': 'Elbow', 'w': 'Wrist pitch', 'g': 'Gripper'}


class Arm:
    def __init__(self, port):
        self.s = serial.Serial()
        self.s.port, self.s.baudrate, self.s.timeout = port, 115200, 0.1
        self.s.dtr = False
        self.s.rts = False
        self.s.open()
        self.lock = threading.Lock()
        self.joints, self.poses, self.speed, self.log = {}, {}, None, []
        self._poses_seen = {}
        threading.Thread(target=self._read, daemon=True).start()
        threading.Thread(target=self._poll, daemon=True).start()

    def send(self, line):
        line = line.strip()
        if line:
            with self.lock:
                self.s.write((line + '\n').encode())
                if line != 'p':
                    self.log.append('> ' + line)

    def _read(self):
        buf = b''
        while True:
            buf += self.s.read(4096)
            while b'\n' in buf:
                raw, buf = buf.split(b'\n', 1)
                line = raw.decode(errors='ignore').strip()
                m = JOINT.match(line)
                if m:
                    k = m.group(1)
                    self.joints[k] = {'pin': int(m.group(2)), 'on': m.group(3) != 'limp', 'state': m.group(3), 'at': int(m.group(4)),
                                      'target': int(m.group(5)), 'lo': int(m.group(6)), 'hi': int(m.group(7)),
                                      'home': int(m.group(8)), 'name': NAMES[k]}
                    if k == 'y':
                        self._poses_seen = {}
                    continue
                m = POSE.match(line)
                if m:
                    self._poses_seen[int(m.group(1))] = [int(x) for x in m.groups()[1:]]
                    continue
                m = SPEED.match(line)
                if m:
                    self.speed = int(m.group(1))
                    self.poses = dict(self._poses_seen)
                    continue
                if line and not line.startswith(('ets ', 'rst:', 'configsip', 'clk_drv', 'mode:', 'load:', 'entry ', 'ho ')):
                    self.log.append(line)
                    self.log = self.log[-200:]

    def _poll(self):
        while True:
            self.send('p')
            time.sleep(0.4)


PAGE = r"""<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>SAM</title><style>
body{font:15px system-ui,sans-serif;margin:0;background:#15171a;color:#e8e8e8}
header{display:flex;gap:10px;align-items:center;padding:12px 16px;background:#1d2024;position:sticky;top:0;z-index:2}
h1{font-size:17px;margin:0 12px 0 0}
button{font:inherit;border:0;border-radius:7px;padding:8px 12px;background:#2d3238;color:#e8e8e8;cursor:pointer}
button:hover{background:#3a4047}.stop{background:#b3261e;font-weight:700;padding:10px 18px}.stop:hover{background:#d2332a}
.go{background:#22663a}.go:hover{background:#2c7d48}
main{display:grid;grid-template-columns:minmax(0,1fr) 340px;gap:16px;padding:16px}
.joint{background:#1d2024;border-radius:10px;padding:12px 14px;margin-bottom:12px}
.row{display:flex;align-items:center;gap:8px}.name{font-weight:600;width:110px}
.badge{font-size:12px;padding:2px 8px;border-radius:10px;background:#444}.on{background:#22663a}
input[type=range]{flex:1;accent-color:#5aa9ff}.val{width:120px;text-align:right;font-variant-numeric:tabular-nums}
.small{font-size:12px;color:#9aa}.nud button{padding:5px 9px}
#log{height:220px;overflow:auto;background:#0e1012;border-radius:8px;padding:8px;font:12px ui-monospace,monospace;white-space:pre-wrap}
.card{background:#1d2024;border-radius:10px;padding:12px 14px;margin-bottom:12px}
.poses button{margin:3px}input[type=number]{width:70px;font:inherit;background:#0e1012;color:#e8e8e8;border:1px solid #333;border-radius:6px;padding:6px}
</style></head><body>
<header><h1>SAM</h1>
<button class="stop" onclick="cmd('off')" title="Space">ALL LIMP (space)</button>
<button onclick="cmd('home')">Home (horizontal)</button>
<button onclick="cmd('s 1960');cmd('e '+state.joints.e.home);cmd('w '+state.joints.w.home)">Rest pose (arm up)</button>
<span style="flex:1"></span><span class="small">speed</span>
<input id="spd" type="range" min="50" max="1500" step="50" style="width:160px" onchange="cmd('speed '+this.value)">
<span id="spdv" class="val" style="width:70px"></span></header>
<main><section id="joints"></section><aside>
<div class="card"><b>Poses</b><div class="small">Save the current pose to a slot; play runs slots in order.</div>
<div class="row" style="margin-top:8px"><input id="slot" type="number" min="0" max="19" value="0">
<button onclick="cmd('save '+slot.value);slot.value=+slot.value+1">Save</button>
<button class="go" onclick="cmd('play')">Play once</button><button class="go" onclick="cmd('loop')">Loop</button>
<button onclick="cmd('stop')">Stop</button></div>
<div class="row" style="margin-top:6px"><span class="small">dwell ms</span><input id="dw" type="number" value="500" step="100"><button onclick="cmd('dwell '+dw.value)">Set</button>
<button onclick="if(confirm('Forget all saved poses?'))cmd('clear')">Clear all</button></div>
<div id="poses" class="poses"></div></div>
<div class="card"><b>Command</b><div class="row"><input id="raw" style="flex:1;font:inherit;background:#0e1012;color:#e8e8e8;border:1px solid #333;border-radius:6px;padding:6px" placeholder="e.g. lim w 800 2200" onkeydown="if(event.key=='Enter'){cmd(this.value);this.value=''}"></div></div>
<div class="card"><b>Log</b><div id="log"></div></div></aside></main>
<script>
const ORDER=['y','s','e','w','g'];let state={joints:{}},dragging={},last={};
async function cmd(c){await fetch('/cmd',{method:'POST',body:c});}
function send(k,v){const t=Date.now();if(last[k]&&t-last[k]<120)return;last[k]=t;cmd(k+' '+v);}
function build(){const box=document.getElementById('joints');box.innerHTML='';
 for(const k of ORDER){const d=document.createElement('div');d.className='joint';d.innerHTML=
 `<div class="row"><span class="name">${k.toUpperCase()} &middot; <span id="n_${k}"></span></span><span id="b_${k}" class="badge">limp</span>
 <span class="small" id="lim_${k}"></span><span style="flex:1"></span>
 <span class="nud"><button onclick="cmd('${k}--')">&minus;50</button><button onclick="cmd('${k}-')">&minus;10</button>
 <button onclick="cmd('${k}+')">+10</button><button onclick="cmd('${k}++')">+50</button>
 <button onclick="cmd('${k} '+state.joints['${k}'].home)">home</button><button onclick="cmd('off ${k}')">limp</button></span></div>
 <div class="row" style="margin-top:6px"><input type="range" id="r_${k}" step="5"
  onpointerdown="dragging['${k}']=1" onpointerup="dragging['${k}']=0;cmd('${k} '+this.value)" oninput="send('${k}',this.value);v_${k}.textContent=this.value+' µs (target)'">
 <span class="val" id="v_${k}"></span></div>`;box.appendChild(d);}}
async function tick(){try{const s=await (await fetch('/state')).json();state=s;
 for(const k of ORDER){const j=s.joints[k];if(!j)continue;const r=document.getElementById('r_'+k);
  r.min=j.lo;r.max=j.hi;if(!dragging[k])r.value=j.target;
  document.getElementById('n_'+k).textContent=j.name;
  const b=document.getElementById('b_'+k);b.textContent=j.state;b.className='badge'+(j.on?' on':'');
  document.getElementById('lim_'+k).textContent=`${j.lo}–${j.hi}, home ${j.home}`;
  if(!dragging[k])document.getElementById('v_'+k).textContent=`${j.at} µs`+(j.at!=j.target?` → ${j.target}`:'');}
 if(s.speed){spd.value=s.speed;spdv.textContent=s.speed+' µs/s';}
 const pz=document.getElementById('poses');pz.innerHTML='';
 for(const [n,v] of Object.entries(s.poses)){const b=document.createElement('button');b.textContent=`go ${n}`;
  b.title=`y ${v[0]} s ${v[1]} e ${v[2]} w ${v[3]} g ${v[4]}`;b.onclick=()=>cmd('go '+n);pz.appendChild(b);}
 const lg=document.getElementById('log');const atEnd=lg.scrollTop+lg.clientHeight>=lg.scrollHeight-4;
 lg.textContent=s.log.join('\n');if(atEnd)lg.scrollTop=lg.scrollHeight;}catch(e){}}
document.addEventListener('keydown',e=>{if(e.code=='Space'&&e.target.tagName!='INPUT'){e.preventDefault();cmd('off');}});
build();setInterval(tick,300);tick();
</script></body></html>"""


def serve(arm, host, port):
    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, body, ctype):
            data = body.encode() if isinstance(body, str) else body
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path == '/':
                self._send(200, PAGE, 'text/html; charset=utf-8')
            elif self.path == '/state':
                self._send(200, json.dumps({'joints': arm.joints, 'poses': arm.poses, 'speed': arm.speed,
                                            'log': arm.log[-80:]}), 'application/json')
            else:
                self._send(404, 'not found', 'text/plain')

        def do_POST(self):
            if self.path == '/cmd':
                n = int(self.headers.get('Content-Length', 0))
                arm.send(self.rfile.read(n).decode(errors='ignore'))
                self._send(200, 'ok', 'text/plain')
            else:
                self._send(404, 'not found', 'text/plain')

    print('arm UI on http://%s:%d' % (host, port), flush=True)
    ThreadingHTTPServer((host, port), H).serve_forever()


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='/dev/cu.usbserial-0001')
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--http', type=int, default=8765)
    a = ap.parse_args()
    serve(Arm(a.port), a.host, a.http)
