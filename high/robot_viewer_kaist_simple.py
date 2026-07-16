#!/usr/bin/env python3
"""
G1 URDF Live Motor Visualization Server (Live-only, offline-ready)

Routes:
  /            -> Full UI (3D viewer + Joint States 모터 값)
  /robot-only  -> 3D 뷰어만 (모터 값/사이드바 모두 숨김)
  /api/*       -> URDF / mesh / SSE
  /vendor/three.min.js -> 로컬 Three.js (오프라인 동작용)
"""

import os
import sys
import json
import asyncio
import uvicorn
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse, StreamingResponse, FileResponse, Response

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)

ASSETS_DIR = os.path.join(current_dir, 'assets', 'g1')
URDF_PATH  = os.path.join(ASSETS_DIR, 'g1_29dof_rev_1_0.urdf')
MESH_DIR   = os.path.join(ASSETS_DIR, 'meshes')
VENDOR_DIR = os.path.join(current_dir, 'assets', 'vendor')   # three.min.js 등 로컬 라이브러리

# ==========================================
# Robot connection (unitree_sdk2py 직접 구독)
# ==========================================
NET_INTERFACE = os.environ.get("G1_NET_IFACE", "")  # e.g. "eth0", "eno1", "" = auto

_last_lowstate = None  # 최신 LowState_ 메시지

try:
    from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

    ChannelFactoryInitialize(0, NET_INTERFACE)

    def _on_lowstate(msg):
        global _last_lowstate
        _last_lowstate = msg

    _lowstate_sub = ChannelSubscriber("rt/lowstate", LowState_)
    _lowstate_sub.Init(_on_lowstate, 10)
    ROBOT_AVAILABLE = True
    print(f"[System] Subscribed rt/lowstate (iface='{NET_INTERFACE or 'auto'}')")
except Exception as e:
    ROBOT_AVAILABLE = False
    print(f"[Warning] unitree_sdk2py init failed (simulation mode): {e}")

JOINT_TO_MOTOR = {
    'left_hip_pitch_joint':      0,
    'left_hip_roll_joint':       1,
    'left_hip_yaw_joint':        2,
    'left_knee_joint':           3,
    'left_ankle_pitch_joint':    4,
    'left_ankle_roll_joint':     5,
    'right_hip_pitch_joint':     6,
    'right_hip_roll_joint':      7,
    'right_hip_yaw_joint':       8,
    'right_knee_joint':          9,
    'right_ankle_pitch_joint':  10,
    'right_ankle_roll_joint':   11,
    'waist_yaw_joint':          12,
    'waist_roll_joint':         13,
    'waist_pitch_joint':        14,
    'left_shoulder_pitch_joint':  15,
    'left_shoulder_roll_joint':   16,
    'left_shoulder_yaw_joint':    17,
    'left_elbow_joint':           18,
    'left_wrist_roll_joint':      19,
    'left_wrist_pitch_joint':     20,
    'left_wrist_yaw_joint':       21,
    'right_shoulder_pitch_joint': 22,
    'right_shoulder_roll_joint':  23,
    'right_shoulder_yaw_joint':   24,
    'right_elbow_joint':          25,
    'right_wrist_roll_joint':     26,
    'right_wrist_pitch_joint':    27,
    'right_wrist_yaw_joint':      28,
}

app = FastAPI(title="G1 URDF Viewer")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

# ==========================================
# API
# ==========================================
@app.get('/api/urdf')
def get_urdf():
    return FileResponse(URDF_PATH, media_type='text/xml')

@app.get('/api/meshes')
def list_meshes():
    files = [f for f in os.listdir(MESH_DIR) if f.lower().endswith('.stl')]
    return {'files': files}

@app.get('/api/mesh/{filename}')
def get_mesh(filename: str):
    path = os.path.join(MESH_DIR, filename)
    if not os.path.exists(path):
        for f in os.listdir(MESH_DIR):
            if f.lower() == filename.lower():
                path = os.path.join(MESH_DIR, f)
                break
    return FileResponse(path, media_type='application/octet-stream')

@app.get('/api/joint_states')
async def joint_states():
    async def gen():
        while True:
            msg = _last_lowstate
            if msg is not None:
                q   = [m.q for m in msg.motor_state]
                imu = list(msg.imu_state.rpy)
                connected = True
            else:
                q   = [0.0] * 35
                imu = [0.0, 0.0, 0.0]
                connected = False
            data = {j: float(q[i]) for j, i in JOINT_TO_MOTOR.items()}
            data['_imu']       = imu
            data['_connected'] = connected
            yield f"data: {json.dumps(data)}\n\n"
            await asyncio.sleep(0.05)
    return StreamingResponse(
        gen(),
        media_type='text/event-stream',
        headers={'Cache-Control': 'no-cache', 'X-Accel-Buffering': 'no'}
    )

@app.get('/api/status')
def status():
    return {'connected': _last_lowstate is not None}

# ==========================================
# Local vendor (Three.js 등)
# ==========================================
@app.get('/vendor/three.min.js')
def vendor_three():
    path = os.path.join(VENDOR_DIR, 'three.min.js')
    if not os.path.exists(path):
        return Response(
            content=f"// three.min.js not found at {path}\n"
                    f"// 다음 명령으로 다운로드:\n"
                    f"//   curl -o {path} https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js",
            media_type='application/javascript',
            status_code=404,
        )
    return FileResponse(path, media_type='application/javascript')

# ==========================================
# HTML (3D viewer)
# ==========================================
HTML_PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>G1 URDF Live Viewer</title>
<script src="/vendor/three.min.js"></script>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#0a0a0f;color:#d0d0d8;font-family:'Segoe UI',system-ui,sans-serif;display:flex;flex-direction:column;height:100vh;overflow:hidden;font-size:13px}
header{background:#0f0f1a;border-bottom:1px solid #f9c30030;padding:0 16px;display:flex;align-items:center;gap:12px;height:44px;flex-shrink:0}
header h1{font-size:14px;color:#f9c300;font-weight:600;letter-spacing:.5px}
.badge{font-size:10px;padding:2px 8px;border-radius:10px;background:#1a1a2a;color:#555;border:1px solid #222}
.badge.ok{color:#4caf80;border-color:#4caf5044}
.badge.live{color:#f9c300;border-color:#f9c30044;animation:pulse 1.5s infinite}
.badge.err{color:#ff6666;border-color:#ff444444}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.5}}
.main{display:flex;flex:1;overflow:hidden;min-height:0}
.left{width:210px;background:#0d0d16;border-right:1px solid #1a1a28;display:flex;flex-direction:column;flex-shrink:0;overflow-y:auto}
.sec{padding:10px 12px;border-bottom:1px solid #1a1a28}
.sec-title{font-size:10px;text-transform:uppercase;color:#444;letter-spacing:1.2px;margin-bottom:8px}
.tb{display:flex;align-items:center;gap:7px;width:100%;padding:6px 9px;background:#12121e;border:1px solid #1e1e2e;border-radius:7px;color:#666;font-size:11px;cursor:pointer;margin-bottom:5px;transition:all .15s;text-align:left}
.tb:hover{border-color:#f9c30033;color:#d5c599}
.tb.on{background:#f9c30010;border-color:#f9c30055;color:#f9c300}
.tb .ic{width:13px;flex-shrink:0;text-align:center;font-size:11px}
.info-row{display:flex;justify-content:space-between;font-size:11px;padding:2px 0;color:#444}
.info-val{color:#666}
.imu-row{font-size:11px;padding:3px 0;color:#6a6a2a}
.status-dot{width:7px;height:7px;border-radius:50%;background:#333;flex-shrink:0}
.status-dot.on{background:#4caf80;box-shadow:0 0 5px #4caf8088}
.status-dot.err{background:#ff4444}
.viewport{flex:1;position:relative;overflow:hidden;background:#0a0a0f}
canvas#cv{display:block;width:100%!important;height:100%!important}
.hud{position:absolute;bottom:10px;left:10px;font-size:10px;color:#282838;pointer-events:none;line-height:1.9}
.load-overlay{position:absolute;inset:0;background:#0a0a0fdd;display:flex;flex-direction:column;align-items:center;justify-content:center;gap:14px}
.load-title{font-size:15px;color:#f9c300;font-weight:500}
.pbar-bg{width:280px;height:5px;background:#1a1a28;border-radius:3px}
.pbar-fill{height:100%;background:#f9c300;border-radius:3px;transition:width .07s;width:0%}
.pbar-text{font-size:11px;color:#555}
.tooltip{position:absolute;background:#14141e;border:1px solid #2a2a3a;border-radius:6px;padding:5px 10px;font-size:11px;color:#aaa;pointer-events:none;z-index:100;display:none}
.right{width:275px;background:#0d0d16;border-left:1px solid #1a1a28;display:flex;flex-direction:column;flex-shrink:0}
.right-head{padding:8px 12px;border-bottom:1px solid #1a1a28;display:flex;align-items:center;justify-content:space-between;gap:7px}
.right-head b{font-size:12px;font-weight:600;color:#aaa;white-space:nowrap}
.search-box{flex:1;background:#12121e;border:1px solid #1e1e2e;border-radius:5px;padding:4px 8px;color:#aaa;font-size:11px;outline:none;min-width:0}
.search-box::placeholder{color:#2a2a3a}
.search-box:focus{border-color:#f9c30044}
.jcount{font-size:10px;color:#444;white-space:nowrap}
.joints{flex:1;overflow-y:auto;padding:7px}
.no-joint{text-align:center;color:#2a2a3a;font-size:11px;padding:40px 16px;line-height:1.8}
.group-header{display:flex;align-items:center;gap:6px;padding:5px 4px;cursor:pointer;color:#444;font-size:10px;text-transform:uppercase;letter-spacing:.8px;user-select:none;margin-top:3px}
.group-header:hover{color:#666}
.garr{font-size:8px;transition:transform .15s;flex-shrink:0}
.garr.open{transform:rotate(90deg)}
.group-body{overflow:hidden}
.ji{margin-bottom:5px;background:#12121e;border-radius:7px;padding:6px 9px;border:1px solid #1a1a28}
.ji-head{display:flex;justify-content:space-between;align-items:center;margin-bottom:4px;gap:4px}
.ji-name{font-size:10px;color:#666;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;flex:1}
.ji-val{font-size:11px;color:#f9c300;font-weight:600;min-width:46px;text-align:right;flex-shrink:0;font-family:monospace}
.ji-bar{position:relative;height:4px;background:#1a1a28;border-radius:2px;overflow:hidden}
.ji-bar-fill{position:absolute;top:0;height:100%;background:#f9c300;transition:left .08s linear,width .08s linear;border-radius:2px}
.ji-bar-zero{position:absolute;top:0;left:50%;width:1px;height:100%;background:#333}
::-webkit-scrollbar{width:3px}
::-webkit-scrollbar-track{background:transparent}
::-webkit-scrollbar-thumb{background:#1e1e2e;border-radius:2px}

/* robot-only mode: 3D 뷰어만 표시 (헤더/좌측/우측 Joint States 모두 숨김) */
body.robot-only-mode header,
body.robot-only-mode .left,
body.robot-only-mode .right,
body.robot-only-mode .hud,
body.robot-only-mode .tooltip { display: none !important; }
body.robot-only-mode .main { flex: 1; }
</style>
</head>
<body>
<header>
  <h1>&#x1F916; G1 Live Viewer</h1>
  <span class="badge" id="statusBadge">Loading...</span>
  <span class="badge live" id="liveBadge" style="display:none">● LIVE</span>
  <span style="margin-left:auto;font-size:10px;color:#2a2a3a" id="imuDisp">IMU: -</span>
</header>

<div class="main">
  <div class="left">
    <div class="sec">
      <div class="sec-title">Connection</div>
      <div style="display:flex;align-items:center;gap:8px">
        <div class="status-dot" id="connDot"></div>
        <span style="font-size:11px;color:#555" id="connText">Checking...</span>
      </div>
    </div>
    <div class="sec">
      <div class="sec-title">View</div>
      <button class="tb on" id="gridBtn"><span class="ic">&#x22DE;</span>Grid</button>
      <button class="tb" id="wireBtn"><span class="ic">&#x25A6;</span>Wireframe</button>
      <button class="tb" id="axisBtn"><span class="ic">&#x2197;</span>Joint Axes</button>
      <button class="tb" id="ssBtn"><span class="ic">&#x1F4F7;</span>Screenshot</button>
    </div>
    <div class="sec" id="infoSec" style="display:none">
      <div class="sec-title">Model Info</div>
      <div id="infoRows"></div>
    </div>
    <div class="sec" id="imuSec" style="display:none">
      <div class="sec-title">IMU (Pelvis)</div>
      <div id="imuRows"></div>
    </div>
  </div>

  <div class="viewport" id="vp">
    <canvas id="cv"></canvas>
    <div class="load-overlay" id="loadOv">
      <div class="load-title">Loading model...</div>
      <div class="pbar-bg"><div class="pbar-fill" id="pbar"></div></div>
      <div class="pbar-text" id="ptext">Loading URDF...</div>
    </div>
    <div class="hud">
      <span id="fpsEl">FPS: --</span>
      <span style="color:#1e1e28">Left-click rotate / Right-click pan / Wheel zoom</span>
    </div>
    <div class="tooltip" id="tt"></div>
  </div>

  <div class="right">
    <div class="right-head">
      <b>Joint States</b>
      <input class="search-box" id="searchBox" placeholder="Search..." type="text">
      <span class="jcount" id="jcount">-</span>
    </div>
    <div class="joints" id="jointsEl">
      <div class="no-joint">Loading...</div>
    </div>
  </div>
</div>

<script>
class Orbit {
  constructor(cam,el){
    this.cam=cam;this.el=el;this.target=new THREE.Vector3(0,.8,0);
    this.phi=1.2;this.theta=0.5;this.r=3.5;
    this._dn=false;this._btn=-1;this._lx=0;this._ly=0;
    el.addEventListener('mousedown',e=>{this._dn=true;this._btn=e.button;this._lx=e.clientX;this._ly=e.clientY;});
    window.addEventListener('mousemove',e=>{
      if(!this._dn)return;
      const dx=e.clientX-this._lx,dy=e.clientY-this._ly;this._lx=e.clientX;this._ly=e.clientY;
      if(this._btn===0){this.theta-=dx*.005;this.phi=Math.max(.04,Math.min(Math.PI-.04,this.phi-dy*.005));}
      else if(this._btn===2){const f=this.r*.0012;const rt=new THREE.Vector3().setFromMatrixColumn(cam.matrix,0);const up=new THREE.Vector3().setFromMatrixColumn(cam.matrix,1);this.target.addScaledVector(rt,-dx*f).addScaledVector(up,dy*f);}
      this.update();
    });
    window.addEventListener('mouseup',()=>this._dn=false);
    el.addEventListener('wheel',e=>{e.preventDefault();this.r=Math.max(.15,Math.min(60,this.r*(e.deltaY>0?1.1:.9)));this.update();},{passive:false});
    el.addEventListener('contextmenu',e=>e.preventDefault());
    this.update();
  }
  update(){const s=Math.sin(this.phi);this.cam.position.set(this.target.x+this.r*s*Math.sin(this.theta),this.target.y+this.r*Math.cos(this.phi),this.target.z+this.r*s*Math.cos(this.theta));this.cam.lookAt(this.target);}
  focusBox(box){const c=box.getCenter(new THREE.Vector3());const sz=box.getSize(new THREE.Vector3()).length();this.target.copy(c);this.r=sz*1.3;this.update();}
}

function parseSTL(buf){
  const dv=new DataView(buf);
  if(buf.byteLength<84)return parseASCII(new TextDecoder().decode(buf));
  const n=dv.getUint32(80,true);
  if(Math.abs(buf.byteLength-(84+n*50))<=4)return parseBin(dv,n);
  return parseASCII(new TextDecoder().decode(buf));
}
function parseBin(dv,n){
  const pos=new Float32Array(n*9),nrm=new Float32Array(n*9);let o=84;
  for(let i=0;i<n;i++){
    const nx=dv.getFloat32(o,true),ny=dv.getFloat32(o+4,true),nz=dv.getFloat32(o+8,true);o+=12;
    for(let j=0;j<3;j++){const b=i*9+j*3;pos[b]=dv.getFloat32(o,true);pos[b+1]=dv.getFloat32(o+4,true);pos[b+2]=dv.getFloat32(o+8,true);nrm[b]=nx;nrm[b+1]=ny;nrm[b+2]=nz;o+=12;}
    o+=2;
  }
  const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.BufferAttribute(pos,3));g.setAttribute('normal',new THREE.BufferAttribute(nrm,3));return g;
}
function parseASCII(txt){
  const pos=[],nrm=[];let nx=0,ny=0,nz=0;
  for(const ln of txt.split('\n')){const l=ln.trim();
    if(l.startsWith('facet normal')){const m=l.match(/normal\s+([\S]+)\s+([\S]+)\s+([\S]+)/);if(m){nx=+m[1];ny=+m[2];nz=+m[3];}}
    else if(l.startsWith('vertex')){const m=l.match(/vertex\s+([\S]+)\s+([\S]+)\s+([\S]+)/);if(m){pos.push(+m[1],+m[2],+m[3]);nrm.push(nx,ny,nz);}}
  }
  const g=new THREE.BufferGeometry();g.setAttribute('position',new THREE.Float32BufferAttribute(pos,3));g.setAttribute('normal',new THREE.Float32BufferAttribute(nrm,3));return g;
}

function parseURDF(xml){
  const doc=new DOMParser().parseFromString(xml,'text/xml');
  const links={},joints={},materials={};

  doc.querySelectorAll('robot > material').forEach(m=>{
    const name=m.getAttribute('name');
    const c=m.querySelector('color')?.getAttribute('rgba');
    if(c){const [r,g,b]=c.split(/\s+/).map(Number);
      materials[name]=(Math.round(r*255)<<16)|(Math.round(g*255)<<8)|Math.round(b*255);}
  });
  materials['white']=0xe8e8e8;
  materials['dark']=0x1e1e1e;

  doc.querySelectorAll('link').forEach(el=>{
    const name=el.getAttribute('name');links[name]={name,visuals:[]};
    el.querySelectorAll('visual').forEach(v=>{
      const me=v.querySelector('geometry mesh');if(!me)return;
      const fn=me.getAttribute('filename')||'';
      const sc=(me.getAttribute('scale')||'1 1 1').split(/\s+/).map(Number);
      const matName=v.querySelector('material')?.getAttribute('name')||'';
      links[name].visuals.push({fn,sc,origin:parseOrig(v.querySelector('origin')),matName});
    });
  });
  doc.querySelectorAll('joint').forEach(el=>{
    const name=el.getAttribute('name'),type=el.getAttribute('type')||'fixed';
    const parent=el.querySelector('parent')?.getAttribute('link')||'';
    const child=el.querySelector('child')?.getAttribute('link')||'';
    const axEl=el.querySelector('axis');
    const axis=(axEl?.getAttribute('xyz')||'0 0 1').split(/\s+/).map(Number);
    const lim=el.querySelector('limit');
    joints[name]={name,type,parent,child,origin:parseOrig(el.querySelector('origin')),axis,
      limit:{lower:lim?+lim.getAttribute('lower'):-3.14,upper:lim?+lim.getAttribute('upper'):3.14}};
  });
  return{links,joints,materials};
}
function parseOrig(el){
  if(!el)return{xyz:[0,0,0],rpy:[0,0,0]};
  return{xyz:(el.getAttribute('xyz')||'0 0 0').split(/\s+/).map(Number),rpy:(el.getAttribute('rpy')||'0 0 0').split(/\s+/).map(Number)};
}
const basename=s=>s.split(/[/\\]/).pop().toLowerCase();
const sid=n=>n.replace(/\W/g,'_');

// ============ Logo: Circulus / Kaist ============
const CIRCULUS_COLOR = '#00AEEF';
const KAIST_COLOR    = '#004A8F';

function makeLogoTextMesh(logoGeo){
  let width=0.10, height=0.025, cx=0.005, cy=0, cz=0;

  if(logoGeo){
    logoGeo.computeBoundingBox();
    const bb=logoGeo.boundingBox;
    width  = (bb.max.y - bb.min.y);
    height = (bb.max.z - bb.min.z);
    cx     = bb.max.x + 0.0008;
    cy     = (bb.min.y + bb.max.y) * 0.5;
    cz     = (bb.min.z + bb.max.z) * 0.5;
  }

  // Shift logo 5cm down (Z is vertical in logo_link local frame)
  cx += 0.005;
  cz -= 0.04;

  // Scale logo plane (text gets bigger by the same factor)
  const SCALE = 3.0;
  width  *= SCALE;
  height *= SCALE;

  const aspect=Math.max(0.3, width/Math.max(height,0.001));
  const canvas=document.createElement('canvas');
  canvas.height=256;
  canvas.width =Math.min(2048, Math.max(256, Math.round(256*aspect)));
  const ctx=canvas.getContext('2d');
  ctx.clearRect(0,0,canvas.width,canvas.height);

  const line1=[
    {text:'Circulus', color:CIRCULUS_COLOR},
  ];
  const line2=[
    {text:'Kaist', color:KAIST_COLOR},
  ];

  let fontSize=Math.round(canvas.height*0.42);
  const setFont=()=>{ctx.font=`900 ${fontSize}px "Segoe UI", Arial, sans-serif`;};
  setFont();
  const measure=(parts)=>parts.reduce((w,p)=>w+ctx.measureText(p.text).width,0);
  const maxW=canvas.width*0.92;
  let w1=measure(line1), w2=measure(line2);
  let widest=Math.max(w1,w2);
  if(widest>maxW){
    fontSize=Math.max(18,Math.floor(fontSize*maxW/widest));
    setFont();
    w1=measure(line1); w2=measure(line2);
  }

  ctx.textBaseline='middle';ctx.textAlign='left';
  ctx.strokeStyle='#000';ctx.lineWidth=Math.max(2,fontSize*0.04);ctx.lineJoin='round';

  function drawLine(parts,totalW,yPos){
    let x=(canvas.width-totalW)/2;
    for(const p of parts){
      ctx.strokeText(p.text,x,yPos);
      ctx.fillStyle=p.color;
      ctx.fillText(p.text,x,yPos);
      x+=ctx.measureText(p.text).width;
    }
  }
  drawLine(line1, w1, canvas.height*0.30);
  drawLine(line2, w2, canvas.height*0.72);

  const tex=new THREE.CanvasTexture(canvas);
  tex.anisotropy=8;tex.needsUpdate=true;
  const mat=new THREE.MeshBasicMaterial({map:tex,transparent:true,depthWrite:false,side:THREE.DoubleSide});
  const plane=new THREE.Mesh(new THREE.PlaneGeometry(width,height),mat);

  plane.rotation.x=Math.PI/2;
  plane.rotation.y=Math.PI/2;
  plane.position.set(cx,cy,cz);
  plane.renderOrder=10;
  return plane;
}

const cv=document.getElementById('cv'),vp=document.getElementById('vp');
const renderer=new THREE.WebGLRenderer({canvas:cv,antialias:true,preserveDrawingBuffer:true});
renderer.setPixelRatio(Math.min(devicePixelRatio,2));
renderer.setClearColor(0x0a0a0f);
const scene=new THREE.Scene();
const camera=new THREE.PerspectiveCamera(45,1,.001,500);
const orbit=new Orbit(camera,cv);
scene.add(new THREE.AmbientLight(0xffffff,.55));
const dl1=new THREE.DirectionalLight(0xffffff,.85);dl1.position.set(5,10,5);scene.add(dl1);
const dl2=new THREE.DirectionalLight(0x8899ff,.3);dl2.position.set(-5,5,-5);scene.add(dl2);
const grid=new THREE.GridHelper(14,28,0x181828,0x181828);scene.add(grid);
const raycaster=new THREE.Raycaster();const mouse=new THREE.Vector2();
function resize(){const w=vp.clientWidth,h=vp.clientHeight;renderer.setSize(w,h,false);camera.aspect=w/h;camera.updateProjectionMatrix();}
resize();new ResizeObserver(resize).observe(vp);

let robotRoot=null,jointObjs={},baseQs={},jointDefs={};
let allMeshes=[],allAxes=[],wireMode=false,axisMode=false;
let selectedMesh=null;
let valEls={},barEls={},jointLimits={};
let liveEvt=null;

let fps=0,fpsT=performance.now();
function renderLoop(now){
  requestAnimationFrame(renderLoop);
  renderer.render(scene,camera);
  fps++;if(now-fpsT>=1000){const el=document.getElementById('fpsEl');if(el)el.textContent='FPS: '+fps;fps=0;fpsT=now;}
}
requestAnimationFrame(renderLoop);

async function loadRobot(){
  document.getElementById('loadOv').style.display='flex';
  const pbar=document.getElementById('pbar'),ptext=document.getElementById('ptext');
  try{
    if(robotRoot){scene.remove(robotRoot);robotRoot=null;}
    jointObjs={};baseQs={};allMeshes=[];allAxes=[];valEls={};barEls={};jointLimits={};

    ptext.textContent='Loading URDF...';pbar.style.width='5%';
    const urdfTxt=await(await fetch('/api/urdf')).text();
    const{links,joints,materials}=parseURDF(urdfTxt);
    jointDefs=joints;

    ptext.textContent='Fetching mesh list...';pbar.style.width='10%';
    const meshListData=await(await fetch('/api/meshes')).json();
    const serverFiles=new Set(meshListData.files.map(f=>f.toLowerCase()));

    const needed=new Set();
    for(const l of Object.values(links))
      for(const v of l.visuals){const b=basename(v.fn);if(serverFiles.has(b))needed.add(b);}

    const logoFn=links['logo_link']?.visuals?.[0]?.fn;
    const logoBase=logoFn?basename(logoFn):null;

    const geos={};const total=needed.size;let loaded=0;
    await Promise.all([...needed].map(async b=>{
      const buf=await(await fetch('/api/mesh/'+b)).arrayBuffer();
      const geo=parseSTL(buf);geo.computeVertexNormals();geos[b]=geo;
      loaded++;pbar.style.width=`${10+(loaded/total)*85}%`;ptext.textContent=`STL ${loaded} / ${total}`;
    }));

    ptext.textContent='Building scene...';pbar.style.width='98%';
    await new Promise(r=>setTimeout(r,0));

    const childSet=new Set(Object.values(joints).map(j=>j.child));
    const rootName=Object.keys(links).find(l=>!childSet.has(l))||Object.keys(links)[0];

    function mkLink(lname){
      const link=links[lname];if(!link)return null;
      const grp=new THREE.Group();grp.name='link:'+lname;

      if(lname==='logo_link'){
        const logoGeo=logoBase?geos[logoBase]:null;
        const txt=makeLogoTextMesh(logoGeo);
        txt.userData={linkName:lname};
        allMeshes.push(txt);grp.add(txt);
      } else {
        for(const v of link.visuals){
          const b=basename(v.fn),geo=geos[b];if(!geo)continue;
          let color=(materials[v.matName]!==undefined)?materials[v.matName]:0xb0b0b0;
          if(/hand|finger|thumb|palm/i.test(lname)) color=0x1a1a1a;
          const isDark=color<0x555555;
          const mat=new THREE.MeshPhongMaterial({color,specular:isDark?0x222222:0x666666,shininess:isDark?55:25});
          const mesh=new THREE.Mesh(geo,mat);mesh.userData={linkName:lname};
          mesh.position.set(...v.origin.xyz);mesh.setRotationFromEuler(new THREE.Euler(...v.origin.rpy,'XYZ'));mesh.scale.set(...v.sc);
          allMeshes.push(mesh);grp.add(mesh);
        }
      }

      const ax=new THREE.AxesHelper(.08);ax.visible=false;allAxes.push(ax);grp.add(ax);
      for(const jt of Object.values(joints).filter(j=>j.parent===lname)){
        const jg=new THREE.Group();jg.name='joint:'+jt.name;
        jg.position.set(...jt.origin.xyz);jg.setRotationFromEuler(new THREE.Euler(...jt.origin.rpy,'XYZ'));
        jointObjs[jt.name]=jg;baseQs[jt.name]=jg.quaternion.clone();
        const child=mkLink(jt.child);if(child)jg.add(child);grp.add(jg);
      }
      return grp;
    }
    robotRoot=mkLink(rootName);
    if(robotRoot){
      robotRoot.rotation.x=-Math.PI/2;
      scene.add(robotRoot);
      const box=new THREE.Box3().setFromObject(robotRoot);
      robotRoot.position.y=-box.min.y;
      orbit.focusBox(new THREE.Box3().setFromObject(robotRoot));
    }
    buildJointDisplays(joints);
    const mv=Object.values(joints).filter(j=>j.type!=='fixed').length;
    const infoSec=document.getElementById('infoSec');if(infoSec)infoSec.style.display='block';
    const infoRows=document.getElementById('infoRows');
    if(infoRows)infoRows.innerHTML=
      [['Links',Object.keys(links).length],['Joints',Object.keys(joints).length],['Movable',mv],['Meshes',allMeshes.length]]
      .map(([k,v])=>`<div class="info-row"><span>${k}</span><span class="info-val">${v}</span></div>`).join('');
    const b=document.getElementById('statusBadge');b.textContent='Ready';b.className='badge ok';
    pbar.style.width='100%';

    await checkConnection();
    startSSE();
    document.getElementById('liveBadge').style.display='';

  }catch(e){console.error(e);document.getElementById('statusBadge').textContent='Error: '+e.message;document.getElementById('statusBadge').className='badge err';}
  document.getElementById('loadOv').style.display='none';
}

async function checkConnection(){
  try{
    const d=await(await fetch('/api/status')).json();
    const dot=document.getElementById('connDot');const txt=document.getElementById('connText');
    if(dot&&txt){
      if(d.connected){dot.className='status-dot on';txt.textContent='Robot connected';txt.style.color='#4caf80';}
      else{dot.className='status-dot err';txt.textContent='Simulation mode';txt.style.color='#ff6666';}
    }
  }catch(e){console.error(e);}
}

function startSSE(){
  if(liveEvt)liveEvt.close();
  liveEvt=new EventSource('/api/joint_states');
  liveEvt.onmessage=e=>{
    const data=JSON.parse(e.data);
    if(data._imu){
      const [r,p,y]=data._imu;
      const imuDispEl=document.getElementById('imuDisp');
      if(imuDispEl)imuDispEl.textContent=
        `IMU  R:${(r*180/Math.PI).toFixed(1)}°  P:${(p*180/Math.PI).toFixed(1)}°  Y:${(y*180/Math.PI).toFixed(1)}°`;
      const imuSec=document.getElementById('imuSec');if(imuSec)imuSec.style.display='block';
      const imuRows=document.getElementById('imuRows');
      if(imuRows)imuRows.innerHTML=
        [['Roll',(r*180/Math.PI).toFixed(2)+'°'],['Pitch',(p*180/Math.PI).toFixed(2)+'°'],['Yaw',(y*180/Math.PI).toFixed(2)+'°']]
        .map(([k,v])=>`<div class="info-row imu-row"><span>${k}</span><span class="info-val">${v}</span></div>`).join('');
    }
    delete data._imu;delete data._connected;
    applyPose(data);
  };
  liveEvt.onerror=()=>{setTimeout(startSSE,1000);};
}

function buildJointDisplays(joints){
  const movable=Object.values(joints).filter(j=>j.type!=='fixed');
  const jcountEl=document.getElementById('jcount');if(jcountEl)jcountEl.textContent=movable.length;
  const groups={'Head':[],'Waist/Pelvis':[],'Left Arm':[],'Right Arm':[],'Left Hand':[],'Right Hand':[],'Left Leg':[],'Right Leg':[],'Other':[]};
  for(const j of movable){const n=j.name.toLowerCase();
    if(n.includes('head'))groups['Head'].push(j);
    else if(n.includes('waist'))groups['Waist/Pelvis'].push(j);
    else if(n.includes('left')&&(n.includes('shoulder')||n.includes('elbow')||n.includes('wrist')))groups['Left Arm'].push(j);
    else if(n.includes('right')&&(n.includes('shoulder')||n.includes('elbow')||n.includes('wrist')))groups['Right Arm'].push(j);
    else if(n.includes('left')&&(n.includes('hand')||n.includes('finger')||n.includes('thumb')||n.includes('index')||n.includes('middle')||n.includes('palm')))groups['Left Hand'].push(j);
    else if(n.includes('right')&&(n.includes('hand')||n.includes('finger')||n.includes('thumb')||n.includes('index')||n.includes('middle')||n.includes('palm')))groups['Right Hand'].push(j);
    else if(n.includes('left')&&(n.includes('hip')||n.includes('knee')||n.includes('ankle')))groups['Left Leg'].push(j);
    else if(n.includes('right')&&(n.includes('hip')||n.includes('knee')||n.includes('ankle')))groups['Right Leg'].push(j);
    else groups['Other'].push(j);
  }
  for(const j of movable)jointLimits[j.name]={lo:j.limit.lower,hi:j.limit.upper};

  const el=document.getElementById('jointsEl');
  if(!el)return;
  if(!movable.length){el.innerHTML='<div class="no-joint">No movable joints</div>';return;}
  let html='';
  for(const[gname,jlist]of Object.entries(groups)){
    if(!jlist.length)continue;const gid=sid(gname);
    html+=`<div class="group-header" onclick="toggleG('${gid}')"><span class="garr open" id="arr_${gid}">&#x25B6;</span><span>${gname}</span><span style="color:#2a2a3a;margin-left:4px">(${jlist.length})</span></div><div class="group-body" id="gb_${gid}">`;
    for(const j of jlist){
      html+=`<div class="ji" id="ji_${sid(j.name)}" data-joint="${j.name}">
        <div class="ji-head">
          <span class="ji-name" title="${j.name}">${j.name}</span>
          <span class="ji-val" id="v_${sid(j.name)}">0.0°</span>
        </div>
        <div class="ji-bar"><div class="ji-bar-zero"></div><div class="ji-bar-fill" id="b_${sid(j.name)}"></div></div>
      </div>`;
    }
    html+='</div>';
  }
  el.innerHTML=html;
  movable.forEach(j=>{valEls[j.name]=document.getElementById('v_'+sid(j.name));barEls[j.name]=document.getElementById('b_'+sid(j.name));});
  setTimeout(()=>document.querySelectorAll('.group-body').forEach(b=>b.style.maxHeight=b.scrollHeight+'px'),60);
}

window.toggleG=function(gid){
  const body=document.getElementById('gb_'+gid),arr=document.getElementById('arr_'+gid);
  const open=arr.classList.contains('open');
  body.style.maxHeight=open?'0px':body.scrollHeight+'px';arr.classList.toggle('open',!open);
};
const searchBox=document.getElementById('searchBox');
if(searchBox)searchBox.addEventListener('input',function(){
  const q=this.value.toLowerCase().trim();
  document.querySelectorAll('.ji').forEach(el=>el.style.display=(!q||el.dataset.joint?.toLowerCase().includes(q))?'':'none');
});

function setAngle(jname,angle){
  const jobj=jointObjs[jname],jdef=jointDefs[jname];if(!jobj||!jdef)return;
  const ax=new THREE.Vector3(...jdef.axis).normalize();
  jobj.quaternion.copy(baseQs[jname]).multiply(new THREE.Quaternion().setFromAxisAngle(ax,angle));
}

function applyPose(joints){
  for(const[jname,angle]of Object.entries(joints)){
    setAngle(jname,angle);
    if(valEls[jname])valEls[jname].textContent=(angle*180/Math.PI).toFixed(1)+'°';
    const lim=jointLimits[jname],bar=barEls[jname];
    if(bar&&lim){
      const range=Math.max(Math.abs(lim.lo),Math.abs(lim.hi),0.001);
      const ratio=Math.max(-1,Math.min(1,angle/range));
      if(ratio>=0){bar.style.left='50%';bar.style.width=(ratio*50)+'%';}
      else{bar.style.left=(50+ratio*50)+'%';bar.style.width=(-ratio*50)+'%';}
    }
  }
}

const gridBtn=document.getElementById('gridBtn');
if(gridBtn)gridBtn.addEventListener('click',function(){grid.visible=!grid.visible;this.classList.toggle('on',grid.visible);});
const wireBtn=document.getElementById('wireBtn');
if(wireBtn)wireBtn.addEventListener('click',function(){wireMode=!wireMode;this.classList.toggle('on',wireMode);allMeshes.forEach(m=>{if(m.material&&m.material.wireframe!==undefined)m.material.wireframe=wireMode;});});
const axisBtn=document.getElementById('axisBtn');
if(axisBtn)axisBtn.addEventListener('click',function(){axisMode=!axisMode;this.classList.toggle('on',axisMode);allAxes.forEach(a=>a.visible=axisMode);});
const ssBtn=document.getElementById('ssBtn');
if(ssBtn)ssBtn.addEventListener('click',()=>{renderer.render(scene,camera);const a=document.createElement('a');a.download='g1_viewer.png';a.href=cv.toDataURL('image/png');a.click();});

cv.addEventListener('click',e=>{
  if(!allMeshes.length)return;
  const rect=cv.getBoundingClientRect();
  mouse.x=((e.clientX-rect.left)/rect.width)*2-1;
  mouse.y=-((e.clientY-rect.top)/rect.height)*2+1;
  raycaster.setFromCamera(mouse,camera);
  const hits=raycaster.intersectObjects(allMeshes);
  if(selectedMesh){if(selectedMesh.material.color)selectedMesh.material.color.set(selectedMesh.userData.origColor||0x78909c);selectedMesh=null;}
  if(!hits.length)return;
  selectedMesh=hits[0].object;
  if(selectedMesh.material.color){
    selectedMesh.userData.origColor=selectedMesh.material.color.getHex();
    selectedMesh.material.color.set(0xf9c300);
  }
  const lname=selectedMesh.userData.linkName;
  Object.values(jointDefs).forEach(j=>{
    if(j.child===lname||j.parent===lname){const el=document.getElementById('ji_'+sid(j.name));if(el)el.scrollIntoView({block:'nearest',behavior:'smooth'});}
  });
  const tt=document.getElementById('tt');
  if(tt){tt.textContent=lname;tt.style.display='block';
    tt.style.left=(e.clientX-rect.left+10)+'px';tt.style.top=(e.clientY-rect.top+8)+'px';
    setTimeout(()=>tt.style.display='none',2000);}
});

window.addEventListener('load',()=>loadRobot());
</script>
</body>
</html>"""

# ==========================================
# Pages
# ==========================================
@app.get('/')
def index():
    # 3D 뷰어 + Joint States (모터 값 포함)
    return HTMLResponse(HTML_PAGE)

@app.get('/robot-only')
def robot_only():
    # 3D 뷰어만 (모터 값/사이드바 모두 숨김)
    return HTMLResponse(HTML_PAGE.replace('<body>', '<body class="robot-only-mode">', 1))


if __name__ == '__main__':
    uvicorn.run(app, host='0.0.0.0', port=50003, timeout_graceful_shutdown=2)
