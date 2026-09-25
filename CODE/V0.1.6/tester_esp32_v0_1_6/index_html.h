// index_html.h - Pagina web del QC LemBot 0.1.6 para ESP32-C3 (la incluye tester_esp32_v0_1_6.ino).
// Va en un archivo aparte porque el preprocesador del IDE (ctags) no entiende los
// raw strings de C++ y confundiria el JavaScript con funciones del sketch.
// Debe quedar en la misma carpeta que el .ino.
#pragma once
#include <Arduino.h>

const char PAGE[] PROGMEM = R"rawliteral(<!doctype html><html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>LemPDA</title>
<style>
:root{--bg:#0f172a;--card:#1e293b;--line:#334155;--txt:#e2e8f0;--mut:#94a3b8;--acc:#38bdf8;--ok:#22c55e;--bad:#ef4444;--warn:#f59e0b}
*{box-sizing:border-box}body{margin:0;font-family:system-ui,-apple-system,sans-serif;background:var(--bg);color:var(--txt);-webkit-tap-highlight-color:transparent}
#top{position:sticky;top:0;z-index:3;background:#020617}
header{padding:10px 16px;display:flex;justify-content:space-between;align-items:center;gap:8px}
h1{font-size:17px;margin:0;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}h1 small{font-size:12px;color:var(--mut);font-weight:400;margin-left:8px}
.pill{font-size:13px;padding:5px 10px;border-radius:999px;background:var(--line);white-space:nowrap}.pill.bad{background:#7f1d1d}
nav{display:flex;border-bottom:1px solid var(--line);max-width:760px;margin:auto}
nav button{flex:1 1 0;min-width:0;background:none;color:var(--mut);border-radius:0;padding:10px 0;font-size:clamp(11px,3.4vw,15px);border-bottom:3px solid transparent;white-space:nowrap}
nav button.on{color:var(--acc);border-bottom-color:var(--acc)}
main{padding:6px 16px 40px;max-width:760px;margin:auto}
.tab{display:none}.tab.on{display:block}
button{font:inherit;border:0;border-radius:10px;padding:11px 14px;background:var(--acc);color:#04111d;font-weight:600;min-height:44px;cursor:pointer}
button:disabled{opacity:.45;cursor:default}.btn2{background:var(--line);color:var(--txt)}.btnr{background:#b91c1c;color:#fff}
.row{display:flex;gap:8px;flex-wrap:wrap;margin-top:12px;align-items:center}.row button{flex:1 1 0;min-width:0;padding:10px 6px;font-size:15px}
.ban{display:none;margin:10px 0;padding:10px 12px;border-radius:10px}.ban.on{display:block}.ban.bad{background:#7f1d1d}.ban.warn{background:#78350f}.ban.good{background:#14532d}
.g2{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:10px}
.g4{display:grid;grid-template-columns:repeat(4,1fr);gap:8px;margin-top:8px}@media(max-width:540px){.g4{grid-template-columns:1fr 1fr}}
.tile{background:var(--card);border-radius:12px;padding:10px 12px;min-width:0;white-space:nowrap;overflow:hidden}
.tile small{display:block;color:var(--mut);font-size:13px;margin-bottom:2px}
.tile b{font-size:clamp(22px,7.5vw,34px);font-variant-numeric:tabular-nums}.tile i{font-style:normal;color:var(--mut);font-size:14px;margin-left:4px}
.g4 .tile b{font-size:20px}
.st{margin:12px 2px 4px;color:var(--mut);font-size:14px}
.dot{display:inline-block;width:10px;height:10px;border-radius:50%;background:var(--bad);margin-right:6px;animation:bl 1s steps(2) infinite}@keyframes bl{50%{opacity:.2}}
canvas{width:100%;height:220px;display:block;background:var(--card);border-radius:12px;margin-top:8px}
.box{background:var(--card);border-radius:12px;padding:12px 16px;margin:10px 0}
h2{font-size:15px;margin:0 0 8px;color:var(--acc)}
table{width:100%;border-collapse:collapse;font-size:14px}td,th{padding:6px 4px;border-bottom:1px solid var(--line);font-variant-numeric:tabular-nums;text-align:left}
th{color:var(--mut);font-weight:400;font-size:13px}td:first-child{color:var(--mut)}tr:last-child td{border-bottom:0}
.k td:first-child{width:45%}
.mut{color:var(--mut);font-size:13px}.okc{color:var(--ok)}.badc{color:var(--bad)}
.vals{display:grid;grid-template-columns:repeat(auto-fill,minmax(80px,1fr));gap:6px;margin:6px 0 10px}
.v{background:var(--bg);border-radius:8px;padding:8px 6px;text-align:center;font-variant-numeric:tabular-nums;word-break:break-all}.v small{display:block;color:var(--mut);font-size:11px}
pre{background:var(--bg);border-radius:8px;padding:10px;margin:0;font-size:13px;white-space:pre-wrap;word-break:break-all;min-height:6em}
.bar{height:6px;background:var(--line);border-radius:3px;overflow:hidden;margin:10px 0}.bar>i{display:block;height:100%;background:var(--acc);width:0}
.chip{display:inline-block;padding:4px 10px;border-radius:999px;margin:3px 4px 3px 0;font-size:13px;background:var(--line)}.chip.ok{background:#14532d}.chip.bad{background:#7f1d1d}
.res{font-size:24px;font-weight:700;margin:4px 0}.res small{font-size:15px;font-weight:400;color:var(--mut);margin-left:6px}.res.ok{color:var(--ok)}.res.bad{color:var(--bad)}.res.warn{color:var(--warn)}
.card{background:var(--card);border-radius:12px;margin:10px 0;overflow:hidden}
.hd{padding:14px 16px;display:flex;justify-content:space-between;align-items:center;cursor:pointer;gap:10px}
.addr{display:inline-block;min-width:34px;font-size:24px;font-weight:700;color:var(--acc)}
.body{padding:0 16px 16px}.mt{margin:14px 0 6px}
select{font:inherit;padding:10px;border-radius:10px;background:var(--bg);color:var(--txt);border:1px solid var(--line);min-height:44px}
.spin{display:inline-block;width:16px;height:16px;border:3px solid var(--acc);border-right-color:transparent;border-radius:50%;animation:r 1s linear infinite;vertical-align:middle;margin-right:6px}@keyframes r{to{transform:rotate(360deg)}}
.cfg{display:grid;grid-template-columns:1fr auto;gap:10px 12px;align-items:center}
.cfg label{font-size:14px}.cfg label small{display:block;color:var(--mut);font-size:12px}
.cfg input{width:112px;font:inherit;padding:9px 8px;border-radius:8px;border:1px solid var(--line);background:var(--bg);color:var(--txt);text-align:right}
.cfg input.chg{border-color:var(--warn)}
#toast{position:fixed;left:50%;bottom:18px;transform:translateX(-50%);background:#020617;border:1px solid var(--line);padding:10px 16px;border-radius:10px;display:none;max-width:90vw;z-index:5}
</style></head><body>
<div id="top">
<header><h1>LemPDA<small id="apn"></small></h1><span id="st" class="pill">Conectando…</span></header>
<nav><button data-t="cur" class="on">Corriente</button><button data-t="sen">Sensores</button><button data-t="sdi">SDI-12</button><button data-t="tst">Tests</button><button data-t="eq">Equipo</button></nav>
</div>
<main>
<div id="err" class="ban bad"></div>
<div id="lost" class="ban bad">Sin conexión con el equipo. Revise que el teléfono siga conectado a la red LemPDA.</div>

<section id="t-cur" class="tab on">
<div id="oc" class="ban"></div>
<div class="g2">
<div class="tile"><small>Actual</small><b id="cI">--</b><i>mA</i></div>
<div class="tile"><small>Mediana</small><b id="cMed">--</b><i>mA</i></div>
<div class="tile"><small>Máxima</small><b id="cMax">--</b><i>mA</i></div>
<div class="tile"><small>Mínima</small><b id="cMin">--</b><i>mA</i></div>
</div>
<div class="g4">
<div class="tile"><small>Voltaje</small><b id="cV">--</b><i>V</i></div>
<div class="tile"><small>Carga</small><b id="cMah">--</b><i>mAh</i></div>
<div class="tile"><small>Promedio</small><b id="cAvg">--</b><i>mA</i></div>
<div class="tile"><small>Uso estimado</small><b id="cUse">--</b><i id="cUseU"></i></div>
</div>
<p class="st" id="cSt"></p>
<canvas id="chart"></canvas>
<p class="mut" id="cInfo"></p>
<div class="row"><button class="btn2" onclick="csv()">Descargar CSV</button><button id="bRst" onclick="curReset()">Reiniciar medición</button></div>
</section>

<section id="t-sen" class="tab">
<div class="box"><h2>Batería del equipo</h2><div id="sBat"></div></div>
<div class="box"><h2>Temperatura y humedad</h2><div id="sEnv"></div><p class="mut" id="sEnvAge"></p><div class="row"><button class="btn2" onclick="s10()">Leer SHT10</button></div></div>
<div class="box"><h2>ADS1115</h2><div id="sAds"></div></div>
<div class="box"><h2>Entradas digitales</h2><div id="sGpio"></div></div>
<div class="box"><h2>Soil-QC (sonda DFM)</h2><div id="sSoil"></div></div>
<div class="box"><h2>Monitor UART (GP20, 115200)</h2><pre id="sUart"></pre></div>
</section>

<section id="t-sdi" class="tab">
<button id="sdiScan" style="width:100%;font-size:17px;margin-top:10px" onclick="sdiAct('scan')">Escanear sensores</button>
<div class="bar"><i id="sdiPb"></i></div>
<p class="st" id="sdiSt"></p>
<div id="sdiDev" class="ban warn">Hay una prueba (Sense-QC o Weather-QC) usando los pines compartidos con SDI-12. Espere a que termine.</div>
<div id="sdiMsg" class="ban"></div>
<div id="sdiList"></div>
<p class="mut" id="sdiFoot"></p>
</section>

<section id="t-tst" class="tab">
<div class="box"><h2>Sense-QC</h2><div id="tSa"></div><div class="row"><button id="bSa" onclick="test('sa')">Iniciar Sense-QC</button></div></div>
<div class="box"><h2>Weather-QC</h2><div id="tWx"></div><div class="row"><button id="bWx" onclick="test('wx')">Iniciar Weather-QC</button></div></div>
<div class="box"><h2>Descarga de batería</h2><div id="tDis"></div><div class="row"><button id="bDs" onclick="disch('start')">Iniciar descarga</button><button id="bDp" class="btnr" onclick="disch('stop')">Detener</button></div></div>
<p class="mut">Mientras corre una prueba no se puede iniciar otra. Durante la descarga de batería el monitor de corriente queda en pausa. En el C3, Sense-QC y Weather-QC comparten pines con SDI-12: no se usan a la vez.</p>
</section>

<section id="t-eq" class="tab">
<div class="box"><h2>Configuración</h2><div class="cfg" id="cfg"></div>
<div class="row"><button onclick="saveCfg()">Guardar</button><button class="btn2" onclick="resetCfg()">Valores por defecto</button></div>
<div id="cfgMsg" class="ban"></div></div>
<div class="box"><h2>Sistema</h2><div id="sys"></div></div>
</section>
</main>
<div id="toast"></div>
<script>
const $=id=>document.getElementById(id);
const ALL='0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ'.split('');
const COL={acc:'#38bdf8',line:'#334155',mut:'#94a3b8',warn:'#f59e0b'};
const TT=3,TH=10;   // tolerancias de Temp & Hum respecto al BME280 (igual que el equipo)
const CFG=[
 ['start','Inicio de grabación','mA','Graba cuando la corriente supera este valor',0,100000],
 ['end','Fin de grabación','mA','Termina cuando la corriente baja de este valor',0,100000],
 ['period','Periodo de muestreo','s','',0.1,3600],
 ['maxt','Tiempo máximo de grabación','s','',1,86400],
 ['oc','Umbral de sobrecorriente','mA','',0,100000],
 ['samples','Máximo de muestras','','10 a 2000; al cambiarlo se reinicia la grabación',10,2000],
 ['bat','Capacidad de la batería','mAh','Para el uso estimado y la descarga',1,1000000],
 ['vref','Sense-QC: voltaje de referencia','V','',0,10],
 ['vtol','Sense-QC: tolerancia','%','',0,100],
 ['imin','Tests: corriente mínima','mA','',0,100000],
 ['imax','Tests: corriente máxima','mA','',0,100000],
 ['dlim','Descarga: corriente límite','mA','La descarga se detiene si la supera',0,100000]];
let D=null,S=null,SM=null,tab='cur',tmD=0,tmS=0,fails=0,sKey='',sAt=0,dirty=false,cfgBuilt=false;
let open={},sel={},lastKey='',lastMsg=null;
const esc=t=>String(t).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
const f=(v,d)=>v==null||isNaN(v)?'--':Number(v).toFixed(d);
const tbl=r=>'<table class="k">'+r.map(x=>`<tr><td>${x[0]}</td><td>${x[1]}</td></tr>`).join('')+'</table>';
const chips=(n,ok)=>n.map((x,k)=>`<span class="chip ${ok[k]?'ok':'bad'}">${x} ${ok[k]?'OK':'X'}</span>`).join('');
const hms=s=>{const h=Math.floor(s/3600),m=Math.floor(s/60)%60,x=s%60;return(h?h+':'+String(m).padStart(2,'0'):m)+':'+String(x).padStart(2,'0')};
const dur=s=>s<60?s+' s':s<3600?Math.floor(s/60)+' min '+s%60+' s':s<86400?Math.floor(s/3600)+' h '+Math.floor(s/60)%60+' min':Math.floor(s/86400)+' d '+Math.floor(s/3600)%24+' h';
const ago=s=>s<60?s+' s':Math.floor(s/60)+' min';
function ban(id,t,k){const e=$(id);e.className='ban'+(t?' on '+(k||''):'');if(e.textContent!==t)e.textContent=t||''}
function toast(t){const e=$('toast');e.textContent=t;e.style.display='block';clearTimeout(e.t);e.t=setTimeout(()=>e.style.display='none',3500)}
async function post(p){try{const r=await fetch('/api/'+p,{method:'POST'});const j=await r.json();if(!j.ok){toast(j.err||'Error');return false}return true}
 catch(e){toast('Sin conexión con el equipo');return false}}

// ---------- pestañas ----------
function show(t){tab=t;document.querySelectorAll('nav button').forEach(b=>b.classList.toggle('on',b.dataset.t===t));
 document.querySelectorAll('.tab').forEach(s=>s.classList.toggle('on',s.id==='t-'+t));
 try{localStorage.setItem('tab',t)}catch(e){}
 if(D)rData();if(t==='cur')drawChart();if(t==='sdi')pollSdi();if(t==='eq')loadCfg(false)}
document.querySelectorAll('nav button').forEach(b=>b.onclick=()=>show(b.dataset.t));

// ---------- datos generales ----------
async function pollData(){clearTimeout(tmD);
 try{const r=await fetch('/api/data'+(tab==='sen'&&!document.hidden?'?sen=1':''),{cache:'no-store'});D=await r.json();fails=0;rData()}
 catch(e){if(++fails>=3){$('st').textContent='Sin conexión';$('st').className='pill bad'}}
 $('lost').classList.toggle('on',fails>=3);
 clearTimeout(tmD);tmD=setTimeout(pollData,document.hidden?5000:1000)}
document.addEventListener('visibilitychange',()=>{if(!document.hidden)pollData()});

function rData(){const c=D.cur;
 $('st').textContent=D.bat.v==null?'Sin dato de batería':'Bat '+D.bat.p+' % · '+f(D.bat.v,2)+' V';$('st').className='pill';$('apn').textContent=D.ap;
 ban('err',D.err,'bad');
 if(tab==='cur')rCur();if(tab==='sen')rSen();if(tab==='tst')rTst();if(tab==='eq')rSys();
 const key=c.n+c.st;if(tab==='cur'&&key!==sKey&&(Date.now()-sAt>3000||c.st!=='rec')){sKey=key;loadSamples()}}

function rCur(){const c=D.cur,E=!!D.err,F=(v,d)=>E?'--':f(v,d);   // sin INA228 no se muestran valores viejos
 $('cI').textContent=F(c.i,3);$('cMed').textContent=F(c.med,2);$('cMax').textContent=F(c.max,2);$('cMin').textContent=F(c.min,3);
 $('cV').textContent=F(c.v,3);$('cMah').textContent=F(c.mah,2);$('cAvg').textContent=F(c.avg,2);
 if(E||c.use==null){$('cUse').textContent='--';$('cUseU').textContent=''}
 else if(c.use<100){$('cUse').textContent=c.use.toFixed(1);$('cUseU').textContent='h'}
 else{$('cUse').textContent=(c.use/24).toFixed(0);$('cUseU').textContent='días'}
 if(E)ban('oc','');
 else if(c.paused)ban('oc','Monitor de corriente en pausa: hay una descarga de batería en curso','warn');
 else ban('oc',c.oc?`Sobrecorriente: ${f(c.i,1)} mA supera el umbral de ${f(c.thO,1)} mA`:'','bad');
 const st={wait:`Esperando una corriente mayor a ${f(c.thS,2)} mA para empezar a grabar`,rec:`<span class="dot"></span>Grabando · ${c.n} muestras`,
  fin:`Terminando la grabación · ${c.n} muestras`,done:`Grabación terminada · ${c.n} muestras`};
 $('cSt').innerHTML=D.err?'Sin medición de corriente':(st[c.st]||'');
 $('bRst').disabled=!!D.err||c.paused}

async function loadSamples(){sAt=Date.now();try{const r=await fetch('/api/samples',{cache:'no-store'});SM=await r.json();drawChart()}catch(e){}}

function drawChart(){const c=$('chart');if(!c.clientWidth)return;
 const dpr=window.devicePixelRatio||1,W=c.clientWidth,H=c.clientHeight;c.width=W*dpr;c.height=H*dpr;
 const g=c.getContext('2d');g.setTransform(dpr,0,0,dpr,0,0);g.clearRect(0,0,W,H);g.font='12px system-ui,sans-serif';g.fillStyle=COL.mut;
 const s=SM,n=s?s.n:0;
 if(n<2){g.textAlign='center';g.fillText(D&&D.cur.st==='wait'?'Esperando que la corriente supere el umbral':'Sin datos aún',W/2,H/2);$('cInfo').textContent='';return}
 let lo=Infinity,hi=-Infinity,sum=0,cnt=0;for(const v of s.i){if(v==null)continue;if(v<lo)lo=v;if(v>hi)hi=v;sum+=v;cnt++}
 if(!cnt)return;const avg=sum/cnt;if(hi-lo<1e-3){hi+=0.5;lo-=0.5}const pd=(hi-lo)*0.08;hi+=pd;lo-=pd;
 const L=52,R=10,T=10,B=24,w=W-L-R,h=H-T-B,t0=s.t[0],t1=s.t[n-1],dec=hi-lo<1?3:hi-lo<10?2:1;
 const X=t=>L+(t1>t0?(t-t0)/(t1-t0):0)*w,Y=v=>T+(hi-v)/(hi-lo)*h;
 g.strokeStyle=COL.line;g.lineWidth=1;g.textAlign='right';g.textBaseline='middle';
 for(let k=0;k<=4;k++){const v=lo+(hi-lo)*k/4,y=Y(v);g.beginPath();g.moveTo(L,y);g.lineTo(W-R,y);g.stroke();g.fillText(v.toFixed(dec),L-6,y)}
 g.textBaseline='top';g.textAlign='left';g.fillText(t0.toFixed(0)+' s',L,H-B+6);g.textAlign='right';g.fillText(t1.toFixed(0)+' s',W-R,H-B+6);
 g.strokeStyle=COL.acc;g.lineWidth=2;g.beginPath();let first=true;s.i.forEach((v,k)=>{if(v==null)return;const x=X(s.t[k]),y=Y(v);first?g.moveTo(x,y):g.lineTo(x,y);first=false});g.stroke();
 g.setLineDash([5,4]);g.strokeStyle=COL.warn;g.lineWidth=1.5;g.beginPath();g.moveTo(L,Y(avg));g.lineTo(W-R,Y(avg));g.stroke();g.setLineDash([]);
 $('cInfo').textContent=`${n} muestras en ${t1.toFixed(1)} s · promedio ${avg.toFixed(2)} mA (línea punteada)`}
window.addEventListener('resize',()=>{if(tab==='cur')drawChart()});

async function csv(){await loadSamples();if(!SM||!SM.n){toast('No hay muestras grabadas');return}
 const L=['t_s;i_mA;carga_mAh'];for(let k=0;k<SM.n;k++)L.push([SM.t[k],SM.i[k],SM.q[k]].map(x=>x==null?'':String(x).replace('.',',')).join(';'));
 const a=document.createElement('a');a.href=URL.createObjectURL(new Blob(['﻿'+L.join('\r\n')],{type:'text/csv'}));
 a.download=(D?D.ap:'LemPDA')+'_corriente.csv';document.body.appendChild(a);a.click();setTimeout(()=>{URL.revokeObjectURL(a.href);a.remove()},2000)}
async function curReset(){if(!confirm('¿Reiniciar la medición? Se borran las muestras grabadas.'))return;if(await post('cur/reset')){sKey='';pollData()}}

// ---------- sensores ----------
function rSen(){const e=D.env,b=D.bat;
 if(b.v==null)$('sBat').innerHTML='<p class="mut">Sin lectura: la batería se mide con el ADS1115 (A3) y no se detectó.</p>';else
 $('sBat').innerHTML=`<div class="bar" style="height:10px"><i style="width:${Math.max(0,Math.min(100,b.p))}%;background:${b.p<20?'var(--bad)':'var(--ok)'}"></i></div>`+tbl([['Voltaje',f(b.v,3)+' V'],['Carga',b.p+' %']]);
 const row=(n,t,hh,ref)=>{const has=t!=null;let d='',s='';
  if(ref)d='ref.';else if(has&&e.bt!=null){const dt=t-e.bt,dh=hh-e.bh;d=(dt>=0?'+':'')+dt.toFixed(1);s=Math.abs(dt)<=TT&&Math.abs(dh)<=TH?'<span class="okc">OK</span>':'<span class="badc">fuera</span>'}
  return`<tr><td>${n}</td><td>${has?f(t,1)+' °C':'--'}</td><td>${hh!=null?f(hh,0)+' %':'--'}</td><td>${d}</td><td>${has?s:'<span class="mut">sin lectura</span>'}</td></tr>`};
 $('sEnv').innerHTML='<table><tr><th>Sensor</th><th>Temp</th><th>Hum</th><th>Dif. T</th><th></th></tr>'+row('BME280',e.bt,e.bh,1)+row('SHT30',e.s30t,e.s30h)+row('SHT10',e.s10t,e.s10h)+'</table>'+
  (e.bp!=null?`<p class="mut">Presión (BME280): ${f(e.bp,0)} hPa</p>`:'');
 $('sEnvAge').textContent=(e.age<0?'Aún sin lectura. ':'Actualizado hace '+e.age+' s. ')+`OK = dentro de ±${TT} °C y ±${TH} % del BME280. Se leen solo mientras esta pestaña está abierta. El SHT10 se lee solo con el botón: su línea de datos es GP7, compartida con Pulse2 y SDI-12 (no lo use con una placa Sense-QC conectada).`;
 $('sAds').innerHTML=D.ads?tbl(D.ads.map((v,k)=>['A'+k,f(v,4)+' V'])):'<p class="mut">ADS1115 no detectado (A0/A1 se usan en Sense-QC y A3 para la batería).</p>';
 const hl=v=>v?'ALTO':'BAJO';$('sGpio').innerHTML=tbl([['GP6 · Pulse1 / SDI-12 RX',hl(D.p1)],['GP7 · Pulse2 / SDI-12 TX',hl(D.p2)],['GP20 · UART RX',hl(D.urx)]]);
 const so=D.soil;
 if(!so.ok)$('sSoil').innerHTML='<p class="mut">Esperando datos de la sonda DFM por el puerto Serial (115200 baud).</p>';
 else{const st=a=>{const v=a.filter(x=>x!=null);return v.length?[v.reduce((p,q)=>p+q,0)/v.length,Math.min(...v),Math.max(...v)]:[null,null,null]},hs=st(so.h),ts=st(so.t);
  $('sSoil').innerHTML=tbl([['Humedad promedio',f(hs[0],1)+' % ('+f(hs[1],1)+' a '+f(hs[2],1)+')'],['Temperatura promedio',f(ts[0],1)+' °C ('+f(ts[1],1)+' a '+f(ts[2],1)+')'],['Último dato','hace '+ago(so.age)]])+
  '<div class="mt"><b>Humedad (12)</b></div><div class="vals">'+so.h.map((v,k)=>`<div class="v"><small>H${k+1}</small>${f(v,1)}</div>`).join('')+'</div>'+
  '<div class="mt"><b>Temperatura (13)</b></div><div class="vals">'+so.t.map((v,k)=>`<div class="v"><small>T${k+1}</small>${f(v,1)}</div>`).join('')+'</div>'}
 const u=D.uart.filter(x=>x.length);$('sUart').textContent=u.length?D.uart.join('\n'):'(sin datos)'}

// ---------- tests ----------
function rTst(){const a=D.sa,w=D.wx,d=D.dis,busy=a.st==='test'||w.st==='test'||d.st==='run',no=!!D.err,sb=!!D.sdib;let h;
 if(a.st==='idle')h='<p class="mut">Conecte la placa hija y presione «Iniciar Sense-QC». La prueba dura 5 s.</p>';
 else if(a.st==='test')h=`<div class="bar"><i style="width:${a.prog}%"></i></div>`+tbl([['Corriente',f(a.i,1)+' mA'],['A0',f(a.a0,3)+' V'],['A1',f(a.a1,3)+' V']]);
 else h=`<div class="res ${a.pass?'ok':'bad'}">${a.pass?'PASS':'FAIL'}</div>`+(a.pass?'':`<p class="mut">Falló: ${esc(a.why)}</p>`)+chips(['I','A0','A1','P1','P2'],a.ok)+
  tbl([['Corriente',f(a.i,1)+' mA'],['A0',f(a.a0,3)+' V'],['A1',f(a.a1,3)+' V']]);
 $('tSa').innerHTML=h;$('bSa').disabled=busy||no||sb;$('bSa').textContent=a.st==='res'?'Repetir Sense-QC':'Iniciar Sense-QC';
 const wv=[['Corriente',f(w.i,1)+' mA'],['BME280',f(w.bt,1)+' °C · '+f(w.bh,0)+' %'],['SHT (10 o 30)',f(w.tt,1)+' °C · '+f(w.th,0)+' %']];
 if(w.st==='idle')h='<p class="mut">Conecte la placa hija y presione «Iniciar Weather-QC». La prueba dura 4 s.</p>';
 else if(w.st==='test')h=`<div class="bar"><i style="width:${w.prog}%"></i></div>`+tbl(wv);
 else h=`<div class="res ${w.pass?'ok':'bad'}">${w.pass?'PASS':'FAIL'}</div>`+(w.pass?'':`<p class="mut">Falló: ${esc(w.why)}</p>`)+chips(['I','SHT','T/H'],w.ok)+tbl(wv);
 $('tWx').innerHTML=h;$('bWx').disabled=busy||no||sb;$('bWx').textContent=w.st==='res'?'Repetir Weather-QC':'Iniciar Weather-QC';
 if(d.st==='idle')h='<p class="mut">Conecte la batería y la resistencia de carga, luego presione «Iniciar descarga». Se detiene sola bajo 3,0 V o si la corriente supera el límite.</p>';
 else{h='';if(d.st==='done'){const g=d.pct>=70?['BUENA','ok']:d.pct>=40?['REGULAR','warn']:['MALA','bad'];h=`<div class="res ${g[1]}">${g[0]}<small>${f(d.pct,0)} % de la capacidad</small></div>`}
  else h='<p class="st"><span class="dot"></span>Descargando…</p>';
  h+=tbl([['Voltaje',f(d.v,3)+' V'],['Corriente',f(d.i,1)+' mA'],['Carga',f(d.mah,2)+' mAh'],['Energía',f(d.mwh,1)+' mWh'],['Tiempo',hms(d.s)],['Corriente pico',f(d.pk,1)+' mA']])}
 $('tDis').innerHTML=h;$('bDs').disabled=busy||no;$('bDp').disabled=d.st!=='run'}
async function s10(){if(await post('sht10'))toast('SHT10 leído');pollData()}
async function test(t){if(await post('test?t='+t))pollData()}
async function disch(op){if(!confirm(op==='start'?'Conecte la batería y la resistencia de carga. ¿Iniciar la descarga?':'¿Detener la descarga?'))return;if(await post('disch?op='+op))pollData()}

// ---------- equipo ----------
function rSys(){$('sys').innerHTML=tbl([['Firmware','QC LemBot v'+esc(D.fw)+' (ESP32-C3)'],['Red WiFi',esc(D.ap)],['Dirección','http://192.168.4.1'],
 ['Clientes conectados',D.cl],['Memoria libre',(D.heap/1024).toFixed(0)+' KB'],['Encendido hace',dur(D.up)]])}
function buildCfg(){if(cfgBuilt)return;cfgBuilt=true;
 $('cfg').innerHTML=CFG.map(([k,l,u,hl,mn,mx])=>`<label for="k_${k}">${l}${u?' ('+u+')':''}${hl?'<small>'+hl+'</small>':''}</label><input id="k_${k}" type="number" inputmode="decimal" step="any" min="${mn}" max="${mx}">`).join('');
 CFG.forEach(([k])=>$('k_'+k).addEventListener('input',e=>{dirty=true;e.target.classList.add('chg')}))}
async function loadCfg(force){buildCfg();if(dirty&&!force)return;
 try{const r=await fetch('/api/cfg',{cache:'no-store'});const j=await r.json();
  CFG.forEach(([k])=>{const e=$('k_'+k);e.value=j[k]==null?'':+Number(j[k]).toFixed(3);e.classList.remove('chg')});dirty=false}
 catch(e){ban('cfgMsg','No se pudo leer la configuración','bad')}}
async function saveCfg(){const q=[],bad=[];
 CFG.forEach(([k,l,u,hl,mn,mx])=>{const v=parseFloat(String($('k_'+k).value).replace(',','.'));if(isNaN(v)||v<mn||v>mx)bad.push(l);else q.push(k+'='+v)});
 if(bad.length){ban('cfgMsg','Revise estos valores: '+bad.join(', '),'bad');return}
 if(await post('cfg?'+q.join('&'))){await loadCfg(true);ban('cfgMsg','Configuración guardada en el equipo','good');sKey=''}}
async function resetCfg(){if(!confirm('¿Restaurar todos los valores por defecto?'))return;
 if(await post('cfg/reset')){await loadCfg(true);ban('cfgMsg','Valores por defecto restaurados','good');sKey=''}}

// ---------- SDI-12 ----------
const vals=r=>r?(r.match(/[+-][^+-]*/g)||[]):[];
function info(s){if(!s)return null;return{ver:s.length>1?s[0]+'.'+s[1]:'',fab:s.substr(2,8).trim(),mod:s.substr(10,6).trim(),sv:s.substr(16,3).trim(),sn:s.substr(19).trim()}}
async function sdiAct(p){await post('sdi/'+p);pollSdi()}
const free=()=>ALL.filter(c=>!S.sensors.some(x=>x.a===c));
function meas(tag,m){
 if(!m||m.n==-1)return`<div class="mt"><b>${tag}</b> <span class="mut">sin medir</span></div>`;
 if(m.n==-2)return`<div class="mt"><b>${tag}</b> <span class="mut">el sensor no responde a este comando</span></div>`;
 const v=vals(m.raw);
 return`<div class="mt"><b>${tag}</b> <span class="mut">${v.length} valores · espera ${m.t} s · hace ${ago(S.up-m.at)}</span></div>`+
 (v.length?`<div class="vals">${v.map((x,i)=>`<div class="v"><small>${i}</small>${esc(x)}</div>`).join('')}</div>`:'<p class="mut">El sensor no devolvió datos</p>')}
function card(s){
 const inf=info(s.i),op=!!open[s.a],work=S.busy&&S.a===s.a,dis=S.busy||S.dev?'disabled':'';
 const name=inf&&(inf.fab||inf.mod)?esc((inf.fab+' '+inf.mod).trim()):'<span class="mut">sin identificación</span>';
 let h=`<div class="card"><div class="hd" onclick="tog('${s.a}')"><div><span class="addr">${s.a}</span>${name}</div><span>${work?'<span class="spin"></span>':(op?'&#9650;':'&#9660;')}</span></div>`;
 if(!op)return h+'</div>';
 h+='<div class="body">';
 h+=inf?`<table class="k"><tr><td>Fabricante</td><td>${esc(inf.fab)}</td></tr><tr><td>Modelo</td><td>${esc(inf.mod)}</td></tr><tr><td>Versión</td><td>${esc(inf.sv)}</td></tr>${inf.sn?`<tr><td>Serie / extra</td><td>${esc(inf.sn)}</td></tr>`:''}<tr><td>SDI-12</td><td>v${esc(inf.ver)}</td></tr></table>`:'<p class="mut">No respondió a aI!</p>';
 h+=meas('aC! (concurrente)',s.c)+meas('aM! (estándar)',s.m);
 h+=`<div class="row"><button onclick="sdiAct('measure?a=${s.a}&m=C')" ${dis}>Medir aC!</button><button onclick="sdiAct('measure?a=${s.a}&m=M')" ${dis}>Medir aM!</button><button class="btn2" onclick="sdiAct('info?a=${s.a}')" ${dis}>Leer aI!</button></div>`;
 const fr=free(),cur=fr.includes(sel[s.a])?sel[s.a]:fr[0];
 h+=`<div class="row"><span>Cambiar ID ${s.a} &rarr;</span><select onchange="sel['${s.a}']=this.value">${fr.map(c=>`<option${c===cur?' selected':''}>${c}</option>`).join('')}</select><button class="btn2" onclick="chid('${s.a}')" ${dis}>Cambiar</button></div>`;
 return h+'</div></div>'}
function tog(a){open[a]=!open[a];lastKey='';rSdi();const s=S.sensors.find(x=>x.a===a);
 if(open[a]&&s&&s.c.n==-1&&s.m.n==-1&&!S.busy&&!S.dev)sdiAct('measure?a='+a+'&m=B')}
function chid(a){const fr=free(),n=fr.includes(sel[a])?sel[a]:fr[0];if(!n)return;
 if(confirm('¿Cambiar el ID del sensor '+a+' a '+n+'?')){open[n]=true;sdiAct('chid?a='+a+'&n='+n)}}
function rSdi(){
 $('sdiPb').style.width=(S.busy&&S.t?Math.min(100,100*S.p/S.t):0)+'%';$('sdiScan').disabled=S.busy||S.dev;
 $('sdiSt').innerHTML=S.busy?'<span class="spin"></span>'+esc(S.txt||'Trabajando…'):'';
 $('sdiDev').classList.toggle('on',!!S.dev);
 if(S.msg!==lastMsg){lastMsg=S.msg;ban('sdiMsg',S.msg,S.err?'bad':'good')}
 const key=JSON.stringify(S.sensors)+S.busy+S.dev+S.a+Math.floor(S.up/10);if(key===lastKey)return;lastKey=key;
 $('sdiList').innerHTML=S.sensors.length?S.sensors.map(card).join(''):'<p class="mut">No hay sensores en la lista. Toque «Escanear sensores».</p>';
 $('sdiFoot').textContent=S.sensors.length+' sensor(es) · toque un sensor para ver sus datos'}
async function pollSdi(){clearTimeout(tmS);try{const r=await fetch('/api/sdi',{cache:'no-store'});S=await r.json();rSdi()}catch(e){}
 clearTimeout(tmS);if(tab==='sdi'||(S&&S.busy))tmS=setTimeout(pollSdi,S&&(S.busy||S.dev)?600:2500)}

try{const t=localStorage.getItem('tab');if(t&&$('t-'+t))show(t)}catch(e){}
pollData();
</script></body></html>)rawliteral";
