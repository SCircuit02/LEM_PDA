// index_html.h - Pagina web del gestor SDI-12 (la incluye esp32s2_sdi12_web.ino).
// Va en un archivo aparte porque el preprocesador del IDE (ctags) no entiende los
// raw strings de C++ y confundiria el JavaScript con funciones del sketch.
#pragma once
#include <Arduino.h>

const char PAGE[] PROGMEM = R"rawliteral(<!doctype html><html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>LemPDA SDI-12</title>
<style>
:root{--bg:#0f172a;--card:#1e293b;--line:#334155;--txt:#e2e8f0;--mut:#94a3b8;--acc:#38bdf8;--ok:#22c55e;--bad:#ef4444}
*{box-sizing:border-box}body{margin:0;font-family:system-ui,-apple-system,sans-serif;background:var(--bg);color:var(--txt)}
header{position:sticky;top:0;z-index:2;background:#020617;padding:12px 16px;display:flex;justify-content:space-between;align-items:center;gap:8px}
h1{font-size:17px;margin:0;white-space:nowrap}
.pill{font-size:13px;padding:5px 10px;border-radius:999px;background:var(--line);text-align:right}.pill.busy{background:#0c4a6e}
main{padding:12px 16px 32px;max-width:720px;margin:auto}
button{font:inherit;border:0;border-radius:10px;padding:11px 14px;background:var(--acc);color:#04111d;font-weight:600;min-height:44px}
button:disabled{opacity:.45}.btn2{background:var(--line);color:var(--txt)}#scan{width:100%;font-size:17px}
.bar{height:6px;background:var(--line);border-radius:3px;overflow:hidden;margin:10px 0}.bar>i{display:block;height:100%;background:var(--acc);width:0;transition:width .3s}
.msg{margin:10px 0;padding:10px 12px;border-radius:10px;background:#14532d;display:none}.msg.bad{background:#7f1d1d}
.card{background:var(--card);border-radius:12px;margin:10px 0;overflow:hidden}
.hd{padding:14px 16px;display:flex;justify-content:space-between;align-items:center;cursor:pointer;gap:10px}
.addr{display:inline-block;min-width:34px;font-size:24px;font-weight:700;color:var(--acc)}
.body{padding:0 16px 16px}
table{width:100%;border-collapse:collapse;font-size:14px;margin-bottom:6px}td{padding:6px 4px;border-bottom:1px solid var(--line)}td:first-child{color:var(--mut);width:40%}
.mt{margin:14px 0 6px}.mut{color:var(--mut);font-size:13px}
.vals{display:grid;grid-template-columns:repeat(auto-fill,minmax(92px,1fr));gap:6px}
.v{background:var(--bg);border-radius:8px;padding:8px 6px;text-align:center;font-variant-numeric:tabular-nums;word-break:break-all}.v small{display:block;color:var(--mut);font-size:11px}
.row{display:flex;gap:8px;flex-wrap:wrap;margin-top:12px;align-items:center}.row button{flex:1 1 0;min-width:0;padding:10px 6px;font-size:15px}
select{font:inherit;padding:10px;border-radius:10px;background:var(--bg);color:var(--txt);border:1px solid var(--line);min-height:44px}
.spin{display:inline-block;width:18px;height:18px;border:3px solid var(--acc);border-right-color:transparent;border-radius:50%;animation:r 1s linear infinite}@keyframes r{to{transform:rotate(360deg)}}
</style></head><body>
<header><h1>LemPDA · SDI-12</h1><span id="st" class="pill">Conectando…</span></header>
<main>
<button id="scan" onclick="act('scan')">Escanear sensores</button>
<div class="bar"><i id="pb"></i></div>
<div id="msg" class="msg"></div>
<div id="list"></div>
<p class="mut" id="foot"></p>
</main>
<script>
const $=id=>document.getElementById(id);
const ALL='0123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ'.split('');
let S=null,open={},sel={},lastKey='',lastMsg=null,tm=null;
const esc=t=>String(t).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
const vals=r=>r?(r.match(/[+-][^+-]*/g)||[]):[];
const ago=s=>s<60?s+' s':Math.floor(s/60)+' min';
function info(s){if(!s)return null;return{ver:s.length>1?s[0]+'.'+s[1]:'',fab:s.substr(2,8).trim(),mod:s.substr(10,6).trim(),sv:s.substr(16,3).trim(),sn:s.substr(19).trim()}}
function note(t,bad){const m=$('msg');m.textContent=t||'';m.className='msg'+(bad?' bad':'');m.style.display=t?'block':'none'}
async function act(p){try{const r=await fetch('/api/'+p,{method:'POST'});const j=await r.json();if(!j.ok)note(j.err||'Error',true)}catch(e){note('Sin conexión con el equipo',true)}poll()}
const free=()=>ALL.filter(c=>!S.sensors.some(x=>x.a===c));
function meas(tag,m){
 if(!m||m.n==-1)return`<div class="mt"><b>${tag}</b> <span class="mut">sin medir</span></div>`;
 if(m.n==-2)return`<div class="mt"><b>${tag}</b> <span class="mut">el sensor no responde a este comando</span></div>`;
 const v=vals(m.raw);
 return`<div class="mt"><b>${tag}</b> <span class="mut">${v.length} valores · espera ${m.t} s · hace ${ago(S.up-m.at)}</span></div>`+
 (v.length?`<div class="vals">${v.map((x,i)=>`<div class="v"><small>${i}</small>${esc(x)}</div>`).join('')}</div>`:'<p class="mut">El sensor no devolvió datos</p>')}
function card(s){
 const inf=info(s.i),op=!!open[s.a],work=S.busy&&S.a===s.a,dis=S.busy?'disabled':'';
 const name=inf&&(inf.fab||inf.mod)?esc((inf.fab+' '+inf.mod).trim()):'<span class="mut">sin identificación</span>';
 let h=`<div class="card"><div class="hd" onclick="tog('${s.a}')"><div><span class="addr">${s.a}</span>${name}</div><span>${work?'<span class="spin"></span>':(op?'&#9650;':'&#9660;')}</span></div>`;
 if(!op)return h+'</div>';
 h+='<div class="body">';
 h+=inf?`<table><tr><td>Fabricante</td><td>${esc(inf.fab)}</td></tr><tr><td>Modelo</td><td>${esc(inf.mod)}</td></tr><tr><td>Versión</td><td>${esc(inf.sv)}</td></tr>${inf.sn?`<tr><td>Serie / extra</td><td>${esc(inf.sn)}</td></tr>`:''}<tr><td>SDI-12</td><td>v${esc(inf.ver)}</td></tr></table>`:'<p class="mut">No respondió a aI!</p>';
 h+=meas('aC! (concurrente)',s.c)+meas('aM! (estándar)',s.m);
 h+=`<div class="row"><button onclick="act('measure?a=${s.a}&m=C')" ${dis}>Medir aC!</button><button onclick="act('measure?a=${s.a}&m=M')" ${dis}>Medir aM!</button><button class="btn2" onclick="act('info?a=${s.a}')" ${dis}>Leer aI!</button></div>`;
 const f=free(),cur=f.includes(sel[s.a])?sel[s.a]:f[0];
 h+=`<div class="row"><span>Cambiar ID ${s.a} &rarr;</span><select onchange="sel['${s.a}']=this.value">${f.map(c=>`<option${c===cur?' selected':''}>${c}</option>`).join('')}</select><button class="btn2" onclick="chid('${s.a}')" ${dis}>Cambiar</button></div>`;
 return h+'</div></div>'}
function tog(a){open[a]=!open[a];lastKey='';render();const s=S.sensors.find(x=>x.a===a);
 if(open[a]&&s&&s.c.n==-1&&s.m.n==-1&&!S.busy)act('measure?a='+a+'&m=B')}
function chid(a){const f=free(),n=f.includes(sel[a])?sel[a]:f[0];if(!n)return;
 if(confirm('¿Cambiar el ID del sensor '+a+' a '+n+'?')){open[n]=true;act('chid?a='+a+'&n='+n)}}
function render(){
 $('st').textContent=S.busy?(S.txt||'Trabajando...'):'Listo';$('st').className='pill'+(S.busy?' busy':'');
 $('pb').style.width=(S.busy&&S.t?Math.min(100,100*S.p/S.t):0)+'%';$('scan').disabled=S.busy;
 if(S.msg!==lastMsg){lastMsg=S.msg;note(S.msg,S.err)}
 const key=JSON.stringify(S.sensors)+S.busy+S.a+Math.floor(S.up/10);if(key===lastKey)return;lastKey=key;
 $('list').innerHTML=S.sensors.length?S.sensors.map(card).join(''):'<p class="mut">No hay sensores en la lista. Toque «Escanear sensores».</p>';
 $('foot').textContent=S.ap+' · '+S.sensors.length+' sensor(es) · toque un sensor para ver sus datos'}
async function poll(){clearTimeout(tm);try{const r=await fetch('/api/state');S=await r.json();render()}catch(e){$('st').textContent='Sin conexión'}
 tm=setTimeout(poll,S&&S.busy?600:2500)}
poll();
</script></body></html>)rawliteral";
