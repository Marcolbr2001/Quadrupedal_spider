#ifndef PAGE_H
#define PAGE_H

#include <Arduino.h>
#include "Remote.h"   // PAGE_VERSION comes from here

// The interface served by the spider. It all lives here as a string in
// flash: one file, no filesystem to mount and nothing to upload besides
// the sketch. With 8 MB of flash, space is not a concern.

const char PAGE[] PROGMEM = R"PAGE(<!doctype html>
<html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no,viewport-fit=cover">
<meta name="apple-mobile-web-app-capable" content="yes">
<meta http-equiv="Cache-Control" content="no-store, must-revalidate">
<title>Spider</title>
<style>
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent;-webkit-user-select:none;user-select:none}
html,body{margin:0;height:100%;overscroll-behavior:none;touch-action:manipulation}
body{background:#14161a;color:#e7e9ee;font:15px/1.4 -apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,sans-serif;
 padding:env(safe-area-inset-top) 12px env(safe-area-inset-bottom)}
h2{font-size:13px;text-transform:uppercase;letter-spacing:.08em;color:#8b93a3;margin:18px 0 8px}
#tabs{display:flex;gap:6px;padding:10px 0 4px;position:sticky;top:0;background:#14161a;z-index:5}
#tabs button{flex:1;padding:11px 0;border:0;border-radius:10px;background:#242832;color:#8b93a3;font-size:14px;font-weight:600}
#tabs button.on{background:#3b82f6;color:#fff}
.pane{display:none}.pane.on{display:block}
#remote{display:flex;gap:6px;margin-bottom:10px}
#remote div{flex:1;background:#1c1f27;border-radius:10px;padding:8px 6px;text-align:center}
#remote b{display:block;font-size:19px;font-variant-numeric:tabular-nums}
#remote span{font-size:10px;color:#8b93a3;text-transform:uppercase;letter-spacing:.05em}
#sticks{display:flex;gap:10px;justify-content:center;margin:6px 0}
.stick{flex:1;max-width:46vw;aspect-ratio:1;background:#1c1f27;border-radius:50%;position:relative;
 border:2px solid #2c313d;touch-action:none}
.stick .knob{position:absolute;width:38%;height:38%;left:31%;top:31%;border-radius:50%;
 background:#3b82f6;box-shadow:0 3px 10px #0008}
.stick.r .knob{background:#8b5cf6}
.stick .lbl{position:absolute;bottom:7%;width:100%;text-align:center;font-size:10px;color:#8b93a3;
 text-transform:uppercase;letter-spacing:.05em}
.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:7px}
.grid.c2{grid-template-columns:repeat(2,1fr)}
button.b{padding:13px 4px;border:0;border-radius:10px;background:#242832;color:#e7e9ee;font-size:13px;font-weight:500}
button.b:active{background:#3b82f6}
button.stop{background:#dc2626;color:#fff;font-size:17px;font-weight:700;padding:16px;width:100%;
 border:0;border-radius:12px;margin-top:8px}
button.tog{background:#242832}button.tog.on{background:#16a34a;color:#fff}
.row{display:flex;align-items:center;gap:10px;margin:14px 0}
.row label{flex:0 0 78px;font-size:13px;color:#8b93a3}
.row output{flex:0 0 52px;text-align:right;font-variant-numeric:tabular-nums;font-size:14px}
input[type=range]{flex:1;-webkit-appearance:none;height:26px;background:transparent}
input[type=range]::-webkit-slider-runnable-track{height:5px;border-radius:3px;background:#2c313d}
input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:26px;height:26px;border-radius:50%;
 background:#3b82f6;margin-top:-11px}
#status{text-align:center;color:#8b93a3;font-size:12px;padding:10px 0 16px}
</style></head><body>

<div id="tabs">
 <button class="on" onclick="tab(0)" data-t="drive">Drive</button>
 <button onclick="tab(1)" data-t="poses">Poses</button>
 <button onclick="tab(2)" data-t="settings">Settings</button>
</div>

<div class="pane on">
 <div id="remote">
  <div><b id="t_d">--</b><span>cm</span></div>
  <div><b id="t_v">--</b><span data-t="tileSpeed">speed</span></div>
  <div><b id="t_bv">--</b><span id="t_bl" data-t="battery">battery</span></div>
 </div>
 <div id="sticks">
  <div class="stick" id="sL"><div class="knob"></div><div class="lbl" data-t="stickDrive">drive</div></div>
  <div class="stick r" id="sR"><div class="knob"></div><div class="lbl" data-t="stickLook">look</div></div>
 </div>
 <button class="stop" ontouchstart="cmd('x')" onclick="cmd('x')" data-t="stop">STOP</button>
 <div class="grid" style="margin-top:8px">
  <button class="b tog" id="bAuto" onclick="cmd('m')" data-t="automatic">Automatic</button>
  <button class="b tog" id="bTrot" onclick="cmd('t')" data-t="trot">Trot</button>
  <button class="b tog" id="bLiv"  onclick="cmd('l')" data-t="level">Level</button>
 </div>
 <div class="grid" style="margin-top:7px">
  <button class="b tog" id="bBatt" onclick="cmd('v')" data-t="battery">Battery</button>
  <button class="b" onclick="cmd('V')" data-t="clearAlarm">Clear alarm</button>
  <button class="b" onclick="cmd('z')" data-t="zeroImu">Zero IMU</button>
 </div>
 <h2 data-t="expressions">Expressions</h2>
 <div class="grid">
  <button class="b" onclick="cmd('1')" data-t="normal">Normal</button>
  <button class="b" onclick="cmd('2')" data-t="happy">Happy</button>
  <button class="b" onclick="cmd('3')" data-t="angry">Angry</button>
  <button class="b" onclick="cmd('4')" data-t="puzzled">Puzzled</button>
  <button class="b" onclick="cmd('5')" data-t="scared">Scared</button>
  <button class="b" onclick="cmd('6')" data-t="sleepy">Sleepy</button>
 </div>
</div>

<div class="pane">
 <h2 data-t="gestures">Gestures</h2>
 <div class="grid">
  <button class="b" onclick="cmd('G')" data-t="giggle">Giggle</button>
  <button class="b" onclick="cmd('S')" data-t="stretch">Stretch</button>
  <button class="b" onclick="cmd('N')" data-t="nod">Nod</button>
  <button class="b" onclick="cmd('K')" data-t="shakeNo">Shake no</button>
  <button class="b" onclick="cmd('C')" data-t="curious">Curious</button>
  <button class="b" onclick="cmd('B')" data-t="bow">Bow</button>
  <button class="b" onclick="cmd('D')" data-t="dance">Dance</button>
  <button class="b" onclick="cmd('P')" data-t="fright">Fright</button>
  <button class="b" onclick="cmd('h')" data-t="wave">Wave</button>
 </div>
 <h2 data-t="bodyPoses">Body poses</h2>
 <div class="grid">
  <button class="b" onclick="cmd('7')" data-t="leanRight">Lean right</button>
  <button class="b" onclick="cmd('9')" data-t="noseDown">Nose down</button>
  <button class="b" onclick="cmd('8')" data-t="leanLeft">Lean left</button>
  <button class="b" onclick="cmd('o')" data-t="yawRight">Yaw right</button>
  <button class="b" onclick="cmd('0')" data-t="noseUp">Nose up</button>
  <button class="b" onclick="cmd('p')" data-t="yawLeft">Yaw left</button>
  <button class="b" onclick="cmd('n')" data-t="weightRight">Weight right</button>
  <button class="b" onclick="cmd('c')" data-t="neutral">Neutral</button>
  <button class="b" onclick="cmd('b')" data-t="weightLeft">Weight left</button>
  <button class="b" onclick="cmd('k')" data-t="crouch">Crouch</button>
  <button class="b" onclick="cmd('i')" data-t="tiptoe">Tiptoe</button>
  <button class="b" onclick="cmd('A')" data-t="square">Square</button>
 </div>
 <h2 data-t="posture">Posture and service</h2>
 <div class="grid">
  <button class="b" onclick="cmd('u')" data-t="stand">Stand</button>
  <button class="b" onclick="cmd('j')" data-t="sit">Sit</button>
  <button class="b" onclick="cmd('Q')" data-t="allTo90">All to 90</button>
  <button class="b" onclick="cmd('r')" data-t="relax">Relax</button>
  <button class="b" onclick="cmd('e')" data-t="enable">Enable</button>
  <button class="b" onclick="cmd('z')" data-t="zeroImu">Zero IMU</button>
 </div>
</div>

<div class="pane">
 <h2 data-t="gait">Gait</h2>
 <div class="row"><label data-t="stride">Stride</label><input type=range id="s_stride" min="20" max="90" step="5"><output id="o_stride"></output></div>
 <div class="row"><label data-t="lift">Lift</label><input type=range id="s_lift" min="8" max="35" step="1"><output id="o_lift"></output></div>
 <div class="row"><label data-t="support">Support</label><input type=range id="s_support" min="0" max="400" step="25"><output id="o_support"></output></div>
 <div class="row"><label data-t="speed">Speed</label><input type=range id="s_speed" min="0.4" max="3" step="0.1"><output id="o_speed"></output></div>
 <h2 data-t="attitude">Attitude</h2>
 <div class="row"><label data-t="gain">Gain</label><input type=range id="s_gain" min="0" max="1.5" step="0.1"><output id="o_gain"></output></div>
 <h2 data-t="distances">Distances</h2>
 <div class="row"><label data-t="panic">Panic</label><input type=range id="s_panic" min="5" max="25" step="1"><output id="o_panic"></output></div>
 <div class="row"><label data-t="scan">Scan</label><input type=range id="s_scan" min="15" max="45" step="1"><output id="o_scan"></output></div>
 <h2 data-t="language">Language</h2>
 <div class="grid c2">
  <button class="b tog" id="bEn" onclick="setLang('en')">English</button>
  <button class="b tog" id="bIt" onclick="setLang('it')">Italiano</button>
 </div>
</div>

<div id="status">connecting...</div>

<script>
const VER=')PAGE" PAGE_VERSION R"PAGE(';
const $=id=>document.getElementById(id);

// --- languages ---
// It all lives in the page: the firmware knows nothing about the language
// and keeps sending the same one-letter commands. Switching language does
// not touch the spider, only what is written on the buttons.
// Mind the name. L is the left joystick, a few dozen lines further down,
// and two const L in the same scope are a syntax error: not a warning in
// the console but a dead script, a page that draws and then does nothing.
const LANG = {
 en:{},                              // English is already in the HTML
 it:{drive:'Guida',poses:'Pose',settings:'Impostazioni',
  automatic:'Automatico',trot:'Trotto',level:'Livella',
  battery:'Batteria',clearAlarm:'Azzera allarme',zeroImu:'Zero IMU',
  expressions:'Espressioni',normal:'Normale',happy:'Felice',angry:'Arrabbiato',
  puzzled:'Confuso',scared:'Impaurito',sleepy:'Dorme',
  gestures:'Gesti',giggle:'Risatina',stretch:'Stiracchiata',nod:'Annuisce',
  shakeNo:'Fa di no',curious:'Curiosone',bow:'Inchino',dance:'Danza',
  fright:'Spavento',wave:'Saluta',
  bodyPoses:'Pose del corpo',leanRight:'Incl. destra',noseDown:'Muso giu',
  leanLeft:'Incl. sinistra',yawRight:'Ruota destra',noseUp:'Muso su',
  yawLeft:'Ruota sinistra',weightRight:'Peso destra',neutral:'Neutro',
  weightLeft:'Peso sinistra',crouch:'Accucciato',tiptoe:'In punta',square:'Quadrata',
  posture:'Posture e servizio',stand:'In piedi',sit:'Seduto',allTo90:'Tutti a 90',
  relax:'Rilassa',enable:'Attiva',
  gait:'Andatura',stride:'Falcata',lift:'Alzata',support:'Appoggio',speed:'Velocita',
  attitude:'Assetto',gain:'Guadagno',
  distances:'Distanze',panic:'Panico',scan:'Scansione',
  language:'Lingua',stop:'STOP',stickDrive:'guida',stickLook:'guarda',
  tileSpeed:'velocita',
  // text that only ever appears on screen, never in the HTML
  connecting:'collegamento...',noReply:'nessuna risposta dal ragno',
  onCable:'a cavo',full:'piena',half:'media',low:'BASSA',empty:'SCARICA',
  frames:'fotogrammi',missing:'ASSENTI',runningCable:'alimentato a cavo',
  oldPage:'PAGINA VECCHIA',reload:'ricaricala tenendo premuto il refresh',
  instead:'invece di'}
};
const EN = {language:'Language',connecting:'connecting...',
 noReply:'no reply from the spider',onCable:'on cable',
 full:'full',half:'half',low:'LOW',empty:'EMPTY',
 frames:'frames',missing:'MISSING',runningCable:'running on cable',
 oldPage:'OLD PAGE',reload:'reload it, holding the refresh button',
 instead:'instead of'};

let lang = 'en';
try { lang = localStorage.getItem('lang') || 'en'; } catch(e) {}

// Key -> text. With no translation we fall back to English, which is
// written straight into the HTML or into EN.
function tr(k, fallback){
 if(lang!=='en' && LANG[lang] && LANG[lang][k]!==undefined) return LANG[lang][k];
 return EN[k]!==undefined ? EN[k] : (fallback!==undefined ? fallback : k);
}

function setLang(v){
 lang = v;
 try { localStorage.setItem('lang', v); } catch(e) {}
 applyLang();
}

function applyLang(){
 document.querySelectorAll('[data-t]').forEach(e=>{
  if(e.dataset.en===undefined) e.dataset.en = e.textContent;   // the original
  e.textContent = (lang==='en') ? e.dataset.en
                                : tr(e.dataset.t, e.dataset.en);
 });
 $('bEn').classList.toggle('on', lang==='en');
 $('bIt').classList.toggle('on', lang==='it');
}
function tab(n){document.querySelectorAll('#tabs button').forEach((b,i)=>b.classList.toggle('on',i==n));
 document.querySelectorAll('.pane').forEach((p,i)=>p.classList.toggle('on',i==n));}

// --- joystick: returns -100..100 on two axes, y positive upwards ---
function stick(el){
 const k=el.querySelector('.knob');let x=0,y=0,id=null;
 const radius=()=>el.clientWidth/2;
 function move(e){
  const r=el.getBoundingClientRect();
  let dx=(e.clientX-r.left-r.width/2)/radius();
  let dy=(e.clientY-r.top-r.height/2)/radius();
  const m=Math.hypot(dx,dy); if(m>1){dx/=m;dy/=m;}
  x=Math.round(dx*100); y=Math.round(-dy*100);
  k.style.transform='translate('+(dx*radius()*0.62)+'px,'+(dy*radius()*0.62)+'px)';
 }
 function springBack(){x=0;y=0;id=null;k.style.transform='';}
 el.addEventListener('pointerdown',e=>{id=e.pointerId;el.setPointerCapture(id);move(e);send();});
 el.addEventListener('pointermove',e=>{if(e.pointerId===id)move(e);});
 el.addEventListener('pointerup',e=>{springBack();release();});
 el.addEventListener('pointercancel',e=>{springBack();release();});
 return {get x(){return x},get y(){return y},get enabled(){return id!==null}};
}
const L=stick($('sL')), R=stick($('sR'));

// --- network ---
// A command must NEVER be thrown away. There used to be a single lock over
// every request: as soon as the network slowed down the periodic heartbeat
// held it shut, and buttons and joystick ended up in the bin while the
// telemetry kept arriving. It looked like a command problem and it was a
// queueing problem. Now commands always go out; only the heartbeat and the
// joystick refresh are skipped when one is already in flight.
async function http(q, timeoutMs){
 const ac = new AbortController();
 const t = setTimeout(()=>ac.abort(), timeoutMs||2000);
 try{
  const r = await fetch('/c'+q, {cache:'no-store', signal:ac.signal});
  show(await r.json());
  return true;
 }catch(e){
  $('status').textContent = tr('noReply');
  return false;
 }finally{ clearTimeout(t); }
}

function cmd(c){ http('?b='+encodeURIComponent(c)); }
function param(n,v){ http('?p='+n+':'+v); }

let jInFlight=false, jLast='';
async function send(){
 if(jInFlight) return;                 // the next pass in 100 ms will do
 const q = '?j='+L.x+','+L.y+','+R.x+','+R.y;
 if(q===jLast && !L.enabled && !R.enabled) return;
 jLast = q; jInFlight = true;
 await http(q, 800);
 jInFlight = false;
}

// The release never goes through the queue: if it were dropped, the spider
// would keep walking until its own watchdog expired.
function release(){ jLast='?j=0,0,0,0'; http(jLast); }

let bInFlight=false;
async function heartbeat(){
 if(bInFlight) return;
 bInFlight = true;
 await http('', 2000);
 bInFlight = false;
}

let firstTime=true;
function show(t){
 $('t_d').textContent = t.d>0? t.d : '--';
 $('t_v').textContent = t.v.toFixed(1);
 const usb = (t.bl===4);
 $('t_bv').textContent = usb ? 'USB' : t.bv.toFixed(1) + 'V';
 $('t_bl').textContent = usb ? tr('onCable')
   : ([tr('full'),tr('half'),tr('low'),tr('empty')][t.bl] || tr('battery','battery'));
 $('t_bv').style.color = (t.bl===2||t.bl===3) ? '#f87171'
   : (usb ? '#8b93a3' : '#e7e9ee');
 $('bBatt').classList.toggle('on',!!t.bm);
 $('bAuto').classList.toggle('on',!!t.a);
 $('bTrot').classList.toggle('on',!!t.t);
 $('bLiv').classList.toggle('on',!!t.l);
 $('status').textContent = t.s + '  \u00b7  ' + t.hz + ' Hz  \u00b7  ' + (t.ram/1024|0) + ' kB'
  + '  \u00b7  r' + t.r.toFixed(0) + ' b' + t.b.toFixed(0)
  + (t.bl===4 ? '  \u00b7  '+tr('runningCable') : '  \u00b7  batt ' + t.bp + '%')
  + '  \u00b7  eyes ' + (t.oc ? t.of + ' '+tr('frames') : tr('missing'));
 if(t.pv && t.pv!==VER){
  $('status').textContent = tr('oldPage')+' ('+VER+' '+tr('instead')+' '+t.pv+
                            '): '+tr('reload');
  $('status').style.color = '#f87171';
 }
 if(firstTime){                     // sliders start from the real values
  firstTime=false;
  setSlider('stride',t.stride); setSlider('lift',t.lift);
  setSlider('support',t.support); setSlider('speed',t.speed);
  setSlider('gain',t.gain); setSlider('panic',t.panic);
  setSlider('scan',t.scan);
 }
}
function setSlider(n,v){ const s=$('s_'+n); if(s===null||v===undefined)return;
 s.value=v; $('o_'+n).textContent=v; }

document.querySelectorAll('input[type=range]').forEach(s=>{
 const n=s.id.slice(2);
 s.addEventListener('input',()=>$('o_'+n).textContent=s.value);
 s.addEventListener('change',()=>param(n,s.value));
});

// Ten times a second while driving, twice a second when idle: plenty, and
// it does not swamp the spider's small server.
setInterval(()=>{ if(L.enabled||R.enabled) send(); },100);
setInterval(()=>{ if(!L.enabled&&!R.enabled) heartbeat(); },600);
applyLang();
heartbeat();
</script></body></html>
)PAGE";

#endif // PAGE_H
