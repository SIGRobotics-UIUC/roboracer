const root=document.getElementById('race-performance'), q=s=>root.querySelector(s);
if(!D.laps.length){q('.race-label').textContent=D.reason||'No measurable laps available';return;}
let lap=-1,plots=[],indices=[],sync=false,indexSet=new Set(),positions=new Map(),timeIndices=new Map(),progressIndices=[];
const visibility={},colors=['#0671ee','#e63572','#768397','#a14b9c','#796618','#537784'];
const keys=Object.keys(D.signals),color=k=>colors[/truth|True/.test(k)?1:/Raceline|raceline/.test(k)?2:/request/.test(k)?3:/gate/.test(k)?4:/odometry/.test(k)?5:0];
function trace(key,x,y,custom){return {name:key.split('|')[0],x,y,customdata:custom,mode:'lines+markers',marker:{color:color(key),size:x.map((v,i)=>v!==null&&y[i]!==null&&(i===0||x[i-1]===null||y[i-1]===null)&&(i===x.length-1||x[i+1]===null||y[i+1]===null)?6:0)},connectgaps:false,line:{color:color(key)},hoverinfo:'none',visible:visibility[key]??true};}
const select=q('.race-select');
D.laps.forEach((l,i)=>{let o=document.createElement('option');o.value=i;o.textContent=`Lap ${i+1}${l.complete?'':' (partial)'}`;select.append(o);});
const times=D.laps.filter(l=>l.complete&&l.valid&&l.time_s>0).map(l=>l.time_s),best=Math.min(...times);
const table=document.createElement('table');
let header=document.createElement('tr');['Lap','Classification','Native time (s)','Completed'].forEach(t=>{let e=document.createElement('th');e.textContent=t;header.append(e)});table.append(header);
D.laps.forEach((l,i)=>{let row=document.createElement('tr');[i+1,l.start_kind||'Classification unavailable',l.valid?l.time_s:'Unavailable',l.complete?'Yes':'No'].forEach(t=>{let e=document.createElement('td');e.textContent=t;row.append(e)});if(l.complete&&l.valid&&l.time_s===best){row.style.background='#e0f4df';row.title='Fastest completed lap';row.children[2].style.fontWeight='750';}row.tabIndex=0;row.onclick=()=>choose(i);row.onkeydown=e=>{if(e.key==='Enter')choose(i)};table.append(row)});q('.race-laps').append(table);
// All interactions publish to the report selection; pinning has no local copy.
const selection=()=>window.benchmarkReport.getSelection();
const lapCache=new Map();
const lapIndices=n=>lapCache.get(n)??cacheLap(n);
function cacheLap(n){const result=D.samples.map((s,i)=>s.recorded>=D.laps[n].start&&s.recorded<=D.laps[n].end&&Number.isFinite(s.t)&&s.xy?.every(Number.isFinite)?i:null).filter(i=>i!==null);lapCache.set(n,result);return result;}
function selectedIndex(){const state=selection();return indexSet.has(state.sampleIndex)&&D.samples[state.sampleIndex].t===state.timestamp?state.sampleIndex:timeIndices.get(state.timestamp);}
function choose(i){if(i<0||i>=D.laps.length)return;const first=lapIndices(i)[0];window.benchmarkReport.requestSelection({lap:i,timestamp:first===undefined?null:D.samples[first].t,sampleIndex:first??null,pinned:selection().pinned&&first!==undefined});}
q('.race-prev').onclick=()=>choose(lap-1);q('.race-next').onclick=()=>choose(lap+1);select.onchange=()=>choose(+select.value);
function unpin(){window.benchmarkReport.requestSelection({pinned:false});}
q('.race-unpin').onclick=unpin;
function inspect(i,pin=false){if(!indexSet.has(i)||selection().pinned&&!pin)return;if(selectedIndex()===i&&selection().pinned===pin)return;window.benchmarkReport.requestSelection({lap,timestamp:D.samples[i].t,sampleIndex:i,pinned:pin});}
// One adjacent localized-pose sample per step, including across telemetry gaps.
function nudge(step){if(!indices.length)return;const at=(positions.get(selectedIndex())??-1),next=at<0?(step>0?0:indices.length-1):Math.max(0,Math.min(indices.length-1,at+step));inspect(indices[next],true);}
q('.race-back').onclick=()=>nudge(-1);q('.race-forward').onclick=()=>nudge(1);
document.addEventListener('keydown',e=>{
 if(e.key==='Escape'){if(selection().pinned){unpin();e.preventDefault();}return;}
 if(!['ArrowLeft','ArrowRight'].includes(e.key)||e.altKey||e.ctrlKey||e.metaKey||e.shiftKey)return;
 if(e.target.closest('input,textarea,select,button,summary,[contenteditable]:not([contenteditable="false"]),[role="slider"],[role="textbox"],[role="combobox"]'))return;
 if(!indices.length)return;e.preventDefault();nudge(e.key==='ArrowLeft'?-1:1);
},true);
function nearestProgress(distance,candidates=progressIndices){
 let lo=0,hi=candidates.length;while(lo<hi){const mid=(lo+hi)>>>1;if(D.samples[candidates[mid]].d<distance)lo=mid+1;else hi=mid;}
 const a=candidates[Math.max(0,lo-1)],b=candidates[Math.min(candidates.length-1,lo)];if(a===undefined)return null;
 return Math.abs(D.samples[a].d-distance)<=Math.abs(D.samples[b].d-distance)?a:b;
}
function pointerInspect(i,pin=false){if(i!==null)inspect(i,pin);}
function plotPick(div,pixel){
 const axis=div._fullLayout?.xaxis;if(!axis)return null;
 if(!div.recordedCandidates)div.recordedCandidates=progressIndices.filter(i=>div.data.some((tr,n)=>tr.visible!==false&&tr.visible!=='legendonly'&&Number.isFinite(D.signals[div.signalKeys[n]]?.[i])));
 const i=nearestProgress(axis.p2l(pixel),div.recordedCandidates);
 return i;
}
function wire(div,signalKeys,map=false){
 div.signalKeys=signalKeys;
 div.on('plotly_hover',e=>{if(e.event||div.racePointerHover)return;const i=e.points[0]?.customdata;if(Number.isInteger(i))inspect(i)});
 div.on('plotly_click',e=>{if(e.event)return;div.racePointClick=performance.now();const i=e.points[0]?.customdata;if(Number.isInteger(i))inspect(i,true)});
 // Plotly's native hover search is rate-limited. Drive the shared position directly
 // from axis coordinates, using the same animation-frame cadence as the track.
 let hoverFrame=0,hoverEvent=null;
 div.addEventListener('pointerleave',()=>{div.racePointerHover=false;hoverEvent=null;});
 div.addEventListener('pointermove',e=>{div.racePointerHover=true;if(selection().pinned||e.buttons||e.target.closest('.legend,.modebar'))return;hoverEvent=e;if(hoverFrame)return;hoverFrame=requestAnimationFrame(()=>{hoverFrame=0;const event=hoverEvent;hoverEvent=null;if(!event)return;const rect=div.getBoundingClientRect(),axis=div._fullLayout?.xaxis,yaxis=div._fullLayout?.yaxis;if(!axis||!yaxis)return;const x=event.clientX-rect.left-axis._offset,y=event.clientY-rect.top-yaxis._offset;if(x<0||x>axis._length||y<0||y>yaxis._length)return;pointerInspect(plotPick(div,x));});},true);
 let down=null;div.addEventListener('pointerdown',e=>{down=[e.clientX,e.clientY]});
 div.addEventListener('click',e=>{if(!down||Math.hypot(e.clientX-down[0],e.clientY-down[1])>4||e.target.closest('.legend,.modebar'))return;const at=performance.now(),rect=div.getBoundingClientRect(),axis=div._fullLayout?.xaxis,yaxis=div._fullLayout?.yaxis;if(!axis||!yaxis)return;const px=e.clientX-rect.left-axis._offset,py=e.clientY-rect.top-yaxis._offset;if(px<0||px>axis._length||py<0||py>yaxis._length)return;requestAnimationFrame(()=>{if(div.racePointClick>=at-10)return;pointerInspect(plotPick(div,px),true)});});
 div.on('plotly_restyle',()=>{div.recordedCandidates=null;if(lastCursor!==null)requestAnimationFrame(()=>cursor(lastCursor))});
 div.on('plotly_legendclick',e=>{const k=signalKeys[e.curveNumber];if(k){visibility[k]=div.data[e.curveNumber].visible==='legendonly'?true:'legendonly';} });
 if(!map)div.on('plotly_relayout',e=>{if(sync)return;let update;if(e['xaxis.range'])update={'xaxis.range':e['xaxis.range']};else if(e['xaxis.range[0]']!==undefined)update={'xaxis.range':[e['xaxis.range[0]'],e['xaxis.range[1]']]};else if(e['xaxis.autorange'])update={'xaxis.autorange':true};if(update){sync=true;Promise.all(plots.filter(p=>p!==div).map(p=>Plotly.relayout(p,update))).finally(()=>{sync=false;if(lastCursor!==null)cursor(lastCursor)})}});
}
let mapMarkers=[],lastCursor=null;
function drawMap(){
 const host=q('.race-track');host.replaceChildren();mapMarkers=[];
 const legend=document.createElement('div');legend.className='race-map-legend';const legendTitle=document.createElement('strong');legendTitle.textContent='Track legend';legend.append(legendTitle);
 const NS='http://www.w3.org/2000/svg',svg=document.createElementNS(NS,'svg');host.append(svg);
 function el(tag,attrs,parent=svg){const e=document.createElementNS(NS,tag);for(const [k,v] of Object.entries(attrs))e.setAttribute(k,v);parent.append(e);return e;}
 const ref=D.reference,all=ref.concat(indices.map(i=>D.samples[i].xy));
 let xs=all.map(p=>p[0]),ys=all.map(p=>-p[1]);let minX=xs.reduce((a,b)=>Math.min(a,b),Infinity),minY=ys.reduce((a,b)=>Math.min(a,b),Infinity),w=xs.reduce((a,b)=>Math.max(a,b),-Infinity)-minX,h=ys.reduce((a,b)=>Math.max(a,b),-Infinity)-minY;
 const pad=Math.max(w,h)*.08;w+=2*pad;h+=2*pad;minX-=pad;minY-=pad;
 const sampleDots=[],view=[minX,minY,w,h];svg.setAttribute('viewBox',view.join(' '));svg.setAttribute('aria-label','Track map with lap start and direction');
 if(D.map){const m=D.map,[x,y,a]=m.origin,c=Math.cos(a),s=Math.sin(a),r=m.resolution;
 el('image',{href:m.image,width:m.width,height:m.height,transform:`matrix(${c*r} ${-s*r} ${s*r} ${c*r} ${x-s*m.height*r} ${-y-c*m.height*r})`});}
 function path(points){return points.map((p,i)=>(i?'L':'M')+p[0]+','+(-p[1])).join(' ')}
 const reference=el('path',{d:path([...ref,ref[0]]),fill:'none',stroke:colors[2],'stroke-width':3,opacity:.45,'vector-effect':'non-scaling-stroke'});
 for(const [field,color,label] of [['truth',colors[1],'Ground truth'],['xy',colors[0],`Estimated · ${D.source}`]]){
 let d='',active=false;for(const i of indices){const s=D.samples[i],p=s[field];if(!p?.every(Number.isFinite)){active=false;continue;}d+=(!active||!s.continuous?'M':'L')+p[0]+','+(-p[1]);active=true;}
 if(!d)continue;
 const width=field==='xy'?1.8:1.4;const line=el('path',{d,fill:'none',stroke:color,'stroke-width':width,'stroke-dasharray':field==='xy'?'1 3':'none','stroke-linecap':'round','vector-effect':'non-scaling-stroke'});
 for(let n=0;n<indices.length;n++){const i=indices[n],p=D.samples[i][field];if(!p?.every(Number.isFinite))continue;const before=n>0&&D.samples[i].continuous&&D.samples[indices[n-1]][field]?.every(Number.isFinite),after=n+1<indices.length&&D.samples[indices[n+1]].continuous&&D.samples[indices[n+1]][field]?.every(Number.isFinite);if(!before&&!after)sampleDots.push(el('circle',{cx:p[0],cy:-p[1],r:3,fill:color,class:'race-sample-dot','data-sample-index':i}));}
 const radius=field==='truth'?7:4.5;const marker=el('circle',{r:radius,fill:field==='truth'?'white':color,stroke:field==='truth'?color:'white','stroke-width':1.5,'vector-effect':'non-scaling-stroke',visibility:'hidden'});mapMarkers.push({field,marker,radius});
 const entry=document.createElement('span'),swatch=document.createElement('i');swatch.style.borderTop=`${field==='xy'?2:1.5}px ${field==='xy'?'dotted':'solid'} ${color}`;entry.append(swatch,document.createTextNode(label));legend.append(entry);
 }
 const referenceLabel=document.createElement('span'),referenceSwatch=document.createElement('i');referenceSwatch.style.borderTop=`3px solid ${colors[2]}`;referenceLabel.append(referenceSwatch,document.createTextNode('Saved raceline'));legend.append(referenceLabel);host.append(legend);
 const origin=el('circle',{cx:ref[0][0],cy:-ref[0][1],r:3,fill:colors[2]});
 el('title',{},origin).textContent='Reference origin: 0% / 100% progress';
 const start=D.samples[indices[0]];let startGroup=null;
 if(start){const [x,y]=start.xy,a=start.heading??0;
 startGroup=el('g',{});
 el('circle',{r:3,fill:'#172438'},startGroup);
 el('path',{d:'M0,0 L24,0 M19,-4 L24,0 L19,4',transform:`rotate(${-a*180/Math.PI})`,fill:'none',stroke:'#172438','stroke-width':2},startGroup);
 const label=el('text',{x:-5,y:14,'text-anchor':'middle','paint-order':'stroke',stroke:'white','stroke-width':2,'font-size':11,fill:'#172438'},startGroup);label.textContent='Lap start';
 }
 function sizeMarkers(){const scale=svg.getScreenCTM()?.a;if(!scale)return;const unit=1/Math.abs(scale);for(const dot of sampleDots)dot.setAttribute('r',3*unit);for(const item of mapMarkers)item.marker.setAttribute('r',item.radius*unit);origin.setAttribute('r',3*unit);if(startGroup)startGroup.setAttribute('transform',`translate(${start.xy[0]},${-start.xy[1]}) scale(${unit})`);}
 sizeMarkers();if(host.raceResize)host.raceResize.disconnect();host.raceResize=new ResizeObserver(sizeMarkers);host.raceResize.observe(svg);
 let frame=0,event=null;
 function nearest(e){const point=new DOMPoint(e.clientX,e.clientY).matrixTransform(svg.getScreenCTM().inverse());let best=null,dist=Infinity;for(const i of indices)for(const field of ['xy','truth']){const p=D.samples[i][field];if(!p?.every(Number.isFinite))continue;let d=(point.x-p[0])**2+(point.y+p[1])**2;if(d<dist){dist=d;best=i}}return best;}
 svg.onpointermove=e=>{event=e;if(frame)return;frame=requestAnimationFrame(()=>{frame=0;pointerInspect(nearest(event));})};
 svg.onclick=e=>pointerInspect(nearest(e),true);
 svg.onwheel=e=>{e.preventDefault();const p=new DOMPoint(e.clientX,e.clientY).matrixTransform(svg.getScreenCTM().inverse()),wanted=e.deltaY>0?1.15:1/1.15,next=Math.max(w*.001,Math.min(w*100,view[2]*wanted)),f=next/view[2];view[0]=p.x+(view[0]-p.x)*f;view[1]=p.y+(view[1]-p.y)*f;view[2]=next;view[3]*=f;svg.setAttribute('viewBox',view.join(' '));sizeMarkers();};
}

async function draw(version){
 if(version!==drawVersion)return;
 plots.forEach(p=>Plotly.purge(p));plots=[];q('.race-panels').replaceChildren();
 q('.race-readout').textContent='Hover to inspect; click to pin.';
 const l=D.laps[lap];indices=lapIndices(lap);indexSet=new Set(indices);positions=new Map(indices.map((i,n)=>[i,n]));timeIndices=new Map(indices.map(i=>[D.samples[i].t,i]));progressIndices=indices.filter(i=>Number.isFinite(D.samples[i].d)).sort((a,b)=>D.samples[a].d-D.samples[b].d);
 select.value=lap;q('.race-label').textContent=`Lap ${lap+1} of ${D.laps.length} · ${l.complete?'completed':'partial / interrupted'}`;
 q('.race-prev').disabled=lap===0;q('.race-next').disabled=lap===D.laps.length-1;
 q('.race-back').disabled=q('.race-forward').disabled=!indices.length;
 drawMap();
 for(const group of ['Motion and commands','Tracking','Localization','Clearance']){
 const details=document.createElement('details');details.open=group==='Motion and commands';const summary=document.createElement('summary');summary.textContent=group;details.append(summary);q('.race-panels').append(details);
 const panels=[...new Set(keys.filter(k=>k.split('|')[2]===group).map(k=>k.split('|')[3]))];
 if(!panels.length){details.append('Unavailable: required evidence was not recorded or could not be aligned.');continue;}
 for(const panel of panels){const ks=keys.filter(k=>k.split('|')[2]===group&&k.split('|')[3]===panel);const box=document.createElement('details');box.open=true;box.className='race-subpanel';const title=document.createElement('summary');title.textContent=panel;box.append(title);const holder=document.createElement('div');holder.className='race-panel';const div=document.createElement('div');holder.append(div);const cursorLine=document.createElement('div');cursorLine.className='race-cursor';holder.append(cursorLine);div.cursorLine=cursorLine;div.valueLabels=document.createElement('div');div.valueLabels.className='race-values';holder.append(div.valueLabels);box.append(holder);details.append(box);
 const ts=ks.map(k=>{let x=[],y=[],ids=[];for(const i of indices){let s=D.samples[i];if(!s.continuous || (i>0 && s.d!==null && D.samples[i-1].d!==null && Math.abs(s.d-D.samples[i-1].d)>D.length/2) || (k.endsWith('|Heading')&&i>0&&Number.isFinite(D.signals[k][i])&&Number.isFinite(D.signals[k][i-1])&&Math.abs(D.signals[k][i]-D.signals[k][i-1])>Math.PI)){x.push(null);y.push(null);ids.push(null)}x.push(s.d);y.push(s.d===null?null:D.signals[k][i]);ids.push(i)}return trace(k,x,y,ids)});
 const missing=ts.every(t=>t.y.every(v=>v===null));
 await Plotly.newPlot(div,ts,{annotations:missing?[{text:'Unavailable: no aligned samples for this lap',xref:'paper',yref:'paper',x:.5,y:.5,showarrow:false}]:[],height:330,margin:{t:20,b:135,l:55,r:15},xaxis:{showgrid:false,zeroline:false,title:{text:'Distance along reference raceline [m]'}},yaxis:{showgrid:false,zeroline:false,title:{text:ks[0].split('|')[1]}},legend:{orientation:'h',y:-.5,yanchor:'top',x:0,font:{size:11}},clickmode:'event',hovermode:'closest'},{responsive:true});plots.push(div);if(version!==drawVersion)return;wire(div,ks);
 box.ontoggle=()=>Plotly.Plots.resize(div);
 }details.ontoggle=()=>plots.forEach(p=>Plotly.Plots.resize(p));
 }
}
function intersectionLabels(p,i,x){
 const host=p.valueLabels;host.replaceChildren();const yaxis=p._fullLayout?.yaxis,axis=p._fullLayout?.xaxis;if(!yaxis||!axis)return;
 const values=[];
 for(const tr of p.data){if(tr.visible==='legendonly'||tr.visible===false)continue;const key=keys.find(k=>k.split('|')[0]===tr.name),value=D.signals[key]?.[i];if(value==null)continue;const y=yaxis.l2p(value);if(!Number.isFinite(y)||y<0||y>yaxis._length)continue;values.push({name:tr.name,value,unit:key.split('|')[1],y,color:tr.line.color});}
 values.sort((a,b)=>a.y-b.y);const crowded=values.length*24>yaxis._length;
 let list=null;if(crowded){list=document.createElement('div');list.className='race-value-list';list.style.left=(axis._offset+(x<axis._length/2?x+10:Math.max(0,x-200)))+'px';list.style.top=yaxis._offset+'px';list.style.maxHeight=yaxis._length+'px';host.append(list);}
 let next=0;for(const value of values){value.labelY=Math.max(value.y-10,next);next=value.labelY+24;}
 let limit=yaxis._length-22;for(let j=values.length-1;j>=0;j--){values[j].labelY=Math.min(values[j].labelY,limit);limit=values[j].labelY-24;}
 for(const v of values){const dot=document.createElement('span');dot.className='race-intersection';dot.style.background=v.color;dot.style.left=(axis._offset+x)+'px';dot.style.top=(yaxis._offset+v.y)+'px';host.append(dot);
 const label=document.createElement('div');label.className='race-value';label.style.borderLeftColor=v.color;label.textContent=`${v.name}: ${v.value.toFixed(3)} ${v.unit}`;label.title=label.textContent;
 if(crowded){list.append(label);}else{label.style.position='absolute';label.style.top=(yaxis._offset+Math.max(0,v.labelY))+'px';label.style.left=(axis._offset+x+(x<axis._length/2?10:-10))+'px';if(x>=axis._length/2)label.style.transform='translateX(-100%)';host.append(label);}}
}
function cursor(t){
 const i=selectedIndex();if(i===undefined)return;const s=D.samples[i],pinned=selection().pinned;t=s.t;lastCursor=t;
 const fields=[['Selected time',`${t.toFixed(3)} s${pinned?' · Pinned':''}`],['Reference progress',s.d===null?'Unavailable':`${s.d.toFixed(2)} m · ${(s.d/D.length*100).toFixed(1)}%`],['Position',`x ${s.xy[0].toFixed(2)} · y ${s.xy[1].toFixed(2)} m`],['Heading',`${(s.heading??0).toFixed(3)} rad`]];
 const readout=q('.race-readout');if(readout.children.length!==fields.length){readout.replaceChildren();for(const [label] of fields){const item=document.createElement('div'),name=document.createElement('span'),val=document.createElement('strong');name.textContent=label;item.append(name,val);readout.append(item);}}fields.forEach(([,value],n)=>{const val=readout.children[n].lastElementChild;if(val.textContent!==value)val.textContent=value;val.title=value;});
 for(const p of plots){const axis=p._fullLayout?.xaxis,line=p.cursorLine;if(!axis||s.d===null){line.style.display='none';p.valueLabels.replaceChildren();continue;}const x=axis.l2p(s.d),inside=x>=0&&x<=axis._length;line.style.display=inside?'block':'none';line.style.left=(axis._offset+x)+'px';line.style.top=p._fullLayout.yaxis._offset+'px';line.style.height=p._fullLayout.yaxis._length+'px';line.style.bottom='auto';if(inside&&pinned)intersectionLabels(p,i,x);else p.valueLabels.replaceChildren();}
 for(const {field,marker} of mapMarkers){const xy=s[field];const valid=xy?.every(Number.isFinite);marker.setAttribute('visibility',valid?'visible':'hidden');if(valid){marker.setAttribute('cx',xy[0]);marker.setAttribute('cy',-xy[1]);}}
}
let drawing=Promise.resolve(),cursorFrame=0,drawVersion=0;
function refreshSelection(){
 const state=selection(),i=selectedIndex();
 q('.race-unpin').disabled=!state.pinned;
 q('.race-back').disabled=!indices.length||state.pinned&&i===indices[0];
 q('.race-forward').disabled=!indices.length||state.pinned&&i===indices[indices.length-1];
 if(i!==undefined)cursor(state.timestamp);
 else {lastCursor=null;q('.race-readout').textContent='No localized pose samples for this lap.';for(const p of plots){p.cursorLine.style.display='none';p.valueLabels.replaceChildren();}for(const {marker} of mapMarkers)marker.setAttribute('visibility','hidden');}
}
window.benchmarkReport.subscribe(s=>{
 if(!Number.isInteger(s.lap)||!D.laps[s.lap])return;
 if(s.lap!==lap){lap=s.lap;lastCursor=null;indices=lapIndices(lap);indexSet=new Set(indices);positions=new Map(indices.map((i,n)=>[i,n]));timeIndices=new Map(indices.map(i=>[D.samples[i].t,i]));progressIndices=indices.filter(i=>Number.isFinite(D.samples[i].d)).sort((a,b)=>D.samples[a].d-D.samples[b].d);const version=++drawVersion;drawing=drawing.then(()=>draw(version)).then(()=>{if(version===drawVersion)refreshSelection()});}
 else if(!cursorFrame)cursorFrame=requestAnimationFrame(()=>{cursorFrame=0;drawing.then(refreshSelection)});
});choose(0);
