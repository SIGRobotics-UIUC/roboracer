const root=document.getElementById('race-performance'), q=s=>root.querySelector(s);
if(!D.laps.length){q('.race-label').textContent=D.reason||'No measurable laps available';return;}
let lap=-1,pinned=false,plots=[],indices=[],sync=false;
const visibility={},colors=['#0671ee','#e63572','#768397','#a14b9c','#796618','#537784'];
const keys=Object.keys(D.signals),color=k=>colors[/truth|True/.test(k)?1:/Raceline|raceline/.test(k)?2:/request/.test(k)?3:/gate/.test(k)?4:/odometry/.test(k)?5:0];
function trace(key,x,y,custom){return {name:key.split('|')[0],x,y,customdata:custom,mode:'lines',connectgaps:false,line:{color:color(key)},visible:visibility[key]??true};}
const select=q('.race-select');
D.laps.forEach((l,i)=>{let o=document.createElement('option');o.value=i;o.textContent=`Lap ${i+1}${l.complete?'':' (partial)'}`;select.append(o);});
const times=D.laps.filter(l=>l.complete&&l.valid&&l.time_s>0).map(l=>l.time_s),best=Math.min(...times);
const table=document.createElement('table');
let header=document.createElement('tr');['Lap','Classification','Native time (s)','Completed'].forEach(t=>{let e=document.createElement('th');e.textContent=t;header.append(e)});table.append(header);
D.laps.forEach((l,i)=>{let row=document.createElement('tr');[i+1,l.start_kind||'Classification unavailable',l.valid?l.time_s:'Unavailable',l.complete?'Yes':'No'].forEach(t=>{let e=document.createElement('td');e.textContent=t;row.append(e)});if(l.complete&&l.valid&&l.time_s===best){row.style.background='#e0f4df';row.title='Fastest completed lap';row.children[2].style.fontWeight='750';}row.tabIndex=0;row.onclick=()=>choose(i);row.onkeydown=e=>{if(e.key==='Enter')choose(i)};table.append(row)});q('.race-laps').append(table);
function choose(i){if(i<0||i>=D.laps.length)return;window.benchmarkReport.requestSelection({lap:i,timestamp:null});}
q('.race-prev').onclick=()=>choose(lap-1);q('.race-next').onclick=()=>choose(lap+1);select.onchange=()=>choose(+select.value);q('.race-unpin').onclick=()=>{pinned=false;q('.race-readout').textContent='Hover to inspect; click to pin.'};
function inspect(i,pin=false){if(i==null||pinned&&!pin)return;pinned=pin;window.benchmarkReport.requestSelection({lap,timestamp:D.samples[i].t});}
function wire(div,signalKeys,map=false){
 div.on('plotly_hover',e=>{const i=e.points[0]?.customdata;if(Number.isInteger(i))inspect(i)});
 div.on('plotly_click',e=>{div.racePointClick=performance.now();const i=e.points[0]?.customdata;if(Number.isInteger(i))inspect(i,true)});
 let down=null;div.addEventListener('pointerdown',e=>{down=[e.clientX,e.clientY]});
 div.addEventListener('click',e=>{if(!down||Math.hypot(e.clientX-down[0],e.clientY-down[1])>4||e.target.closest('.legend,.modebar'))return;const at=performance.now(),rect=div.getBoundingClientRect(),axis=div._fullLayout?.xaxis,yaxis=div._fullLayout?.yaxis;if(!axis||!yaxis)return;const px=e.clientX-rect.left-axis._offset,py=e.clientY-rect.top-yaxis._offset;if(px<0||px>axis._length||py<0||py>yaxis._length)return;requestAnimationFrame(()=>{if(div.racePointClick>=at-10)return;const distance=axis.p2l(px);let best=null,delta=Infinity;for(const i of indices){const d=D.samples[i].d;if(d===null)continue;const error=Math.abs(d-distance);if(error<delta){best=i;delta=error}}inspect(best,true)});});
 div.on('plotly_restyle',()=>{if(lastCursor!==null)requestAnimationFrame(()=>cursor(lastCursor))});
 div.on('plotly_legendclick',e=>{const k=signalKeys[e.curveNumber];if(k){visibility[k]=div.data[e.curveNumber].visible==='legendonly'?true:'legendonly';} });
 if(!map)div.on('plotly_relayout',e=>{if(sync)return;let update;if(e['xaxis.range'])update={'xaxis.range':e['xaxis.range']};else if(e['xaxis.range[0]']!==undefined)update={'xaxis.range':[e['xaxis.range[0]'],e['xaxis.range[1]']]};else if(e['xaxis.autorange'])update={'xaxis.autorange':true};if(update){sync=true;Promise.all(plots.filter(p=>p!==div).map(p=>Plotly.relayout(p,update))).finally(()=>{sync=false;if(lastCursor!==null)cursor(lastCursor)})}});
}
let mapMarkers=[],lastCursor=null;
function drawMap(){
 const host=q('.race-map');host.replaceChildren();mapMarkers=[];
 const legend=document.createElement('div');legend.className='race-map-legend';const legendTitle=document.createElement('strong');legendTitle.textContent='Track legend';legend.append(legendTitle);
 const NS='http://www.w3.org/2000/svg',svg=document.createElementNS(NS,'svg');host.append(svg);
 function el(tag,attrs,parent=svg){const e=document.createElementNS(NS,tag);for(const [k,v] of Object.entries(attrs))e.setAttribute(k,v);parent.append(e);return e;}
 const ref=D.reference,all=ref.concat(indices.map(i=>D.samples[i].xy));
 let xs=all.map(p=>p[0]),ys=all.map(p=>-p[1]);let minX=xs.reduce((a,b)=>Math.min(a,b),Infinity),minY=ys.reduce((a,b)=>Math.min(a,b),Infinity),w=xs.reduce((a,b)=>Math.max(a,b),-Infinity)-minX,h=ys.reduce((a,b)=>Math.max(a,b),-Infinity)-minY;
 const pad=Math.max(w,h)*.08;w+=2*pad;h+=2*pad;minX-=pad;minY-=pad;
 const zoomStrokes=[],view=[minX,minY,w,h];svg.setAttribute('viewBox',view.join(' '));svg.setAttribute('aria-label','Track map with lap start and direction');
 if(D.map){const m=D.map,[x,y,a]=m.origin,c=Math.cos(a),s=Math.sin(a),r=m.resolution;
 el('image',{href:m.image,width:m.width,height:m.height,transform:`matrix(${c*r} ${-s*r} ${s*r} ${c*r} ${x-s*m.height*r} ${-y-c*m.height*r})`});}
 function path(points){return points.map((p,i)=>(i?'L':'M')+p[0]+','+(-p[1])).join(' ')}
 const reference=el('path',{d:path([...ref,ref[0]]),fill:'none',stroke:colors[2],'stroke-width':3,opacity:.45,'vector-effect':'non-scaling-stroke'});zoomStrokes.push({element:reference,width:3});
 for(const [field,color,label] of [['truth',colors[1],'Ground truth'],['xy',colors[0],`Estimated · ${D.source}`]]){
 let d='',active=false;for(const i of indices){const s=D.samples[i],p=s[field];if(!p?.every(Number.isFinite)){active=false;continue;}d+=(!active||!s.continuous?'M':'L')+p[0]+','+(-p[1]);active=true;}
 if(!d)continue;
 const width=field==='xy'?1.8:1.4;const line=el('path',{d,fill:'none',stroke:color,'stroke-width':width,'stroke-dasharray':field==='xy'?'1 3':'none','stroke-linecap':'round','vector-effect':'non-scaling-stroke'});zoomStrokes.push({element:line,width,dotted:field==='xy'});
 const radius=Math.max(w,h)*(field==='truth' ? .009 : .006);const marker=el('circle',{r:radius,fill:field==='truth'?'white':color,stroke:field==='truth'?color:'white','stroke-width':1.5,'vector-effect':'non-scaling-stroke',visibility:'hidden'});mapMarkers.push({field,marker,radius});
 const entry=document.createElement('span'),swatch=document.createElement('i');swatch.style.borderTop=`${field==='xy'?2:1.5}px ${field==='xy'?'dotted':'solid'} ${color}`;entry.append(swatch,document.createTextNode(label));legend.append(entry);
 }
 const referenceLabel=document.createElement('span'),referenceSwatch=document.createElement('i');referenceSwatch.style.borderTop=`3px solid ${colors[2]}`;referenceLabel.append(referenceSwatch,document.createTextNode('Saved raceline'));legend.append(referenceLabel);host.append(legend);
 const start=D.samples[indices[0]];
 if(start){const [x,y]=start.xy,a=start.heading??0,step=Math.max(w,h)*.06;
 el('circle',{cx:x,cy:-y,r:step*.1,fill:'#172438'});
 el('path',{d:`M${x},${-y}L${x+Math.cos(a)*step},${-y-Math.sin(a)*step}`,stroke:'#172438','stroke-width':2,'vector-effect':'non-scaling-stroke'});
 const tip=[x+Math.cos(a)*step,-y-Math.sin(a)*step];
 el('path',{d:`M${tip[0]-Math.cos(a-.5)*step*.25},${tip[1]+Math.sin(a-.5)*step*.25}L${tip[0]},${tip[1]}L${tip[0]-Math.cos(a+.5)*step*.25},${tip[1]+Math.sin(a+.5)*step*.25}`,fill:'none',stroke:'#172438','stroke-width':2,'vector-effect':'non-scaling-stroke'});
 const label=el('text',{x:x-Math.cos(a)*step*.5+Math.sin(a)*step*.6,y:-y+Math.sin(a)*step*.5+Math.cos(a)*step*.6,'text-anchor':'middle','paint-order':'stroke','stroke':'white','stroke-width':step*.08,'font-size':step*.35,fill:'#172438'});label.textContent='Start →';
 }
 let frame=0,event=null;
 function nearest(e){const point=new DOMPoint(e.clientX,e.clientY).matrixTransform(svg.getScreenCTM().inverse());let best=null,dist=Infinity;for(const i of indices)for(const field of ['xy','truth']){const p=D.samples[i][field];if(!p?.every(Number.isFinite))continue;let d=(point.x-p[0])**2+(point.y+p[1])**2;if(d<dist){dist=d;best=i}}return best;}
 svg.onpointermove=e=>{event=e;if(frame)return;frame=requestAnimationFrame(()=>{frame=0;inspect(nearest(event));})};
 svg.onclick=e=>inspect(nearest(e),true);
 svg.onwheel=e=>{e.preventDefault();const p=new DOMPoint(e.clientX,e.clientY).matrixTransform(svg.getScreenCTM().inverse()),f=e.deltaY>0?1.15:1/1.15;view[0]=p.x+(view[0]-p.x)*f;view[1]=p.y+(view[1]-p.y)*f;view[2]*=f;view[3]*=f;svg.setAttribute('viewBox',view.join(' '));const scale=Math.sqrt(view[2]/w);for(const item of zoomStrokes){item.element.setAttribute('stroke-width',item.width*scale);if(item.dotted)item.element.setAttribute('stroke-dasharray',`${scale} ${3*scale}`)}for(const item of mapMarkers){item.marker.setAttribute('r',item.radius*Math.pow(view[2]/w,1.5));item.marker.setAttribute('stroke-width',1.5*scale)}};
}

async function draw(){
 plots.forEach(p=>Plotly.purge(p));plots=[];q('.race-panels').replaceChildren();
 q('.race-readout').textContent='Hover to inspect; click to pin.';
 const l=D.laps[lap];indices=D.samples.map((s,i)=>s.recorded>=l.start&&s.recorded<=l.end?i:null).filter(i=>i!==null);
 select.value=lap;q('.race-label').textContent=`Lap ${lap+1} of ${D.laps.length} · ${l.complete?'completed':'partial / interrupted'}`;
 q('.race-prev').disabled=lap===0;q('.race-next').disabled=lap===D.laps.length-1;
 drawMap();
 for(const group of ['Motion and commands','Tracking','Localization','Clearance']){
 const details=document.createElement('details');details.open=group==='Motion and commands';const summary=document.createElement('summary');summary.textContent=group;details.append(summary);q('.race-panels').append(details);
 const panels=[...new Set(keys.filter(k=>k.split('|')[2]===group).map(k=>k.split('|')[3]))];
 if(!panels.length){details.append('Unavailable: required evidence was not recorded or could not be aligned.');continue;}
 for(const panel of panels){const ks=keys.filter(k=>k.split('|')[2]===group&&k.split('|')[3]===panel);const box=document.createElement('details');box.open=true;box.className='race-subpanel';const title=document.createElement('summary');title.textContent=panel;box.append(title);const holder=document.createElement('div');holder.className='race-panel';const div=document.createElement('div');holder.append(div);const cursorLine=document.createElement('div');cursorLine.className='race-cursor';holder.append(cursorLine);div.cursorLine=cursorLine;div.valueLabels=document.createElement('div');div.valueLabels.className='race-values';holder.append(div.valueLabels);box.append(holder);details.append(box);
 const ts=ks.map(k=>{let x=[],y=[],ids=[];for(const i of indices){let s=D.samples[i];if(!s.continuous || (i>0 && s.d!==null && D.samples[i-1].d!==null && Math.abs(s.d-D.samples[i-1].d)>D.length/2)){x.push(null);y.push(null);ids.push(null)}x.push(s.d);y.push(s.d===null?null:D.signals[k][i]);ids.push(i)}return trace(k,x,y,ids)});
 const missing=ts.every(t=>t.y.every(v=>v===null));
 await Plotly.newPlot(div,ts,{annotations:missing?[{text:'Unavailable: no aligned samples for this lap',xref:'paper',yref:'paper',x:.5,y:.5,showarrow:false}]:[],height:330,margin:{t:20,b:135,l:55,r:15},xaxis:{showgrid:false,zeroline:false,title:{text:'Distance along reference raceline [m]'}},yaxis:{showgrid:false,zeroline:false,title:{text:ks[0].split('|')[1]}},legend:{orientation:'h',y:-.5,yanchor:'top',x:0,font:{size:11}},clickmode:'event',hovermode:'closest'},{responsive:true});plots.push(div);wire(div,ks);
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
 const i=indices.find(i=>D.samples[i].t===t);if(i===undefined)return;lastCursor=t;const s=D.samples[i];
 const fields=[['Selected time',`${t.toFixed(3)} s${pinned?' · Pinned':''}`],['Progress',s.d===null?'Unavailable':`${s.d.toFixed(2)} m · ${(s.d/D.length*100).toFixed(1)}%`],['Position',`x ${s.xy[0].toFixed(2)} · y ${s.xy[1].toFixed(2)} m`],['Heading',`${(s.heading??0).toFixed(3)} rad`]];
 q('.race-readout').replaceChildren();for(const [label,value] of fields){const item=document.createElement('div'),name=document.createElement('span'),val=document.createElement('strong');name.textContent=label;val.textContent=value;item.append(name,val);q('.race-readout').append(item)}
 for(const p of plots){const axis=p._fullLayout?.xaxis,line=p.cursorLine;if(!axis||s.d===null){line.style.display='none';p.valueLabels.replaceChildren();continue;}const x=axis.l2p(s.d),inside=x>=0&&x<=axis._length;line.style.display=inside?'block':'none';line.style.left=(axis._offset+x)+'px';line.style.top=p._fullLayout.yaxis._offset+'px';line.style.height=p._fullLayout.yaxis._length+'px';line.style.bottom='auto';if(inside)intersectionLabels(p,i,x);else p.valueLabels.replaceChildren();}
 for(const {field,marker} of mapMarkers){const xy=s[field];const valid=xy?.every(Number.isFinite);marker.setAttribute('visibility',valid?'visible':'hidden');if(valid){marker.setAttribute('cx',xy[0]);marker.setAttribute('cy',-xy[1]);}}
}
let drawing=Promise.resolve(),cursorFrame=0,pendingTime=null;
window.benchmarkReport.subscribe(s=>{if(s.lap!==lap){lap=s.lap;pinned=false;lastCursor=null;drawing=drawing.then(draw)}else if(s.timestamp!==null){pendingTime=s.timestamp;if(!cursorFrame)cursorFrame=requestAnimationFrame(()=>{cursorFrame=0;drawing.then(()=>cursor(pendingTime))})}});choose(0);
