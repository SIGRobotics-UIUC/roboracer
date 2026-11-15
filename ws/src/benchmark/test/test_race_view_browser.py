import numpy as np
from pathlib import Path
from tempfile import TemporaryDirectory
from benchmark.analysis.core.signals import Dataset,Sample
from benchmark.analysis.modules.race_view import payload,render_report
from benchmark.reports.render import SHARED
import pytest
playwright = pytest.importorskip('playwright.sync_api')
sync_playwright = playwright.sync_playwright


def test_offline_lap_zoom_visibility_and_pin():
    rows=np.array([[0,0,0,-np.pi/2,0,2],[2,2,0,0,0,2],[4,2,2,np.pi/2,0,2],[6,0,2,np.pi,0,2]])
    samples=[Sample(int(t*1e9),int(t*1e9),[float(t%1),0,0],order=i) for i,t in enumerate(np.arange(1,3,.05))]
    data=Dataset(signals={'pose':samples,'lap_count':[Sample(None,int(t*1e9),n,order=i) for i,(t,n) in enumerate([(1,0),(2,1),(3,2)])]},metadata={})
    v=payload(data,rows,[],{})
    v['samples'][10]['d']=7.9
    v['samples'][11]['d']=.1
    with TemporaryDirectory() as d:
        path=Path(d)/'report.html';path.write_text(SHARED+render_report({'viewer':v,'metrics':{}}))
        with sync_playwright() as p:
            try:
                browser=p.chromium.launch(headless=True,args=['--no-sandbox'])
            except playwright.Error as exc:
                if 'Executable doesn' in str(exc):
                    pytest.skip('Playwright Chromium is not installed')
                raise
            page=browser.new_page();errors=[];page.on('pageerror',lambda e:errors.append(str(e)))
            page.goto(path.as_uri());page.wait_for_selector('.race-panels .js-plotly-plot');page.wait_for_timeout(1000)
            assert page.evaluate('''() => {const x=document.querySelector('.race-panels .js-plotly-plot').data[0].x;return x.every((v,i)=>i===0||v===null||x[i-1]===null||Math.abs(v-x[i-1])<=4)}''')
            page.evaluate('''async () => {const p=document.querySelector('.race-panels .js-plotly-plot');p.emit('plotly_legendclick',{curveNumber:0});await Plotly.restyle(p,{visible:'legendonly'},[0]);await Plotly.relayout(p,{'xaxis.range':[.1,.5]});}''');page.wait_for_timeout(300)
            assert page.evaluate('''() => [...document.querySelectorAll('.race-panels .js-plotly-plot')].every(p=>p.layout.xaxis.range[0]===.1)''')
            page.locator('.race-next').click();page.wait_for_timeout(800)
            assert page.evaluate('''() => document.querySelector('.race-panels .js-plotly-plot').data[0].visible''') == 'legendonly'
            assert 'Lap 2 of' in page.locator('.race-label').inner_text()
            page.evaluate('''() => {const p=document.querySelector('.race-panels .js-plotly-plot');p.emit('plotly_click',{points:[{customdata:25}]});}''');page.wait_for_timeout(300)
            assert page.locator('.race-readout > div').count() == 4
            assert 'Pinned' in page.locator('.race-readout').inner_text()
            assert page.evaluate('window.benchmarkReport.getSelection().lap') == 1
            assert float(page.locator('.race-map circle[visibility=visible]').first.get_attribute('cx')) == v['samples'][25]['xy'][0]
            assert ' m' in page.locator('.race-readout').inner_text()
            assert not errors,errors
            print('Browser lap navigation and pinned cursor: passed')
            browser.close()
