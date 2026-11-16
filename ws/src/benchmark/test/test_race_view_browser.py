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


def test_shared_position_navigation_hover_and_escape(tmp_path):
    # Deliberately irregular time spacing and an unavailable progress sample.
    viewer = {'laps': [{'start': 0, 'end': 3, 'complete': True},
                       {'start': 4, 'end': 5, 'complete': False},
                       {'start': 10, 'end': 11, 'complete': False}],
              'samples': [{'t': t, 'recorded': i, 'xy': [i, 0], 'heading': 0,
                           'truth': [None, None], 'd': None if i == 2 else i,
                           'continuous': True}
                          for i, t in enumerate([0, .03, .2, .4, .8, 1.])],
              'signals': {'Test speed|m/s|Motion and commands|Speed': [1, 2, None, 4, 5, 6]},
              'reference': [[0, 0], [5, 0], [5, 1]], 'length': 12, 'source': 'test'}
    path = tmp_path/'viewer.html'
    path.write_text(SHARED+render_report({'viewer': viewer, 'metrics': {}}))
    with sync_playwright() as p:
        try:
            browser = p.chromium.launch(headless=True, args=['--no-sandbox'])
        except playwright.Error as exc:
            if 'Executable doesn' in str(exc):
                pytest.skip('Playwright Chromium is not installed')
            raise
        page = browser.new_page(viewport={'width': 1200, 'height': 900})
        errors = []
        page.on('pageerror', lambda e: errors.append(str(e)))
        page.goto(path.as_uri())
        page.wait_for_selector('.race-panels .js-plotly-plot')
        plot = "document.querySelector('.race-panels .js-plotly-plot')"

        def state():
            return page.evaluate('window.benchmarkReport.getSelection()')

        def wait_index(i):
            page.wait_for_function('i => window.benchmarkReport.getSelection().sampleIndex === i', arg=i)
            page.wait_for_timeout(100)

        assert page.locator('.race-map .race-controls').count() == 1
        assert page.locator('.race-map .race-readout').count() == 1
        page.evaluate(f"{plot}.emit('plotly_hover',{{points:[{{customdata:1}}]}})")
        wait_index(1)
        assert not state()['pinned']
        assert page.locator('.race-value').count() == 0
        assert page.evaluate(f"{plot}.data[0].hoverinfo") == 'none'
        page.evaluate(f"{plot}.emit('plotly_click',{{points:[{{customdata:1}}]}})")
        page.wait_for_selector('.race-value')
        assert state()['pinned']
        page.evaluate(f"{plot}.emit('plotly_hover',{{points:[{{customdata:3}}]}})")
        assert state()['sampleIndex'] == 1

        page.locator('.race-forward').click()
        wait_index(2)
        assert 'Unavailable' in page.locator('.race-readout').inner_text()
        assert page.locator('.race-value').count() == 0
        # Keyboard navigation from plot focus uses the same adjacent-sample increment.
        page.evaluate(f"{plot}.tabIndex=0;{plot}.focus()")
        page.keyboard.press('ArrowRight')
        wait_index(3)
        assert float(page.locator('.race-track circle[visibility=visible]').first.get_attribute('cx')) == 3
        page.keyboard.press('ArrowRight')
        assert state()['sampleIndex'] == 3  # clamped to this lap
        page.keyboard.press('ArrowLeft')
        wait_index(2)
        page.keyboard.press('Escape')
        page.wait_for_function('!window.benchmarkReport.getSelection().pinned')
        assert page.locator('.race-value').count() == 0

        # Form controls keep their own keyboard behavior.
        page.evaluate("const input=document.createElement('input');document.body.append(input);input.focus()")
        page.keyboard.press('ArrowRight')
        assert state()['sampleIndex'] == 2
        page.locator('.race-select').focus()
        page.keyboard.press('ArrowLeft')
        assert state()['sampleIndex'] == 2
        page.locator('.race-forward').focus()
        page.keyboard.press('ArrowLeft')
        assert state()['sampleIndex'] == 2

        # Track hover and click still flow through the same selection.
        svg = page.locator('.race-track svg')
        point = svg.evaluate('s => {const p=new DOMPoint(1,0).matrixTransform(s.getScreenCTM());return {x:p.x,y:p.y}}')
        page.mouse.move(point['x'], point['y'])
        wait_index(1)
        page.mouse.click(point['x'], point['y'])
        page.wait_for_function('window.benchmarkReport.getSelection().pinned')
        # Zoom changes geometry, not screen-space line and vehicle-marker sizes.
        marker = page.locator('.race-track circle[visibility=visible]').first
        before = marker.bounding_box()['width']
        for delta in [-100]*30 + [100]*60:
            svg.evaluate('(s,d) => {const r=s.getBoundingClientRect();s.dispatchEvent(new WheelEvent("wheel",{clientX:r.x+r.width/2,clientY:r.y+r.height/2,deltaY:d,cancelable:true}))}', delta)
        assert abs(marker.bounding_box()['width']-before) < .1
        assert svg.locator('path[stroke-dasharray="1 3"]').get_attribute('stroke-width') == '1.8'
        # Readout fields keep one row despite longer numbers and missing progress.
        height = page.locator('.race-readout').bounding_box()['height']
        page.evaluate('window.benchmarkReport.requestSelection({sampleIndex:2,timestamp:.2,pinned:true})')
        wait_index(2)
        assert abs(page.locator('.race-readout').bounding_box()['height']-height) < 1
        assert page.locator('.race-controls').evaluate('e => new Set([...e.children].map(c=>c.offsetTop)).size') == 2
        assert page.locator('.race-readout').evaluate('e => new Set([...e.children].map(c=>c.offsetTop)).size') == 1
        # Real pointer movement over the plot selects by progress without native
        # trace-proximity hover being required.
        page.keyboard.press('Escape')
        page.evaluate(f'{plot}.scrollIntoView()')
        point = page.evaluate(f'() => {{const p={plot},r=p.getBoundingClientRect(),a=p._fullLayout.xaxis,y=p._fullLayout.yaxis;return {{x:r.x+a._offset+a.l2p(1),y:r.y+y._offset+10}}}}')
        page.mouse.move(point['x'], point['y'])
        wait_index(1)
        page.locator('.race-next').click()
        wait_index(4)
        assert not state()['pinned']
        page.locator('.race-next').click()
        page.wait_for_function('window.benchmarkReport.getSelection().lap === 2')
        page.wait_for_function("document.querySelector('.race-readout').textContent.includes('No localized pose')")
        assert not state()['pinned'] and state()['timestamp'] is None
        assert page.locator('.race-forward').is_disabled()
        assert not errors, errors
        browser.close()


def test_isolated_recorded_points_visible_and_gap_clicks_snap_to_samples(tmp_path):
    viewer = {'laps': [{'start': 0, 'end': 2, 'complete': False}],
              'samples': [{'t': i, 'recorded': i, 'xy': [i*5,0], 'heading': 0,
                           'truth': [None,None], 'd': i*5, 'continuous': False}
                          for i in range(3)],
              'signals': {'Recorded speed|m/s|Motion and commands|Speed': [1,2,3]},
              'reference': [[0,0],[10,0],[10,2]], 'length': 24, 'source': 'test'}
    path = tmp_path/'gaps.html'
    path.write_text(SHARED+render_report({'viewer': viewer, 'metrics': {}}))
    with sync_playwright() as p:
        try:
            browser = p.chromium.launch(headless=True, args=['--no-sandbox'])
        except playwright.Error as exc:
            if 'Executable doesn' in str(exc):
                pytest.skip('Playwright Chromium is not installed')
            raise
        page = browser.new_page(viewport={'width': 1200, 'height': 900})
        errors = []
        page.on('pageerror', lambda e: errors.append(str(e)))
        page.goto(path.as_uri())
        page.wait_for_selector('.race-panels .js-plotly-plot')
        plot = "document.querySelector('.race-panels .js-plotly-plot')"
        page.wait_for_function(f'{plot}.data[0].marker.size.filter(s=>s>0).length===3')
        assert page.locator('.race-sample-dot').count() == 3
        svg = page.locator('.race-track svg')
        for x, should_pin in [(2.5, True), (5, True)]:
            point = svg.evaluate('(s,x)=>{const p=new DOMPoint(x,0).matrixTransform(s.getScreenCTM());return {x:p.x,y:p.y}}', x)
            page.mouse.click(point['x'], point['y'])
            page.wait_for_timeout(150)
            assert page.evaluate('benchmarkReport.getSelection().pinned') == should_pin
        assert page.evaluate('benchmarkReport.getSelection().sampleIndex') == 1
        page.locator('.race-unpin').click()
        page.evaluate(f'{plot}.scrollIntoView()')
        for d, should_pin in [(2.5, True), (5, True)]:
            point = page.evaluate(f'''d=>{{const p={plot},r=p.getBoundingClientRect(),a=p._fullLayout.xaxis,y=p._fullLayout.yaxis;
                return {{x:r.x+a._offset+a.l2p(d),y:r.y+y._offset+10}}}}''', d)
            page.mouse.click(point['x'], point['y'])
            page.wait_for_timeout(150)
            assert page.evaluate('benchmarkReport.getSelection().pinned') == should_pin
            assert page.evaluate('benchmarkReport.getSelection().sampleIndex') in (0, 1)
            assert page.locator('.race-selection-hint').count() == 0
        assert page.evaluate('benchmarkReport.getSelection().sampleIndex') == 1
        assert not errors, errors
        browser.close()
