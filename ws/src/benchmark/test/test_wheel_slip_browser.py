"""Module contributions share the original SVG track and race plot controls."""
import json
from pathlib import Path
import pytest
from benchmark.analysis.core.signals import Dataset, Sample
from benchmark.analysis.modules.wheel_contact import analyze
from benchmark.analysis.modules.wheel_contact_view import render_report as summary, extension
from benchmark.analysis.modules.race_view import render_report
from benchmark.reports.render import SHARED
playwright = pytest.importorskip('playwright.sync_api')


def test_shared_track_overlay_and_extension_observations(tmp_path):
    contact = json.loads((Path(__file__).resolve().parents[4]/'docs/fixtures/wheel_contact_v1.json').read_text())
    samples = []
    for i in range(4):
        contact['stamp_ns'] = 1_000_000_000+i*10_000_000
        contact['sample_sequence'] = i
        samples.append(Sample(contact['stamp_ns'], contact['stamp_ns'], json.loads(json.dumps(contact)), order=i))
    result = analyze(Dataset(signals={'wheel_contact': samples}), tmp_path, {})
    v = result['viewer']; v['length'] = 10
    for i, s in enumerate(v['samples']):
        s.update(xy=[i,0], d=i, progress_percent=i*10, lap=i//2, continuous=i%2 == 1)
        s['pose'] = [i,0,0]
    for i, sample in enumerate(v['samples']):
        for wheel in sample['wheels'].values():
            for direction in ('forward', 'sideways'):
                wheel[direction]['ratio'] = None if i == 3 else 0
        if i in (1,2):
            sample['wheels']['front_right']['sideways']['ratio'] = i
    # Contact stamps differ from base pose stamps, exercising namespaced selection.
    base = {'time_origin_ns':'1000000000', 'length':10, 'reference':[[0,0],[4,0],[4,1]], 'source':'fixture',
            'laps':[{'start':1_000_000_000,'end':1_015_000_000,'complete':True}, {'start':1_015_000_001,'end':1_040_000_000,'complete':False}],
            'samples':[{'t':.001+i*.01,'recorded':1_000_000_000+i*10_000_000,'xy':[i,0], 'truth':[i,.1], 'heading':0,'d':i,'continuous':i%2==1} for i in range(4)],
            'signals':{'Speed|m/s|Motion and commands|Speed':[1,2,3,4]}}
    ext = extension(result)['viewer']
    path = tmp_path/'report.html';path.write_text(SHARED+render_report({'viewer':base,'extensions':[ext]})+summary(result))
    with playwright.sync_playwright() as p:
        try:
            browser = p.chromium.launch(headless=True, args=['--no-sandbox'])
        except playwright.Error as exc:
            if 'Executable doesn' in str(exc): pytest.skip('Playwright Chromium unavailable')
            raise
        page=browser.new_page();errors=[];page.on('pageerror',lambda e:errors.append(str(e)))
        page.goto(path.as_uri());page.wait_for_function("document.querySelectorAll('.race-panel .js-plotly-plot').length===5")
        page.wait_for_timeout(600)
        assert page.locator('.race-track svg').count()==1
        assert page.locator('.slip-map').count()==0
        assert page.locator('.race-overlay').input_value()==''
        assert page.locator('.race-map-legend').inner_text().find('Ground truth')>=0
        details=page.locator('.race-panels > details').filter(has=page.locator('summary',has_text='Wheel slip'))
        details.locator('summary').first.click();page.wait_for_timeout(300)
        slip=details.locator('.js-plotly-plot')
        assert slip.count()==4
        assert slip.nth(2).evaluate('p=>p.layout.shapes[0].y0')==1
        assert slip.first.evaluate('p=>p.data.length')==4
        total=page.locator('.slip-summary').inner_text()
        options=page.locator('.race-overlay option').evaluate_all('(os)=>os.map(o=>({value:o.value,text:o.textContent}))')
        assert len(options)==2
        ratio=next(o['value'] for o in options if o['text']=='Slip')
        assert details.locator(':scope > p').count()==0
        page.locator('.race-overlay').select_option(ratio)
        assert page.locator('.race-overlay-dot').count()==2
        assert page.locator('.race-overlay-dot').evaluate_all('(ds)=>ds.map(d=>d.getAttribute("fill"))')==['rgb(22,163,74)','rgb(250,204,21)']
        slip.first.evaluate("p=>p.emit('plotly_click',{points:[{customdata:1}]})")
        page.wait_for_function("window.benchmarkReport.getSelection().extension==='wheel_contact'")
        state=page.evaluate('window.benchmarkReport.getSelection()')
        assert state['sampleIndex'] is None and state['observationIndex']==1
        page.wait_for_timeout(150)
        assert 'Pinned' in page.locator('.race-readout').inner_text()
        page.locator('.race-extension-readout > summary').click()
        assert 'runtime peak' in page.locator('.race-extension-readout').inner_text()
        assert page.locator('.race-map circle[visibility=visible]').first.get_attribute('cx')=='1'
        # No exact base stamp: no invented base telemetry or truth position.
        assert page.locator('.race-track circle[visibility=visible]').count()==1
        slip.first.focus();page.keyboard.press('ArrowLeft')
        page.wait_for_function("window.benchmarkReport.getSelection().observationIndex===0")
        page.locator('.race-unpin').click()
        # Preserve both top-level groups and nested panels, including closed ones.
        details.locator('.race-subpanel > summary').first.click()
        groups=page.locator('.race-panels > details')
        groups.nth(0).locator(':scope > summary').click()
        groups.nth(1).locator(':scope > summary').click()
        expected=page.locator('.race-panels details').evaluate_all('(nodes)=>nodes.map(n=>[n.dataset.panelKey,n.open])')
        page.locator('.race-select').select_option('1');page.wait_for_timeout(700)
        assert page.locator('.race-panels details').evaluate_all('(nodes)=>nodes.map(n=>[n.dataset.panelKey,n.open])')==expected
        page.locator('.race-select').select_option('0');page.wait_for_timeout(700)
        assert page.locator('.race-panels details').evaluate_all('(nodes)=>nodes.map(n=>[n.dataset.panelKey,n.open])')==expected
        page.locator('.race-select').select_option('1');page.wait_for_timeout(700)
        assert page.locator('.slip-summary').inner_text()==total
        assert page.locator('.race-overlay').input_value()==ratio
        assert page.locator('.race-overlay-dot').evaluate_all('(ds)=>ds.map(d=>d.getAttribute("fill"))')==['rgb(220,38,38)','#a8adb5']
        page.locator('.race-overlay').select_option('')
        assert page.locator('.race-overlay-dot').count()==0
        assert 'Ground truth' in page.locator('.race-map-legend').inner_text()
        page.set_viewport_size({'width':600,'height':800})
        assert page.locator('.race-inspect').evaluate('e=>getComputedStyle(e).display')=='block'
        for width in (600,360):
            page.set_viewport_size({'width':width,'height':800})
            assert page.locator('.race-readout').evaluate('e=>new Set([...e.children].map(c=>c.offsetTop)).size')==1
            assert page.locator('.race-readout').evaluate('e=>e.scrollWidth<=e.clientWidth')
        assert not errors
        browser.close()
