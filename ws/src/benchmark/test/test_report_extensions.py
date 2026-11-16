"""Generic composition supports additional modules without host-specific dispatch."""
from benchmark.analysis.core.results import Module
from benchmark.reports.render import render
from benchmark.analysis.modules.race_view import render_report


def test_generic_host_and_multiple_contributors(tmp_path, monkeypatch):
    from benchmark.analysis.core import results
    host = Module('host', '1', (), (), None, render_report, report_host='race-performance')
    def contribution(result):
        return {'host':'race-performance', 'viewer':{'id':result['name'], 'title':result['name'],
                'samples':[], 'signals':{}, 'overlays':[]}, 'fallback':{'viewer':{'laps':[]}}}
    modules = {'host':host, **{n:Module(n, '1', (), (), None, lambda r:'<p>'+r['name']+'</p>', report_extensions=contribution) for n in ('tires','future_sensor')}}
    monkeypatch.setattr(results, 'REGISTRY', modules)
    def result(n):
        return {'name':n,'status':'ok','viewer':{'laps':[]},'metrics':{},'figures':[]}
    for selected in (['tires','host','future_sensor'], ['tires','future_sensor']):
        render(tmp_path, {'modules':[result(n) for n in selected]})
        text=(tmp_path/'report.html').read_text()
        assert text.count('id="race-performance"') == 1
        assert text.index('id="race-performance"') < text.index('<p>tires</p>')
        assert '"id": "tires"' in text and '"id": "future_sensor"' in text
