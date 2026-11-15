"""Manual real-pointer hover profile using a saved metrics.json; no ROS required."""
import argparse
import json
from pathlib import Path
from statistics import median
from tempfile import TemporaryDirectory

from playwright.sync_api import sync_playwright
from benchmark.analysis.modules.race_view import render_report
from benchmark.reports.render import SHARED


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('metrics', type=Path)
    parser.add_argument('--html', type=Path, help='Profile an existing HTML baseline instead of rendering current code')
    args = parser.parse_args()
    summary = json.loads(args.metrics.read_text())
    result = next(m for m in summary['modules'] if m['name'] == 'race_performance')
    with TemporaryDirectory() as directory, sync_playwright() as p:
        report = Path(directory)/'report.html'
        report.write_text(args.html.read_text() if args.html else SHARED+render_report(result))
        browser = p.chromium.launch(args=['--no-sandbox'])
        page = browser.new_page(viewport={'width': 1366, 'height': 900})
        page.goto(report.as_uri())
        page.wait_for_selector('.race-panels .js-plotly-plot')
        page.wait_for_timeout(1800)
        page.evaluate('''() => {
          window.profile={moves:0,updates:0,redraws:0,intervals:[],last:null};
          benchmarkReport.subscribe(()=>{const t=performance.now();profile.updates++;
            if(profile.last!==null)profile.intervals.push(t-profile.last);profile.last=t});
          document.addEventListener('pointermove',()=>profile.moves++);
          document.querySelectorAll('.js-plotly-plot').forEach(p=>p.on('plotly_afterplot',()=>profile.redraws++));
        }''')
        for target in ['.race-track svg', '.race-panels .js-plotly-plot']:
            element = page.locator(target).first
            element.scroll_into_view_if_needed()
            box = element.bounding_box()
            page.evaluate('profile.moves=profile.updates=profile.redraws=0;profile.intervals=[];profile.last=null')
            for n in range(80):
                page.mouse.move(box['x']+box['width']*(.2+.6*n/79),
                                box['y']+min(box['height']*.3, 100))
                page.wait_for_timeout(8)
            page.wait_for_timeout(200)
            stats = page.evaluate('profile')
            intervals = sorted(stats.pop('intervals'))
            stats.pop('last')
            stats.update(target=target, median_interval_ms=median(intervals) if intervals else None,
                         p95_interval_ms=intervals[int(.95*(len(intervals)-1))] if intervals else None)
            print(json.dumps(stats))
        browser.close()


if __name__ == '__main__':
    main()
