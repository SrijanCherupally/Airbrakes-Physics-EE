"""Airbrakes V3 load/current/RPM experiment. Python 3.10+, stdlib analysis."""
import argparse
import csv
from datetime import datetime, timezone
import html
import json
import math
from pathlib import Path
import random
import statistics as stats
import time

FIELDS = ['time_ms', 'phase', 'motor_pos', 'motor_vel', 'motor_current', 'battery_voltage', 'sequence']
ROOT = Path(__file__).resolve().parents[1]

def save_json(path, value):
    path = Path(path)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False), encoding='utf-8')
    temporary.replace(path)

def validate(config):
    for key in ['start_rev', 'end_rev', 'speed_rev_s', 'current_limit_a', 'timeout_s', 'trim_fraction']:
        if not math.isfinite(config[key]):
            raise ValueError(f'{key} must be finite')
    if not (-42 <= config['start_rev'] <= 0 and -42 <= config['end_rev'] <= 0):
        raise ValueError('Endpoints must be within the inherited -42 to 0 rev range; verify physical calibration')
    if abs(config['start_rev']-config['end_rev']) < 1:
        raise ValueError('Stroke must span at least 1 revolution')
    if not 0 < config['speed_rev_s'] <= 50 or not 0 < config['current_limit_a'] <= 3:
        raise ValueError('Speed must be >0 and <=50 rev/s; current limit >0 and <=3 A')
    if not 1 <= config['timeout_s'] <= 120 or not 0 <= config['trim_fraction'] < 0.5:
        raise ValueError('Timeout must be 1–120 seconds; trim fraction 0–<0.5')
    if not config['loads_kg'] or any(not math.isfinite(x) or x < 0 for x in config['loads_kg']):
        raise ValueError('Load masses must be finite and nonnegative')
    if len(set(config['loads_kg'])) != len(config['loads_kg']):
        raise ValueError('Load levels must be unique')
    if not 1 <= config['repeats'] <= 100:
        raise ValueError('Repetitions must be 1–100')

def create_session(config, simulated=False, root=None):
    validate(config)
    root = Path(root or ROOT / 'ee_results')
    root.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    folder = root / (stamp + ('_simulated' if simulated else '_physical'))
    folder.mkdir()
    trials = []
    rng = random.Random(config.get('seed', 42))
    # Randomized blocks reduce drift while guaranteeing each load occurs per repeat.
    for repeat in range(1, config['repeats']+1):
        loads = list(config['loads_kg']); rng.shuffle(loads)
        for load in loads:
            trials.append({'id': f'trial_{len(trials)+1:03}', 'load_kg': load,
                           'repeat': repeat, 'status': 'PENDING'})
    session = {'schema_version': 1, 'created_utc': datetime.now(timezone.utc).isoformat(),
               'source': 'SIMULATED' if simulated else 'PHYSICAL', 'config': config, 'trials': trials}
    save_json(folder / 'session.json', session)
    return folder, session

def write_rows(path, rows):
    with Path(path).open('w', newline='', encoding='utf-8') as file:
        writer = csv.DictWriter(file, fieldnames=FIELDS)
        writer.writeheader(); writer.writerows(rows)

def simulate(config, mass, seed):
    """Illustrative trends only, not an identified motor/linkage model."""
    rng = random.Random(seed)
    rows = []; stamp = 0; sequence = 0
    start, end = config['start_rev'], config['end_rev']
    # Fixed controller settings: illustrate speed droop and current growth with load.
    speed = config['speed_rev_s'] / (1+0.25*mass)
    offset = rng.gauss(0, 0.018)
    for phase, a, b in [('outbound', start, end), ('return', end, start)]:
        n = max(20, math.ceil(abs(b-a) / speed / 0.02))
        for i in range(n+1):
            fraction = i/n; direction = 1 if b>a else -1
            current = 0.25+0.8*mass+offset+rng.gauss(0, 0.02)
            if phase == 'return': current *= 0.85
            rows.append(dict(zip(FIELDS, [stamp, phase, a+(b-a)*fraction,
                direction*speed*(0 if i in (0,n) else 1), direction*current,
                12.0-0.1*mass, sequence])))
            stamp += 20; sequence += 1
    return rows

def capture(config, port, path, stop=lambda: False, progress=lambda row: None):
    import serial
    rows = []
    # Stream each sample to disk so interrupted trials retain their raw data.
    with Path(path).open('w', newline='', encoding='utf-8') as file:
        writer = csv.DictWriter(file, fieldnames=FIELDS); writer.writeheader(); file.flush()
        with serial.Serial(port, 115200, timeout=0.1, write_timeout=1) as board:
            time.sleep(2.2)
            board.reset_input_buffer()
            board.write(b'EE HELLO\n')
            deadline=time.monotonic()+5
            while time.monotonic() < deadline:
                if board.readline().decode(errors='replace').strip() == 'EE:HELLO:1': break
            else: raise RuntimeError('PhysicsEE firmware handshake failed; flash the PhysicsEE environment')
            if stop(): return 'ABORTED', rows
            command = 'EE RUN {start_rev} {end_rev} {speed_rev_s} {current_limit_a} {timeout}\n'.format(
                **config, timeout=int(config['timeout_s']*1000))
            board.write(command.encode())
            deadline=time.monotonic()+config['timeout_s']*2+8
            next_ping=0; started=False
            try:
                while time.monotonic() < deadline:
                    if stop():
                        board.write(b'EE STOP\n'); return 'ABORTED', rows
                    if time.monotonic() >= next_ping:
                        board.write(b'EE PING\n'); next_ping=time.monotonic()+0.5
                    line=board.readline().decode(errors='replace').strip()
                    if line == 'EE:STARTED': started=True
                    elif line.startswith('EE:ERROR:'): raise RuntimeError(line)
                    elif line.startswith('EE:DONE:'):
                        status=line.split(':',2)[2]
                        if status == 'COMPLETE' and (not started or not rows):
                            raise RuntimeError('Complete response without a started trial and samples')
                        return status, rows
                    elif line.startswith('EE:DATA:'):
                        parts=line[8:].split(',')
                        if len(parts) != len(FIELDS): raise RuntimeError('Malformed telemetry')
                        row=dict(zip(FIELDS,parts))
                        for key in FIELDS:
                            if key != 'phase' and not math.isfinite(float(row[key])): raise RuntimeError('Nonfinite telemetry')
                        if row['phase'] not in ('outbound','return'): raise RuntimeError('Unknown phase')
                        writer.writerow(row); file.flush(); rows.append(row); progress(row)
                raise RuntimeError('Trial timed out without a completion response')
            finally:
                try: board.write(b'EE STOP\n')
                except (OSError, serial.SerialException): pass

def summarize(rows, config):
    numeric=[]
    for row in rows:
        value={k: (v if k=='phase' else float(v)) for k,v in row.items()}
        if any(not math.isfinite(v) for k,v in value.items() if k!='phase'):
            raise ValueError('Nonfinite data')
        numeric.append(value)
    for a,b in zip(numeric,numeric[1:]):
        if b['time_ms'] <= a['time_ms'] or b['sequence'] <= a['sequence']:
            raise ValueError('Timestamps and fresh-sample sequences must increase')
    results=[]; start=config['start_rev']; end=config['end_rev']; trim=config['trim_fraction']
    for phase in ('outbound','return'):
        selected=[r for r in numeric if r['phase']==phase and
                  trim <= (r['motor_pos']-start)/(end-start) <= 1-trim and abs(r['motor_vel']) > 0.05]
        if len(selected)<5: raise ValueError(f'Insufficient moving samples in {phase} analysis window')
        gaps=[b['time_ms']-a['time_ms'] for a,b in zip(selected,selected[1:])]
        current=[abs(r['motor_current']) for r in selected]
        rpm=[abs(r['motor_vel'])*60 for r in selected]
        results.append({'phase':phase,'samples':len(selected),'mean_abs_current_a':stats.mean(current),
            'sd_abs_current_a':stats.stdev(current),'mean_abs_rpm':stats.mean(rpm),'sd_abs_rpm':stats.stdev(rpm),
            'peak_abs_current_a':max(abs(r['motor_current']) for r in numeric if r['phase']==phase),
            'mean_battery_v':stats.mean(r['battery_voltage'] for r in selected),
            'median_sample_interval_ms':stats.median(gaps),'max_gap_ms':max(gaps),
            'speed_cv':stats.stdev(rpm)/stats.mean(rpm),
            'warnings':(['Sampling gap >100 ms'] if max(gaps)>100 else [])+
                       (['Speed varies >10%; inspect trace before calling this steady motion'] if stats.stdev(rpm)/stats.mean(rpm)>0.1 else [])})
    return results

def linear_fit(points):
    if len(points)<3 or len(set(x for x,y in points))<2: return None
    xs,ys=zip(*points); xbar=stats.mean(xs); ybar=stats.mean(ys)
    sxx=sum((x-xbar)**2 for x in xs)
    slope=sum((x-xbar)*(y-ybar) for x,y in points)/sxx; intercept=ybar-slope*xbar
    residual=sum((y-(intercept+slope*x))**2 for x,y in points)
    total=sum((y-ybar)**2 for y in ys)
    return {'slope':slope,'intercept':intercept,'slope_standard_error':math.sqrt(residual/(len(points)-2)/sxx),
            'r_squared':1-residual/total if total else None,'n_independent_trials':len(points)}

def plot_svg(points, ylabel):
    if not points: return '<p>No eligible trials.</p>'
    xmax=max(x for x,y in points) or 1; ymax=max(y for x,y in points)*1.1 or 1
    circles=''.join(f'<circle cx="{65+x/xmax*560:.2f}" cy="{270-y/ymax*210:.2f}" r="5" fill="#247a9c"><title>{x:g} kg: {y:.4g}</title></circle>' for x,y in points)
    ticks=''.join(f'<text x="{65+i*112}" y="290">{xmax*i/5:.3g}</text><text x="5" y="{274-i*42}">{ymax*i/5:.3g}</text>' for i in range(6))
    return f'<svg viewBox="0 0 680 335" role="img" aria-label="{ylabel}"><path d="M65 45 V270 H640" fill="none" stroke="#333"/>{ticks}{circles}<text x="250" y="320">Added mass (kg)</text><text x="65" y="25">{ylabel}</text></svg>'

def trace_svg(rows, metric, scale, label):
    points=[(float(r['time_ms'])/1000,float(r[metric])*scale) for r in rows]
    if not points: return ''
    xmax=max(x for x,y in points) or 1
    low=min(0,min(y for x,y in points)); high=max(0,max(y for x,y in points))
    if low==high: high=low+1
    coordinates=' '.join(f'{65+x/xmax*560:.2f},{270-(y-low)/(high-low)*210:.2f}' for x,y in points)
    ticks=''.join(f'<text x="{65+i*112}" y="290">{xmax*i/5:.3g}</text><text x="3" y="{274-i*42}">{low+(high-low)*i/5:.3g}</text>' for i in range(6))
    return f'<svg viewBox="0 0 680 335" role="img" aria-label="{label}"><text x="65" y="25">{label}</text><path d="M65 45 V270 H640" fill="none" stroke="#333"/>{ticks}<polyline points="{coordinates}" fill="none" stroke="#247a9c" stroke-width="1.5"/><text x="280" y="320">Time (s)</text></svg>'

def analyze(folder):
    folder=Path(folder); session=json.loads((folder/'session.json').read_text()); summaries=[]; excluded=[]; traces=[]
    for trial in session['trials']:
        if trial['status']!='COMPLETE':
            excluded.append({'id':trial['id'],'reason':trial['status']}); continue
        try:
            with (folder/trial['id']/'data.csv').open(newline='',encoding='utf-8') as file:
                rows=list(csv.DictReader(file))
            for summary in summarize(rows,session['config']):
                summaries.append({**summary,'trial_id':trial['id'],'load_kg':trial['load_kg'],'repeat':trial['repeat']})
            traces.append('<details><summary>'+html.escape(f"{trial['id']} • {trial['load_kg']} kg • repeat {trial['repeat']}")+'</summary>'+trace_svg(rows,'motor_current',1,'Signed Iq (A)')+trace_svg(rows,'motor_vel',60,'Signed motor RPM')+'</details>')
        except (ValueError,OSError,KeyError) as error:
            excluded.append({'id':trial['id'],'reason':str(error)})
    grouped=[]; fits={}; charts=[]
    for phase in ('outbound','return'):
        for metric in ('mean_abs_current_a','mean_abs_rpm'):
            subset=[s for s in summaries if s['phase']==phase]
            points=[(s['load_kg'],s[metric]) for s in subset]
            fits[f'{phase}_{metric}']=linear_fit(points)
            charts.append(f'<h2>{phase}: {metric}</h2>'+plot_svg(points,metric))
            for load in sorted(set(s['load_kg'] for s in subset)):
                values=[s[metric] for s in subset if s['load_kg']==load]
                sd=stats.stdev(values) if len(values)>1 else None
                grouped.append({'phase':phase,'metric':metric,'load_kg':load,'weight_n':load*9.80665,
                    'n_trials':len(values),'mean':stats.mean(values),'between_trial_sd':sd,
                    'standard_error':sd/math.sqrt(len(values)) if sd is not None else None})
    result={'source':session['source'],'trial_summaries':summaries,'groups':grouped,'fits':fits,'excluded':excluded}
    save_json(folder/'analysis.json',result)
    if grouped:
        with (folder/'load_group_summary.csv').open('w',newline='',encoding='utf-8') as file:
            writer=csv.DictWriter(file,fieldnames=list(grouped[0])); writer.writeheader(); writer.writerows(grouped)
    if summaries:
        with (folder/'trial_summary.csv').open('w',newline='',encoding='utf-8') as file:
            writer=csv.DictWriter(file,fieldnames=list(summaries[0])); writer.writeheader(); writer.writerows(summaries)
    body='<h1>Airbrakes Physics EE: '+html.escape(session['source'])+'</h1>'
    body+='<p>Added load mass vs motor |Iq| and |RPM|. Each plotted point is one independent trial. Outbound and return are analyzed separately. RPM = 60 × rev/s.</p>'
    body+='<p>Analysis uses the central travel window; this does not guarantee constant speed. Weight = mg is the applied weight, not the axial screw force. Iq is torque-producing motor current, not supply current. Simulated data cannot establish experimental conclusions.</p>'
    body+=''.join(charts)+'<h2>Raw traces: inspect acceleration, return motion and current peaks</h2>'+''.join(traces)
    body+='<h2>Grouped results, regression and exclusions</h2><pre>'+html.escape(json.dumps({'groups':grouped,'fits':fits,'excluded':excluded},indent=2))+'</pre>'
    (folder/'report.html').write_text('<!doctype html><meta charset="utf-8"><title>EE results</title><style>body{font:16px system-ui;max-width:1100px;margin:40px auto;padding:20px;background:#f4f7fb;color:#15283b}svg{width:100%;max-width:680px;background:white}pre{white-space:pre-wrap}h2{margin-top:35px}</style>'+body,encoding='utf-8')
    return result

def default_config():
    return {'loads_kg':[0,0.1,0.2,0.3,0.4], 'repeats':5,'start_rev':0.0,'end_rev':-10.0,
            'speed_rev_s':2.0,'current_limit_a':1.0,'timeout_s':30.0,'trim_fraction':0.2,
            'seed':42,'load_attachment':'Specify flap(s), attachment point, load direction and total added mass',
            'mass_uncertainty_kg':0.001,'notes':'Illustrative defaults; verify stroke calibration and load fixture before physical use'}

def run_trial(folder, session, trial, port=None, stop=lambda:False, progress=lambda row:None):
    target=Path(folder)/trial['id']; target.mkdir(exist_ok=False)
    trial['status']='RUNNING'; save_json(Path(folder)/'session.json',session)
    save_json(target/'metadata.json',{'source':session['source'],'trial':trial.copy(),'config':session['config']})
    try:
        if session['source']=='SIMULATED':
            rows=simulate(session['config'],trial['load_kg'],session['config']['seed']+int(trial['id'].split('_')[1]))
            write_rows(target/'data.csv',rows); status='COMPLETE'
        else:
            status,rows=capture(session['config'],port,target/'data.csv',stop,progress)
        trial['status']=status; trial['sample_count']=len(rows)
    except Exception as error:
        trial['status']='ERROR'; trial['error']=str(error)
        raise
    finally:
        save_json(target/'metadata.json',{'source':session['source'],'trial':trial.copy(),'config':session['config']})
        save_json(Path(folder)/'session.json',session)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='command',required=True)
    sub.add_parser('demo'); sub.add_parser('gui')
    p=sub.add_parser('analyze'); p.add_argument('folder')
    args=parser.parse_args()
    if args.command=='gui':
        from gui import launch
        launch(); return
    if args.command=='analyze': analyze(args.folder); print(Path(args.folder)/'report.html'); return
    folder,session=create_session(default_config(),True)
    for trial in session['trials']: run_trial(folder,session,trial)
    analyze(folder); print(folder/'report.html')

if __name__=='__main__': main()
