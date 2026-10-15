import tempfile
import unittest
from unittest.mock import patch
from pathlib import Path
from experiment import default_config, create_session, simulate, summarize, linear_fit, run_trial, analyze
from experiment import capture

class FakeSerial:
    def __init__(self, lines): self.lines=iter(lines); self.writes=[]
    def __enter__(self): return self
    def __exit__(self,*args): pass
    def reset_input_buffer(self): pass
    def write(self,value): self.writes.append(value)
    def readline(self): return next(self.lines,b'')

class ExperimentTests(unittest.TestCase):
    def test_serial_success_is_saved_and_stop_sent(self):
        board=FakeSerial([b'EE:HELLO:1\n',b'EE:STARTED\n',
            b'EE:DATA:20,outbound,-1,-2,-0.5,12,4\n',b'EE:DONE:COMPLETE\n'])
        with tempfile.TemporaryDirectory() as folder, patch('serial.Serial',return_value=board), patch('experiment.time.sleep'):
            status,rows=capture(default_config(),'FAKE',Path(folder)/'data.csv')
            self.assertEqual(status,'COMPLETE'); self.assertEqual(len(rows),1)
            self.assertIn('-0.5',(Path(folder)/'data.csv').read_text())
            self.assertEqual(board.writes[-1],b'EE STOP\n')

    def test_stop_during_connection_never_sends_run(self):
        board=FakeSerial([b'EE:HELLO:1\n'])
        with tempfile.TemporaryDirectory() as folder, patch('serial.Serial',return_value=board), patch('experiment.time.sleep'):
            status,rows=capture(default_config(),'FAKE',Path(folder)/'data.csv',stop=lambda:True)
            self.assertEqual(status,'ABORTED')
            self.assertFalse(any(x.startswith(b'EE RUN') for x in board.writes))

    def test_serial_error_sends_stop(self):
        board=FakeSerial([b'EE:HELLO:1\n',b'EE:ERROR:NOT_READY\n'])
        with tempfile.TemporaryDirectory() as folder, patch('serial.Serial',return_value=board), patch('experiment.time.sleep'):
            with self.assertRaises(RuntimeError): capture(default_config(),'FAKE',Path(folder)/'data.csv')
            self.assertEqual(board.writes[-1],b'EE STOP\n')

    def test_rpm_units_and_direction(self):
        config=default_config(); rows=simulate(config,0,1)
        out,back=summarize(rows,config)
        self.assertAlmostEqual(out['mean_abs_rpm'],120)
        self.assertAlmostEqual(back['mean_abs_rpm'],120)
        self.assertLess(rows[1]['motor_vel'],0)
        self.assertEqual(out['phase'],'outbound')

    def test_fit_uses_trial_points(self):
        fit=linear_fit([(0,1),(1,3),(2,5)])
        self.assertAlmostEqual(fit['slope'],2); self.assertAlmostEqual(fit['intercept'],1)
        self.assertEqual(fit['n_independent_trials'],3)

    def test_stale_samples_rejected(self):
        config=default_config(); rows=simulate(config,0,1); rows[1]['sequence']=rows[0]['sequence']
        with self.assertRaises(ValueError): summarize(rows,config)

    def test_aborted_trial_excluded_and_repeats_aggregated(self):
        with tempfile.TemporaryDirectory() as root:
            config=default_config(); config.update(loads_kg=[0,0.2],repeats=2)
            folder,session=create_session(config,True,root)
            for trial in session['trials']: run_trial(folder,session,trial)
            result=analyze(folder)
            self.assertEqual(len(result['trial_summaries']),8)
            self.assertTrue(all(g['n_trials']==2 for g in result['groups']))
            self.assertEqual(result['fits']['outbound_mean_abs_current_a']['n_independent_trials'],4)
            self.assertTrue((folder/'report.html').exists())
            from experiment import save_json
            session['trials'][0]['status']='ABORTED'; save_json(folder/'session.json',session)
            self.assertEqual(len(analyze(folder)['trial_summaries']),6)

    def test_invalid_limits_and_missing_data(self):
        with tempfile.TemporaryDirectory() as root:
            config=default_config(); config['current_limit_a']=4
            with self.assertRaises(ValueError): create_session(config,False,root)
        with self.assertRaises(ValueError): summarize([],default_config())

if __name__=='__main__': unittest.main()
