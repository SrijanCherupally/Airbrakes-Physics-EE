import json
from pathlib import Path
import queue
import threading
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import webbrowser
from experiment import create_session, default_config, run_trial, analyze, ROOT

def launch():
    app=tk.Tk(); app.title('Airbrakes V3 • Physics EE bench'); app.geometry('1100x780')
    state={'folder':None,'session':None,'busy':False}; events=queue.Queue(); stop=threading.Event()
    toolbar=ttk.Frame(app,padding=12); toolbar.pack(fill='x')
    simulated=tk.BooleanVar(value=True); port=tk.StringVar(); status=tk.StringVar(value='Create a session, then run one trial at a time.')
    ttk.Checkbutton(toolbar,text='Simulated experiment',variable=simulated).pack(side='left')
    ttk.Label(toolbar,text='Serial port:').pack(side='left',padx=12)
    ports=ttk.Combobox(toolbar,textvariable=port,width=15); ports.pack(side='left')
    def refresh():
        try:
            from serial.tools.list_ports import comports
            ports['values']=[p.device for p in comports()]
        except ImportError: messagebox.showinfo('Serial dependency','Run: python -m pip install -r ee/requirements.txt')
    ttk.Button(toolbar,text='Refresh ports',command=refresh).pack(side='left',padx=5)
    pane=ttk.Panedwindow(app,orient='horizontal'); pane.pack(fill='both',expand=True,padx=12)
    left=ttk.Frame(pane); right=ttk.Frame(pane); pane.add(left,weight=1); pane.add(right,weight=2)
    ttk.Label(left,text='Experiment settings (JSON; save with each session)').pack(anchor='w')
    settings=tk.Text(left,width=45,height=29,font=('Consolas',10)); settings.pack(fill='both',expand=True)
    settings.insert('1.0',json.dumps(default_config(),indent=2))
    ttk.Label(left,text='Physical testing uses the PhysicsEE firmware build.\nConfirm actual endpoint calibration and support weights\nwhen motor power is removed. Start position is checked.\nChange weights only between trials.',wraplength=350).pack(pady=10,anchor='w')
    table=ttk.Treeview(right,columns=('mass','repeat','status'),show='tree headings')
    table.heading('#0',text='Trial'); table.heading('mass',text='Added mass (kg)')
    table.heading('repeat',text='Repeat'); table.heading('status',text='Status')
    table.column('#0',width=110); table.column('mass',width=110); table.column('repeat',width=65); table.column('status',width=150)
    table.pack(fill='both',expand=True)
    live=tk.StringVar(value='Current: — A   Speed: — RPM'); ttk.Label(right,textvariable=live,font=('Segoe UI',15)).pack(pady=12)
    ttk.Label(app,textvariable=status,wraplength=1040).pack(fill='x',padx=12,pady=8)
    buttons=ttk.Frame(app,padding=12); buttons.pack(fill='x')
    def update_table():
        table.delete(*table.get_children())
        for trial in state['session']['trials']:
            table.insert('', 'end', iid=trial['id'],text=trial['id'],values=(trial['load_kg'],trial['repeat'],trial['status']))
    def create():
        if state['busy']: return
        try:
            config=json.loads(settings.get('1.0','end'))
            folder,session=create_session(config,simulated.get())
            state.update(folder=folder,session=session); update_table(); status.set(str(folder))
        except Exception as error: messagebox.showerror('Invalid settings',str(error))
    def resume():
        if state['busy']: return
        folder=filedialog.askdirectory(initialdir=ROOT/'ee_results')
        if not folder: return
        try:
            session=json.loads((Path(folder)/'session.json').read_text())
            state.update(folder=Path(folder),session=session)
            settings.delete('1.0','end'); settings.insert('1.0',json.dumps(session['config'],indent=2))
            simulated.set(session['source']=='SIMULATED'); update_table()
            status.set('Session loaded. Any interrupted RUNNING trial remains excluded; pending trials can continue.')
        except Exception as error: messagebox.showerror('Load failed',str(error))
    def run():
        if state['busy'] or not state['session']: return
        session=state['session']; trial=next((t for t in session['trials'] if t['status']=='PENDING'),None)
        if not trial: status.set('No pending trials. Generate the report or create a new session.'); return
        physical=session['source']=='PHYSICAL'
        if physical and not port.get(): messagebox.showerror('Select port','Select the Pico serial port.'); return
        if physical and not messagebox.askokcancel('Prepare load',f"Fit {trial['load_kg']:g} kg TOTAL added mass using the recorded attachment geometry.\nVerify the mechanism is at {session['config']['start_rev']:g} rev and clear the stroke.\nStart {trial['id']} (repeat {trial['repeat']})?"): return
        state['busy']=True; stop.clear(); status.set(f"Running {trial['id']} at {trial['load_kg']} kg")
        selected_port=port.get()
        def worker():
            try:
                run_trial(state['folder'],session,trial,selected_port,stop.is_set,lambda row: events.put(('sample',row)))
                events.put(('done',f"{trial['id']}: {trial['status']}"))
            except Exception as error: events.put(('done',str(error)))
        threading.Thread(target=worker,daemon=True).start()
    def report():
        if state['busy'] or not state['session']: return
        try:
            analyze(state['folder']); webbrowser.open((state['folder']/'report.html').resolve().as_uri())
            status.set(f"Analysis saved in {state['folder']}")
        except Exception as error: messagebox.showerror('Analysis error',str(error))
    def poll():
        try:
            while True:
                kind,value=events.get_nowait()
                if kind=='sample': live.set(f"Iq: {float(value['motor_current']):.3f} A   Speed: {float(value['motor_vel'])*60:.1f} RPM   {value['phase']}")
                else: state['busy']=False; status.set(value); update_table()
        except queue.Empty: pass
        app.after(100,poll)
    def close():
        if state['busy']:
            stop.set(); status.set('Stopping trial; close again once it has stopped.'); return
        app.destroy()
    ttk.Button(buttons,text='New session',command=create).pack(side='left',padx=4)
    ttk.Button(buttons,text='Resume session',command=resume).pack(side='left',padx=4)
    ttk.Button(buttons,text='Run next trial',command=run).pack(side='left',padx=4)
    ttk.Button(buttons,text='STOP',command=stop.set).pack(side='left',padx=15)
    ttk.Button(buttons,text='Analyze / open report',command=report).pack(side='left',padx=4)
    app.protocol('WM_DELETE_WINDOW',close); poll(); app.mainloop()

if __name__=='__main__': launch()
