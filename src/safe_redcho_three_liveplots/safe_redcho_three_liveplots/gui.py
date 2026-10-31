# Release preparation: Qt5/Qt6-compatible pen-style enum access.
"""Independent Qt plot windows; ROS spins on the GUI thread with a bounded budget."""
import argparse
from datetime import datetime
import json
import os
import signal
import time
from pathlib import Path
import numpy as np
os.environ.setdefault('PYQTGRAPH_QT_LIB','PyQt5')
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets, QtGui
from .configuration import GROUPS, default_config, load
from .buffer import Histories
from .specs import specifications

COLORS = {'cf2':'#38bdf8', 'cf3':'#fbbf24', 'cf4':'#f472b6', None:'#d5deec'}


def get_value(data, dotted):
    for part in dotted.split('.'): data = data[part]
    return float(data)


class Window(QtWidgets.QMainWindow):
    def __init__(self, group, cfg):
        super().__init__()
        self.group, self.cfg = group, cfg
        self.history = Histories(cfg['max_points_per_series'], cfg['history_s'])
        self.last_received, self.packet, self.paused = None, None, False
        self.setWindowTitle('REDCHO · '+group.replace('_',' ').upper())
        self.resize(1400, 900 if group in ('flight','timing','hocbf') else 780)
        container = QtWidgets.QWidget(); self.setCentralWidget(container)
        box = QtWidgets.QVBoxLayout(container)
        self.title = QtWidgets.QLabel(group.replace('_',' ').upper()+'  |  cf2 / cf3 / cf4')
        self.title.setStyleSheet('font-size:18px;font-weight:bold;color:#e2e8f0;padding:5px')
        box.addWidget(self.title)
        self.status = QtWidgets.QLabel('MISSING — waiting for live_capture')
        self.status.setWordWrap(True); box.addWidget(self.status)
        toolbar = QtWidgets.QHBoxLayout(); box.addLayout(toolbar)
        toolbar.addWidget(QtWidgets.QLabel('Axis:'))
        self.axis = QtWidgets.QComboBox(); self.axis.addItems(['x','y','z'])
        self.axis.setCurrentText('z' if group=='position_redcho' else cfg.get('default_axis','x'))
        toolbar.addWidget(self.axis)
        self.field = QtWidgets.QComboBox(); self.field.addItem('MISSING — optional onboard log')
        if group in ('flight','control') and cfg.get('onboard_blocks'):
            toolbar.addWidget(QtWidgets.QLabel('Onboard field:')); toolbar.addWidget(self.field)
        pause = QtWidgets.QPushButton('Pause drawing'); pause.clicked.connect(self.pause); toolbar.addWidget(pause)
        clear = QtWidgets.QPushButton('Clear view'); clear.clicked.connect(self.clear); toolbar.addWidget(clear)
        save = QtWidgets.QPushButton('Save PNG'); save.clicked.connect(self.save); toolbar.addWidget(save)
        toolbar.addStretch()
        self.graph = pg.GraphicsLayoutWidget(); box.addWidget(self.graph,1)
        self.foot = QtWidgets.QLabel('Last observed stage · missing ≠ zero · host publication ≠ onboard receipt · optional fields require existing onboard logs')
        self.foot.setWordWrap(True); box.addWidget(self.foot)
        self.setStyleSheet('QWidget{background:#111827;color:#cbd5e1} QPushButton,QComboBox{background:#253249;padding:5px}')
        pg.setConfigOptions(antialias=False, background='#111827', foreground='#cbd5e1')
        self.panels, self.curves, self.lines, self.markers = [], [], [], []
        specs = [s for s in specifications(group) if '{field}' not in s['title'] or cfg.get('onboard_blocks')]
        columns = 3 if len(specs)>=7 or group=='flight' else 2
        for i,spec in enumerate(specs):
            plot = self.graph.addPlot(row=i//columns, col=i%columns)
            plot.showGrid(x=True,y=True,alpha=.12)
            plot.addLegend(labelTextSize='7pt', colCount=3, offset=(5,5), horSpacing=8, verSpacing=0)
            plot.setLabel('left',spec['unit']); plot.setLabel('bottom','X (m)' if spec['xy'] else 'Time since capture start (s)')
            if spec['xy']: plot.setAspectLocked(True)
            handles = []
            for line in spec['lines']:
                pen = pg.mkPen(COLORS[line['agent']], width=1,
                    style=QtCore.Qt.PenStyle.DashLine if line['dash'] else QtCore.Qt.PenStyle.SolidLine)
                handle = plot.plot([],[],pen=pen,name=line['label'],connect='finite')
                if not spec['xy']: handle.setDownsampling(auto=True,method='peak'); handle.setClipToView(True)
                handles.append(handle)
            threshold_lines = []
            for th in spec['thresholds']:
                obj = pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen('#f87171',style=QtCore.Qt.PenStyle.DotLine),
                      label=th[0],labelOpts={'position':.98,'anchors':[(1,0),(1,1)]})
                obj.label.setFont(QtGui.QFont('Sans',7))
                plot.addItem(obj); obj.hide(); threshold_lines.append(obj)
            if spec['xy']:
                a = cfg['profile']['arena']; xmin,xmax=a['x_limits']; ymin,ymax=a['y_limits']
                plot.plot([xmin,xmax,xmax,xmin,xmin],[ymin,ymin,ymax,ymax,ymin],pen=pg.mkPen('#64748b',style=QtCore.Qt.PenStyle.DotLine))
            self.panels.append((plot,spec)); self.curves.append(handles); self.lines.append(threshold_lines)
            self.markers.append([])

    def pause(self): self.paused = not self.paused

    def clear(self):
        self.history.series.clear()

    def save(self):
        root = Path(self.cfg['output_root']).expanduser()/'snapshots'; root.mkdir(parents=True,exist_ok=True)
        p = root/(self.group+'_'+datetime.now().strftime('%Y%m%dT%H%M%S')+'.png')
        self.grab().save(str(p)); self.foot.setText('Saved '+str(p))

    def accept(self, packet):
        if packet.get('schema_version') != 1 or packet.get('group') != self.group: return
        self.history.accept(packet); self.packet = packet; self.last_received = time.monotonic()

    def draw(self):
        if self.last_received is None or time.monotonic()-self.last_received > 1.:
            self.status.setText('MISSING / STALE — no fresh capture stream; displayed history is old')
            self.status.setStyleSheet('color:#f87171'); return
        p = self.packet; s = p['status']; states = s['streams']
        wanted = ('algorithm',) if self.group in ('reference','velocity_redcho','position_redcho','control','hocbf') else (
                 ('battery','algorithm') if self.group=='battery' else ('mocap','onboard','command'))
        issues = [key+':'+value for key,value in states.items() if key.split('.')[-1] in wanted and value!='LIVE']
        selected_field=self.field.currentText()
        if self.group in ('flight','control') and not selected_field.startswith('MISSING'):
            for n in ('cf2','cf3','cf4'):
                key=n+'.firmware.'+selected_field
                if states.get(key,'MISSING')!='LIVE': issues.append(key+':'+states.get(key,'MISSING'))
        errors=[n+':'+error for n,error in s.get('algorithm_errors',{}).items() if error]
        issues.extend(errors)
        quality = ' | '.join(issues) if issues else 'Streams live'
        self.status.setText(s['stage']+'  |  '+quality+
            f"  |  rejected={s['rejected']} · skipped cycles={s['skipped_cycles']} · incomplete={s['incomplete_batches']}"
            f" · capture drops={s['capture_overflow']} · file drops={s['csv_tail_drops']} · writer drops={s['writer_drops']}"
            f" · viewer lost batches={self.history.lost_batches}"+('  |  DRAWING PAUSED' if self.paused else ''))
        loss=any(s[k] for k in ('capture_overflow','csv_tail_drops','writer_drops','rejected','skipped_cycles','csv_tail_errors'))
        self.status.setStyleSheet('color:#fbbf24' if issues or loss else 'color:#6ee7b7')
        if self.paused: return
        fields = sorted({k.split('.firmware.',1)[1] for k in self.history.series if '.firmware.' in k})
        existing = [self.field.itemText(i) for i in range(self.field.count())]
        for field in fields:
            if field not in existing: self.field.addItem(field)
        if fields and self.field.currentIndex()==0: self.field.setCurrentText(fields[0])
        axis = self.axis.currentText(); field=self.field.currentText()
        now=(p['now_ns']-self.history.origin)/1e9
        for i,(plot,spec) in enumerate(self.panels):
            plot.setTitle(spec['title'].format(axis=axis,field=field), size='10pt')
            for line,handle in zip(spec['lines'],self.curves[i]):
                key=line['key'].format(axis=axis,field=field)
                x,y=self.history.xy(line['xy'].format(axis=axis,field=field),key) if line['xy'] else self.history.data(key)
                handle.setData(x,y,connect='finite')
            for th,obj in zip(spec['thresholds'],self.lines[i]):
                try:
                    value=float(th[1]) if isinstance(th[1],(float,int)) else get_value(p['limits'],th[1])
                    obj.setValue(value*(th[2] if len(th)>2 else 1)); obj.show()
                except (KeyError,ValueError): obj.hide()
            if not spec['xy']:
                plot.setXRange(max(0.,now-self.cfg['history_s']),max(5.,now),padding=.01)
                for obj in self.markers[i]: plot.removeItem(obj)
                self.markers[i]=[]
                for t,label in list(self.history.stages)[-6:]:
                    if t<now-self.cfg['history_s']: continue
                    short = 'HANDOVER' if 'HANDOVER' in label else label.split(' ')[0][:12]
                    obj=pg.InfiniteLine(t,angle=90,pen=pg.mkPen('#334155'),
                        label=short if i==0 else None,labelOpts={'position':.9,'rotateAxis':(1,0)})
                    if i==0:obj.label.setFont(QtGui.QFont('Sans',7))
                    plot.addItem(obj); self.markers[i].append(obj)


def run(group):
    parser=argparse.ArgumentParser(description='Read-only '+group+' live plots')
    parser.add_argument('--config',default=None); parser.add_argument('--profile',default=None)
    args,ros_args=parser.parse_known_args()
    cfg=load(args.config or default_config(),args.profile)
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from std_msgs.msg import String
    app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    rclpy.init(args=ros_args,signal_handler_options=SignalHandlerOptions.NO)
    node=Node('redcho_plot_'+group); window=Window(group,cfg)
    topic='/safe_redcho/live/'+group
    def received(msg):
        try:
            if node.count_publishers(topic)!=1:
                window.last_received=None; return
            window.accept(json.loads(msg.data))
        except (ValueError,KeyError,TypeError): node.get_logger().warning('Invalid plot batch discarded')
    subscription=node.create_subscription(String,topic,received,
        QoSProfile(depth=8,reliability=ReliabilityPolicy.BEST_EFFORT))
    timer=QtCore.QTimer()
    def tick():
        deadline=time.monotonic()+.008
        while time.monotonic()<deadline and rclpy.ok(): rclpy.spin_once(node,timeout_sec=0.)
        window.draw()
    timer.timeout.connect(tick); timer.start(int(1000/cfg['refresh_hz']))
    previous={s:signal.signal(s,lambda *_:app.quit()) for s in (signal.SIGINT,signal.SIGTERM)}
    window.show()
    try: app.exec()
    finally:
        timer.stop(); node.destroy_node(); rclpy.try_shutdown()
        for s,handler in previous.items(): signal.signal(s,handler)


def flight(): run('flight')
def battery(): run('battery')
def timing(): run('timing')
def reference(): run('reference')
def velocity_redcho(): run('velocity_redcho')
def position_redcho(): run('position_redcho')
def control(): run('control')
def hocbf(): run('hocbf')
