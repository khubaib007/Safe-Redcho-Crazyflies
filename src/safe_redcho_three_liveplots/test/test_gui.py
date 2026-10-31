import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
import unittest
from support import cfg,packet
from safe_redcho_three_liveplots.engine import Engine
from safe_redcho_three_liveplots.configuration import GROUPS,limits
try:
    from pyqtgraph.Qt import QtWidgets
    from safe_redcho_three_liveplots.gui import Window
    QT=True
except ImportError:
    QT=False


@unittest.skipUnless(QT,'Qt/pyqtgraph not installed; data tests still run')
class GUIRenderingTests(unittest.TestCase):
    def test_all_eight_windows_render_both_axes_and_missing_state(self):
        app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
        c=cfg();t=1_000_000_000_000;e=Engine(c,t)
        for n in c['names']:e.algorithm(packet(c,n),n,t,t)
        e.ages(t,t);status=e.banner(t,t)
        status.update(csv_tail_drops=0,csv_tail_errors=0,writer_drops=0)
        for group in GROUPS:
            w=Window(group,c)
            w.accept(dict(schema_version=1,group=group,epoch=e.epoch,origin_ns=t,batch=1,now_ns=t,
                          events=list(e.out[group]),status=status,limits=limits(c)))
            w.show();app.processEvents();w.draw();app.processEvents()
            w.axis.setCurrentText('z');w.draw();app.processEvents()
            self.assertFalse(w.grab().isNull(),group)
            w.last_received=None;w.draw()
            self.assertIn('MISSING',w.status.text());w.close()


if __name__=='__main__':unittest.main()
