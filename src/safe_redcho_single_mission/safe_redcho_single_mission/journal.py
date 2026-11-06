"""Bounded nonblocking command journal with original publication timestamps."""
import csv,queue,threading,time

HEADER=['wall_monotonic_s','ros_ns','stage','name','published_x_m','published_y_m','published_z_m','published_yaw_rad','computed_vx_mps','computed_vy_mps','computed_vz_mps','computed_ax_mps2','computed_ay_mps2','computed_az_mps2','interface','received_onboard_verified']
class CommandJournal:
 def __init__(self,path):
  self.path=path;self.queue=queue.Queue(maxsize=600);self.errors=[];self.dropped=0;self.rows=0;self.stop=threading.Event();self.closed=False
  self.ready=threading.Event();self.worker=threading.Thread(target=self.run,daemon=True,name='mission_command_journal');self.worker.start()
  # Before takeoff, ensure the destination could be opened. No per-cycle wait.
  if not self.ready.wait(2.) or self.errors:raise RuntimeError('Command journal unavailable: '+str(self.errors))
 def run(self):
  try:
   with self.path.open('w',newline='',buffering=1) as f:
    writer=csv.writer(f);writer.writerow(HEADER);self.ready.set()
    while not self.stop.is_set() or not self.queue.empty():
     try:batch=self.queue.get(timeout=.05)
     except queue.Empty:continue
     writer.writerows(batch);self.rows+=len(batch)
  except Exception as e:self.errors.append(f'{type(e).__name__}: {e}');self.ready.set()
 def submit(self,rows):
  if self.closed or self.errors:raise RuntimeError('Command journal failed: '+str(self.errors))
  try:self.queue.put_nowait(rows)
  except queue.Full:self.dropped+=len(rows);raise RuntimeError('Command journal queue full; published rows could not be recorded')
 def close(self):
  self.closed=True;self.stop.set();self.worker.join(timeout=3.)
  return dict(rows=self.rows,dropped_rows=self.dropped,errors=self.errors,pending_batches=self.queue.qsize(),worker_still_running=self.worker.is_alive())
