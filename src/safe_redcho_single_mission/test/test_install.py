import hashlib,json,subprocess,sys,tempfile,unittest
from pathlib import Path
import yaml

BUNDLE=Path(__file__).resolve().parents[3]
@unittest.skipUnless((BUNDLE/'install.py').is_file(),'Installer tests require the distribution bundle')
class InstallationTests(unittest.TestCase):
 def workspace(self,root):
  p=Path(root);(p/'scripts').mkdir();(p/'config').mkdir();(p/'src/safe_redcho_three').mkdir(parents=True)
  (p/'scripts/environment.bash').write_text('# unchanged environment\n');(p/'scripts/redcho_three.sh').write_text('# unchanged launcher\n');(p/'config/lab_config.yaml').write_text('network: {}\n');(p/'src/safe_redcho_three/old.py').write_text('original=True\n');return p
 def test_install_freezes_legacy_and_refuses_overwrite(self):
  with tempfile.TemporaryDirectory() as temp:
   ws=self.workspace(temp);before={str(p.relative_to(ws)):p.read_bytes() for p in ws.rglob('*') if p.is_file()}
   out=subprocess.run([sys.executable,str(BUNDLE/'install.py'),'--workspace',str(ws)],capture_output=True,text=True);self.assertEqual(out.returncode,0,out.stderr)
   r=json.loads(out.stdout);self.assertTrue((ws/'src/safe_redcho_single_mission').is_dir());checkpoint=Path(r['checkpoint']);lock=json.loads((checkpoint/'LOCK_MANIFEST.json').read_text())
   for rel,data in before.items():self.assertEqual((ws/rel).read_bytes(),data);self.assertEqual((checkpoint/rel).read_bytes(),data);self.assertEqual(lock['files_sha256'][rel],hashlib.sha256(data).hexdigest())
   again=subprocess.run([sys.executable,str(BUNDLE/'install.py'),'--workspace',str(ws)],capture_output=True,text=True);self.assertNotEqual(again.returncode,0);self.assertIn('already exists',again.stderr)
 def test_invalid_inherited_room_installs_nothing(self):
  with tempfile.TemporaryDirectory() as temp:
   ws=self.workspace(temp);c=yaml.safe_load((BUNDLE/'src/safe_redcho_single_mission/config/mission.yaml').read_text());c['arena']['ceiling_z_m']=.5;(ws/'config/redcho_three.yaml').write_text(yaml.safe_dump(c))
   out=subprocess.run([sys.executable,str(BUNDLE/'install.py'),'--workspace',str(ws)],capture_output=True,text=True);self.assertNotEqual(out.returncode,0);self.assertFalse((ws/'src/safe_redcho_single_mission').exists());self.assertFalse((ws/'results/checkpoints').exists())
if __name__=='__main__':unittest.main()
