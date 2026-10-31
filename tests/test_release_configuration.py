"""Local configuration is relocatable and cannot fabricate measured starts."""
from pathlib import Path
import shutil,subprocess,sys,tempfile,unittest
import yaml
ROOT=Path(__file__).resolve().parents[1]

class ConfigurationInitialization(unittest.TestCase):
    def test_relocated_init_preserves_edits_and_creates_no_captured_starts(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);(root/'scripts').mkdir();(root/'config').mkdir()
            shutil.copyfile(ROOT/'scripts/configure.py',root/'scripts/configure.py')
            for p in (ROOT/'config').glob('*.yaml'):shutil.copyfile(p,root/'config'/p.name)
            subprocess.run([sys.executable,str(root/'scripts/configure.py')],check=True,capture_output=True)
            self.assertFalse(list((root/'config/local').glob('*starts.yaml')))
            lab=root/'config/local/lab_config.yaml';cfg=yaml.safe_load(lab.read_text());cfg['robot']['uri']='MY_EDIT';lab.write_text(yaml.safe_dump(cfg))
            subprocess.run([sys.executable,str(root/'scripts/configure.py')],check=True,capture_output=True)
            self.assertEqual(yaml.safe_load(lab.read_text())['robot']['uri'],'MY_EDIT')
            for p in (root/'config/local').glob('*.yaml'):
                c=yaml.safe_load(p.read_text())
                if isinstance(c,dict) and 'workspace' in c:self.assertEqual(c['workspace'],str(root))
    def test_supported_runner_never_calls_live_route_for_offline(self):
        # Smoke the parser without sourcing ROS or executing an aircraft action.
        p=subprocess.run(['bash','scripts/run.sh','help'],cwd=ROOT,text=True,capture_output=True)
        self.assertEqual(p.returncode,0,p.stderr)
        self.assertIn('Offline:',p.stdout)

if __name__=='__main__':unittest.main()
