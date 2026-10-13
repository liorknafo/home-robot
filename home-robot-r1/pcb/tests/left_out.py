"""Files a fresh copy from GitHub does not have (tools/sync.sh in the published repo, its .gitignore and ours), and what makes each.

A test that reads one of them skips, with a reason that names the file and the command that makes it. Only a missing file skips:
a file that is there and fails its check still fails."""
import unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
HOW = {
    'cad/assembly-meshes.json': 'it holds manufacturer geometry; run tools/fetch_reference_models.sh, then python cad/build_robot.py (about 33 min)',
    'design-review.html': 'it embeds those meshes; after cad/build_robot.py run python cad/rebuild_review.py',
    'pcb/tools/freerouting.jar': 'it is not redistributed; download Freerouting 2.4.1 from the Freerouting releases and save it there (README, Build, step 3)',
}


def missing(rel): return not (ROOT / rel).is_file()
def reason(rel): return f'{rel} is not in this copy: {HOW[rel]}'
def needs(rel):
    """Decorator for a test or test class that reads `rel` (a path under the R1 folder, one of HOW)."""
    return unittest.skipIf(missing(rel), reason(rel))
