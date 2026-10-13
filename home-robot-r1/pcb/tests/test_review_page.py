"""LIO-170: the robot model page (design-review.html, served at /robot/design-review.html by the sim's autonomy service) works from a phone.

Pure: text only. Two facts, each checked where it lives:
  - the page has no link back to this Mac (127.0.0.1 / localhost): the LiDAR review link is relative, so it follows whatever address the
    page was opened at (../lidar-review.html from /robot/ is /lidar-review.html on the same host), in the template and in the committed page;
  - `cad/build_robot.py --step-only` writes only cad/home-robot-r1.step (git-ignored): every line that deletes or exports a tracked output
    before that branch is guarded by STEP_ONLY, and the branch itself saves nothing else."""
import re, unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[2]
LIDAR_LINK = '<a href="../lidar-review.html">open the LiDAR simulation review</a>'


class ReviewPageLinks(unittest.TestCase):
    def test_no_link_points_back_at_this_mac(self):
        for page in ('cad/review-template.html', 'design-review.html'):
            text = (ROOT / page).read_text(encoding='utf8')
            self.assertEqual(re.findall(r'(?:127\.0\.0\.1|localhost|0\.0\.0\.0)[:/\w.-]*', text), [], page)

    def test_lidar_review_link_is_relative_in_template_and_page(self):
        for page in ('cad/review-template.html', 'design-review.html'):
            text = (ROOT / page).read_text(encoding='utf8')
            self.assertEqual(text.count(LIDAR_LINK), 1, page)
            self.assertNotRegex(text, r'href="https?://[^"]*lidar-review\.html"', page)

    def test_step_download_link_is_the_file_the_build_writes(self):
        page = (ROOT / 'design-review.html').read_text(encoding='utf8')
        self.assertIn('<a href="cad/home-robot-r1.step" download>', page)
        self.assertIn("ROOT/'cad'/'home-robot-r1.step'", (ROOT / 'cad/build_robot.py').read_text(encoding='utf8'))


class StepOnlyBuild(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lines = (ROOT / 'cad/build_robot.py').read_text(encoding='utf8').splitlines()
        cls.branch = next(i for i, l in enumerate(cls.lines) if l.startswith('if STEP_ONLY:') and 'assembly' not in l and i > 100)

    def test_flag_is_read_from_the_command_line(self):
        self.assertTrue(any(re.match(r"STEP_ONLY\s*=\s*'--step-only' in sys\.argv", l) for l in self.lines))

    def test_every_tracked_output_before_the_branch_is_guarded(self):
        # unlink / exporters.export / write_text / .save before the step-only branch: the line itself, or an enclosing guard
        # on the line above (an `if not STEP_ONLY:` header), must name STEP_ONLY.
        writes = [i for i, l in enumerate(self.lines[:self.branch])
                  if re.search(r'\.unlink\(|exporters\.export\(|\.write_text\(|\.write_bytes\(|\.save\(|json\.dump\(', l)]
        self.assertGreaterEqual(len(writes), 3, 'the print-part delete, the print-part export and the coupon export')
        for i in writes:
            guarded = any('STEP_ONLY' in self.lines[j] for j in range(max(0, i - 2), i + 1))
            self.assertTrue(guarded, f'build_robot.py line {i + 1} writes or deletes a tracked output without a STEP_ONLY guard: {self.lines[i][:100]}')

    def test_the_branch_saves_only_the_step_and_exits(self):
        block = []
        for l in self.lines[self.branch:self.branch + 6]:
            block.append(l)
            if 'sys.exit(0)' in l:
                break
        text = '\n'.join(block)
        self.assertIn('sys.exit(0)', text)
        self.assertEqual(re.findall(r"\.save\(str\(ROOT/'cad'/'([^']+)'\)\)", text), ['home-robot-r1.step'])
        self.assertNotRegex(text, r'write_text|exporters|json\.dump|trimesh|tessellate')


if __name__ == '__main__':
    unittest.main()
