"""LIO-170: the robot model page (design-review.html, served at /robot/design-review.html by the sim's autonomy service) works from a phone.

Pure: text only. Two facts, each checked where it lives:
  - the page has no link back to this Mac (127.0.0.1 / localhost): the LiDAR review link is relative, so it follows whatever address the
    page was opened at (../lidar-review.html from /robot/ is /lidar-review.html on the same host), in the template and in the committed page;
  - `cad/build_robot.py --step-only` writes only cad/home-robot-r1.step (git-ignored): every line that deletes or exports a tracked output
    before that branch is guarded by STEP_ONLY, and the branch itself saves nothing else.

LIO-173: a fresh copy from GitHub has no design-review.html (the published repo leaves it out, with the meshes it embeds). The template is
checked everywhere; the page only where it exists, and the test then skips with the command that makes it (pcb/tests/left_out.py).
The parts table links only a real address or a file of this repo; any other source text is shown as plain text, never as a dead link."""
import copy, json, re, subprocess, sys, tempfile, unittest
from pathlib import Path
from pcb.tests import left_out
ROOT = Path(__file__).resolve().parents[2]
LIDAR_LINK = '<a href="../lidar-review.html">open the LiDAR simulation review</a>'
PAGE = 'design-review.html'
def page_text(): return (ROOT / PAGE).read_text(encoding='utf8')


class ReviewPageLinks(unittest.TestCase):
    def skip_without_page(self):
        if left_out.missing(PAGE): self.skipTest(left_out.reason(PAGE))     # after the template checks, so those still ran

    def test_no_link_points_back_at_this_mac(self):
        for page in ('cad/review-template.html', PAGE):
            if page == PAGE: self.skip_without_page()
            text = (ROOT / page).read_text(encoding='utf8')
            self.assertEqual(re.findall(r'(?:127\.0\.0\.1|localhost|0\.0\.0\.0)[:/\w.-]*', text), [], page)

    def test_lidar_review_link_is_relative_in_template_and_page(self):
        for page in ('cad/review-template.html', PAGE):
            if page == PAGE: self.skip_without_page()
            text = (ROOT / page).read_text(encoding='utf8')
            self.assertEqual(text.count(LIDAR_LINK), 1, page)
            self.assertNotRegex(text, r'href="https?://[^"]*lidar-review\.html"', page)

    def test_step_download_link_is_the_file_the_build_writes(self):
        self.assertIn("ROOT/'cad'/'home-robot-r1.step'", (ROOT / 'cad/build_robot.py').read_text(encoding='utf8'))
        self.skip_without_page()
        self.assertIn('<a href="cad/home-robot-r1.step" download>', page_text())


class PartsTableLinks(unittest.TestCase):
    @classmethod
    def setUpClass(cls): cls.data = json.loads((ROOT / 'parts-and-connections.json').read_text(encoding='utf8'))

    def test_every_source_is_a_web_address_or_a_file_of_this_repo(self):
        # LIO-173: C18, C19 and C20 once held supplier names, which the page turned into links that 404 on every device.
        for p in self.data['parts']:
            s = p['source']
            self.assertTrue(s == '' or re.fullmatch(r'https?://\S+', s) or (ROOT / s).is_file(), f"{p['id']}: source is text, not an address: {s[:80]!r}")

    def test_builder_links_only_an_address_or_a_repo_file(self):
        """Run cad/rebuild_review.py on a small tree (a stub for the meshes, so it also runs in a copy that has none): a web address and a repo
        file become links, with the row's own link text; a supplier name or a file that is not there stays plain text with no href."""
        data = copy.deepcopy(self.data); parts = data['parts']
        cases = {parts[0]['id']: ('https://example.com/part', 'Maker part at Shop'), parts[1]['id']: ('pcb/fab/ORDERING.md', None),
                 parts[2]['id']: ('Maker 1234 (Digi-Key / Mouser)', None), parts[3]['id']: ('pcb/fab/not-there.md', None), parts[4]['id']: ('', None),
                 parts[5]['id']: ('/etc/hosts', None)}      # a file, but not inside this repo
        for i, (src, label) in zip(cases, cases.values()):
            p = next(q for q in parts if q['id'] == i); p['source'] = src; p.pop('source_label', None)
            if label: p['source_label'] = label
        with tempfile.TemporaryDirectory() as t:
            t = Path(t); (t / 'cad').mkdir(); (t / 'pcb/fab').mkdir(parents=True)
            for f in ('cad/rebuild_review.py', 'cad/review-template.html', 'cad/component-manifest.json', 'cad/validation.json', 'cad/parameters.json', 'cable-cut-list.json'):
                (t / f).write_bytes((ROOT / f).read_bytes())
            (t / 'parts-and-connections.json').write_text(json.dumps(data), encoding='utf8')
            (t / 'pcb/fab/ORDERING.md').write_text('x'); (t / 'cad/assembly-meshes.json').write_text('[]')
            for png in ('robot-assembled.png', 'robot-exploded.png', 'robot-internal.png'): (t / png).write_bytes(b'')       # newer than the stub meshes
            r = subprocess.run([sys.executable, str(t / 'cad/rebuild_review.py')], capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, r.stderr[-600:] + r.stdout[-300:])
            page = (t / PAGE).read_text(encoding='utf8')
        def row(i): return re.search(r'<td class="id">' + i + r'</td>.*?</details>', page, re.S).group(0)
        a, b, c, d, e, f = (row(i) for i in cases)
        def has(needle, text): self.assertTrue(needle in text, f'{needle!r} is not in the row')        # not assertIn: it would print the whole row
        has('<a href="https://example.com/part" target="_blank" rel="noopener">Maker part at Shop ↗</a>', a)
        has('<a href="pcb/fab/ORDERING.md" target="_blank" rel="noopener">Product / source ↗</a>', b)
        for text, plain in ((c, 'Maker 1234 (Digi-Key / Mouser)'), (d, 'pcb/fab/not-there.md'), (f, '/etc/hosts')):
            self.assertFalse('<a href' in text, f'{plain!r} became a link'); has('Source (no link): ' + plain, text)
        has('Specification / allowance', e)


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
