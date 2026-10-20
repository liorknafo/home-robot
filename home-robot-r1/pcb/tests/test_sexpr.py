"""pcb/sexpr.py (pure): KiCad's quoted-string escapes survive parse -> dump."""
import unittest

from pcb.sexpr import Q, dump, parse


class SexprTests(unittest.TestCase):
    def test_string_escapes_round_trip(self):
        r"""\n (a newline in a library description), \" and \\ come back as the text, and dump writes the same escapes."""
        text = r'(property "Description" "x\ny \"q\" a\\b")'
        node = parse(text)[0]
        self.assertEqual(node[2], 'x\ny "q" a\\b')
        self.assertIsInstance(node[2], Q)
        self.assertEqual(dump(node), text)


if __name__ == '__main__':
    unittest.main()
