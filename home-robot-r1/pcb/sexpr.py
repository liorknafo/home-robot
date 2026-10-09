"""Minimal KiCad S-expression reader/writer. Quoted strings come back as Q (a str subclass) so dump() re-quotes them."""
import re
class Q(str):
    pass
_TOKEN = re.compile(r'\(|\)|"(?:\\.|[^"\\])*"|[^\s()]+')
def parse(text):
    stack = [[]]
    for t in _TOKEN.findall(text):
        if t == '(':
            stack.append([])
        elif t == ')':
            node = stack.pop(); stack[-1].append(node)
        elif t.startswith('"'):
            stack[-1].append(Q(t[1:-1].replace('\\"', '"').replace('\\\\', '\\')))
        else:
            stack[-1].append(t)
    return stack[0]
def dump(x):
    if isinstance(x, list):
        return '(' + ' '.join(dump(i) for i in x) + ')'
    if isinstance(x, Q):
        return '"' + str(x).replace('\\', '\\\\').replace('"', '\\"') + '"'
    return str(x)
def find(node, key):
    return [c for c in node if isinstance(c, list) and c and c[0] == key]
