#!/usr/bin/env python3
"""
su2_inspect.py - locate the nodes named in SU2's
    "Can't find the edge that connects A and B"
error, and show what is wrong around them.

usage:   python su2_inspect.py  mesh.su2  A  B
         (A and B are the two node ids from the SU2 message; SU2 ids are 0-based,
          i.e. the first point listed in NPOIN is 0)

It reports, for A and B:
  * coordinates
  * which boundary markers use them
  * how many volume elements use them (0 = orphan point)
  * whether ANY volume element has an edge A-B  (SU2 says no)
  * boundary elements that contain both A and B
  * other points at (almost) the same location  (duplicate nodes)
"""
import sys, itertools
from collections import defaultdict

# nodes per SU2 element type, and local edges of the 3D volume types
NNODE = {3: 2, 5: 3, 9: 4, 10: 4, 12: 8, 13: 6, 14: 5}
EDGES = {
    10: [(0,1),(1,2),(2,0),(0,3),(1,3),(2,3)],                                   # tet
    12: [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)],  # hex
    13: [(0,1),(1,2),(2,0),(3,4),(4,5),(5,3),(0,3),(1,4),(2,5)],                 # prism
    14: [(0,1),(1,2),(2,3),(3,0),(0,4),(1,4),(2,4),(3,4)],                       # pyramid
}
TYPENAME = {3:"line",5:"tri",9:"quad",10:"tet",12:"hex",13:"prism",14:"pyramid"}

def read_su2(path):
    ndime, elems, pts, markers = 3, [], [], []
    with open(path) as f:
        lines = iter(f.readlines())
    for line in lines:
        s = line.split("%")[0].strip()
        if not s: continue
        key, _, val = s.partition("=")
        key = key.strip().upper(); val = val.strip()
        if key == "NDIME":
            ndime = int(val)
        elif key == "NELEM":
            n = int(val.split()[0])
            for _ in range(n):
                t = next(lines).split()
                et = int(t[0]); k = NNODE[et]
                elems.append((et, tuple(int(x) for x in t[1:1+k])))
        elif key == "NPOIN":
            n = int(val.split()[0])
            for _ in range(n):
                t = next(lines).split()
                pts.append(tuple(float(x) for x in t[:ndime]))
        elif key == "NMARK":
            for _ in range(int(val)):
                tag = None; ne = 0
                while True:                      # MARKER_TAG= / MARKER_ELEMS=
                    s2 = next(lines).split("%")[0].strip()
                    if not s2: continue
                    k2, _, v2 = s2.partition("=")
                    k2 = k2.strip().upper(); v2 = v2.strip()
                    if k2 == "MARKER_TAG": tag = v2
                    elif k2 == "MARKER_ELEMS":
                        ne = int(v2); break
                els = []
                for _ in range(ne):
                    t = next(lines).split()
                    et = int(t[0]); k = NNODE[et]
                    els.append((et, tuple(int(x) for x in t[1:1+k])))
                markers.append((tag, els))
    return pts, elems, markers

def main():
    if len(sys.argv) != 4:
        print(__doc__); sys.exit(1)
    path, A, B = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
    pts, elems, markers = read_su2(path)
    print(f"{len(pts)} points, {len(elems)} volume elements, markers: "
          + ", ".join(f"{t}({len(e)})" for t, e in markers))

    # volume edges and element counts
    vol_edges = set()
    use = defaultdict(int)
    for et, nd in elems:
        for a in nd: use[a] += 1
        for i, j in EDGES.get(et, []):
            vol_edges.add((min(nd[i], nd[j]), max(nd[i], nd[j])))

    for p in (A, B):
        print(f"\nnode {p}: xyz = {pts[p]}   used by {use[p]} volume elements")
        for tag, els in markers:
            n = sum(1 for et, nd in els if p in nd)
            if n: print(f"   marker {tag}: {n} boundary elements")

    d = sum((a-b)**2 for a, b in zip(pts[A], pts[B]))**0.5
    print(f"\ndistance between the two nodes: {d:.3e}")
    print("volume element with edge A-B  :", (min(A, B), max(A, B)) in vol_edges)
    for tag, els in markers:
        for et, nd in els:
            if A in nd and B in nd:
                print(f"boundary element containing both: marker {tag}, type {TYPENAME[et]}, nodes {nd}")

    # duplicate points (same location, different id)
    try:
        import numpy as np
        from scipy.spatial import cKDTree
        arr = np.array(pts); tree = cKDTree(arr)
        for p in (A, B):
            near = [q for q in tree.query_ball_point(arr[p], 1e-9) if q != p]
            print(f"points coincident with node {p}: {near}")
    except ImportError:
        print("(install numpy+scipy to also check for duplicate nodes)")

if __name__ == "__main__":
    main()
