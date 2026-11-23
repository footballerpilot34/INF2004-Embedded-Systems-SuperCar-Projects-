"""Test C planner against independent dense polygon-distance calculations.

These are geometric/synthetic checks, not measured encoder accuracy or floor
validation. Includes actual axle/sensor geometry and both tracking extremes.
"""
from pathlib import Path
import json
import math
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
SOURCE = r'''
#include <stdio.h>
#include <stdlib.h>
#include "bypass_geometry.h"
int main(int argc, char **argv) {
    if (argc!=4 && argc!=5) return 2;
    bypass_geometry_t p;
    bypass_geometry_init(&p,strtof(argv[1],0),strtof(argv[2],0),strtof(argv[3],0));
    p.offset=argc==5 ? strtof(argv[4],0) : BYPASS_COMPACT_OFFSET_MM;
    p.return_deg=BYPASS_COMPACT_RETURN_DEG;
    p.return_mm=p.offset/sinf(p.return_deg*BG_RAD);
    if (!bypass_geometry_candidate(&p,BYPASS_COMPACT_DEPARTURE_DEG)) {
        puts("null"); return 0;
    }
    printf("{\"a\":%g,\"b\":%g,\"offset\":%g,\"departure\":%g,"
           "\"departure_y\":%g,\"clear_y\":%g,\"return\":%g,"
           "\"path\":%g,\"wheel\":%g,\"margin\":%g}\n",
           p.departure,p.return_deg,p.offset,p.departure_mm,p.departure_y,
           p.clear_y,p.return_mm,p.translation_mm,p.wheel_mm,bypass_geometry_margin());
    return 0;
}
'''

def point_segment(p, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]
    u = max(0, min(1, ((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy)))
    return math.hypot(p[0]-a[0]-u*dx, p[1]-a[1]-u*dy)

def polygon_distance(a, b):
    # Independent SAT for intersection, then exact vertex-to-edge distances.
    for poly in (a, b):
        for p, q in zip(poly, poly[1:]+poly[:1]):
            axis = (p[1]-q[1], q[0]-p[0])
            ap = [x*axis[0]+y*axis[1] for x,y in a]
            bp = [x*axis[0]+y*axis[1] for x,y in b]
            if max(ap)<min(bp) or max(bp)<min(ap):
                break
        else:
            continue
        break
    else:
        return 0.0
    return min(point_segment(p,c,d)
               for vertices, edges in ((a,b),(b,a))
               for p in vertices for c,d in zip(edges,edges[1:]+edges[:1]))

def chassis(x, y, h):
    s, c = math.sin(math.radians(h)), math.cos(math.radians(h))
    # Axle-to-front 60, axle-to-rear 110. Subtract 60 from world y so
    # obstacle's front remains measured from the INITIAL front bumper.
    return [(x+side*c+fore*s, y-side*s+fore*c-60)
            for side,fore in ((-70,-110),(70,-110),(70,60),(-70,60))]

def dense_check(plan, front, left, right):
    obstacle = [(left,front),(right,front),(right,front+100),(left,front+100)]
    minimum = math.inf
    # Include independent tracking deviations, measured-heading uncertainty,
    # and a 10 mm displacement toward the obstacle in cardinal directions.
    for da, db in ((0,0),(-2,-2),(-2,2),(2,-2),(2,2)):
        a, b = plan['a']+da, plan['b']+db
        d, end = plan['offset'], plan['clear_y']
        y = d/math.tan(math.radians(a))
        final = end+d/math.tan(math.radians(b))
        poses = [(0,0,0),(0,0,-a),(-d,y,-a),(-d,y,0),
                 (-d,end,0),(-d,end,b),(0,final,b),(0,final,0)]
        for p,q in zip(poses,poses[1:]):
            n = math.ceil(max(math.hypot(q[0]-p[0],q[1]-p[1])/2,abs(q[2]-p[2]),1))
            for i in range(n+1):
                x,y,h = (p[k]+(q[k]-p[k])*i/n for k in range(3))
                for dh in (-5,0,5):
                    for dx,dy in ((0,0),(10,0),(-10,0),(0,10),(0,-10)):
                        clearance = polygon_distance(chassis(x+dx,y+dy,h+dh),obstacle)
                        minimum = min(minimum,clearance)
                        assert clearance>=20-1e-4, (plan,da,db,x,y,h,clearance)
    return minimum

with tempfile.TemporaryDirectory() as directory:
    work = Path(directory)
    (work/'geometry.c').write_text(SOURCE)
    subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror',
                    '-fsanitize=undefined','-fno-sanitize-recover=all',
                    '-I',str(ROOT/'src'),str(work/'geometry.c'),'-lm','-o',str(work/'geometry')],
                   check=True,timeout=30)
    def plan(front,left,right,offset=None):
        return json.loads(subprocess.check_output([str(work/'geometry'),str(front),str(left),str(right)]+([] if offset is None else [str(offset)]),text=True,timeout=10))
    nominal = plan(240,-100,100)
    assert nominal and nominal['offset']==219 and nominal['b']==65
    assert 35<=nominal['a']<58
    assert abs(nominal['path']-(nominal['departure']+nominal['clear_y']-nominal['departure_y']+nominal['return']))<0.02
    # Nominal side clearance 110 = target 100 plus uncertainty allowance 10.
    assert nominal['offset']-100-70==49
    assert nominal['clear_y']==240+100+170+20
    assert plan(20,-100,100) is None
    assert plan(240,-100,100,190) is None
    assert plan(240,-100,100,218) is None
    assert plan(240,-150,50) is None
    minimum=dense_check(nominal,240,-100,100)
    print(f'Compact geometry: {nominal}; dense minimum={minimum:.2f} mm')
    a,b=map(math.radians,(nominal['a'],nominal['b']))
    expected=nominal['clear_y']+nominal['offset']*math.tan(a/2)+nominal['offset']/math.sin(b)
    assert abs(expected-nominal['path'])<0.02
    print('Independent swept-footprint geometry checks passed')
