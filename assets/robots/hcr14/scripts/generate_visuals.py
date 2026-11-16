#!/usr/bin/env python3
"""Author HCR-14-like visual shells from profiles; no vendor CAD is read.

These are decorative approximations in the existing link-local frames. URDF
continues to own joints, collision geometry and inertias. Python stdlib only.
"""
import hashlib
import math
from pathlib import Path
import xml.etree.ElementTree as ET

NS = 'http://www.collada.org/2005/11/COLLADASchema'
ET.register_namespace('', NS)
PALETTE = {
    'charcoal': (.075, .080, .077, 1),
    'orange': (.98, .265, .025, 1),
    'seal': (.018, .022, .025, 1),
    'steel': (.62, .66, .68, 1),
    'fastener': (.16, .18, .19, 1),
    'led': (.20, .95, .24, 1),
}
SEGMENTS = 64


def add(parent, tag, text=None, **attributes):
    child = ET.SubElement(parent, f'{{{NS}}}{tag}', attributes)
    child.text = text
    return child


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def unit(a):
    length = math.sqrt(sum(v*v for v in a))
    return tuple(v/length for v in a)


class Shell:
    def __init__(self):
        self.parts = []

    def lathe(self, center, axis, profile, material):
        """Revolve an axial/radial outline with smooth vertex normals."""
        axis = unit(axis)
        u = unit(cross((0, 1, 0) if abs(axis[2]) > .9 else (0, 0, 1), axis))
        v = cross(axis, u)
        points, normals, faces = [], [], []
        for row, (z, radius) in enumerate(profile):
            prev = profile[max(0, row-1)]
            following = profile[min(len(profile)-1, row+1)]
            dz, dr = following[0]-prev[0], following[1]-prev[1]
            normal_scale = math.hypot(dz, dr)
            for column in range(SEGMENTS):
                angle = 2*math.pi*column/SEGMENTS
                radial = tuple(math.cos(angle)*a + math.sin(angle)*b for a,b in zip(u,v))
                points.append(tuple(c+z*a+radius*r for c,a,r in zip(center,axis,radial)))
                normals.append(tuple((dz*r-dr*a)/normal_scale for r,a in zip(radial,axis)))
        for row in range(len(profile)-1):
            for column in range(SEGMENTS):
                a = row*SEGMENTS+column
                b = row*SEGMENTS+(column+1)%SEGMENTS
                for face in ((a,b,b+SEGMENTS),(a,b+SEGMENTS,a+SEGMENTS)):
                    p,q,r = (points[i] for i in face)
                    area = cross(tuple(y-x for x,y in zip(p,q)),tuple(y-x for x,y in zip(p,r)))
                    if sum(x*x for x in area) > 1e-22:
                        faces.append(face)
        self.parts.append((points,normals,faces,material))

    def body(self, center, axis, length, radius, material, edge=.006):
        edge = min(edge, length/4, radius/4)
        half = length/2
        self.lathe(center,axis,[(-half,0),(-half,radius-edge),
            (-half+edge*.3,radius-edge*.3),(-half+edge,radius),
            (half-edge,radius),(half-edge*.3,radius-edge*.3),
            (half,radius-edge),(half,0)],material)

    def ring(self, center, axis, length, outer, inner, material):
        h = length/2
        self.lathe(center,axis,[(-h,inner),(-h,outer),(h,outer),(h,inner),(-h,inner)],material)

    def bolts(self, center, axis, pitch_radius, count, size=.004):
        axis=unit(axis)
        u=unit(cross((0,1,0) if abs(axis[2])>.9 else (0,0,1),axis));v=cross(axis,u)
        for i in range(count):
            angle=2*math.pi*(i+.5)/count
            p=tuple(c+pitch_radius*(math.cos(angle)*a+math.sin(angle)*b) for c,a,b in zip(center,u,v))
            self.body(p,axis,.002,size,'fastener',edge=.0005)

    def save(self, path):
        root = ET.Element(f'{{{NS}}}COLLADA', version='1.4.1')
        asset=add(root,'asset');add(asset,'contributor');add(asset,'created','2026-11-16T00:00:00Z');add(asset,'modified','2026-11-16T00:00:00Z')
        add(asset,'unit',name='meter',meter='1');add(asset,'up_axis','Z_UP')
        effects=add(root,'library_effects');materials=add(root,'library_materials')
        for name,color in PALETTE.items():
            effect=add(effects,'effect',id=f'{name}-fx');technique=add(add(effect,'profile_COMMON'),'technique',sid='common');phong=add(technique,'phong')
            add(add(phong,'diffuse'),'color',' '.join(map(str,color)))
            add(add(phong,'specular'),'color','0.45 0.45 0.45 1' if name=='steel' else '0.16 0.16 0.16 1')
            add(add(phong,'shininess'),'float','72' if name=='steel' else '36')
            if name=='led':add(add(phong,'emission'),'color','0.12 0.6 0.12 1')
            add(add(materials,'material',id=name,name=name),'instance_effect',url=f'#{name}-fx')
        geometries=add(root,'library_geometries');scene=add(add(root,'library_visual_scenes'),'visual_scene',id='Scene')
        for index,(points,normals,faces,material) in enumerate(self.parts):
            identity=f'part{index}';mesh=add(add(geometries,'geometry',id=identity),'mesh')
            for suffix,values in [('positions',points),('normals',normals)]:
                sid=f'{identity}-{suffix}';source=add(mesh,'source',id=sid)
                add(source,'float_array',' '.join(f'{v:.9g}' for vector in values for v in vector),id=sid+'-array',count=str(len(values)*3))
                accessor=add(add(source,'technique_common'),'accessor',source='#'+sid+'-array',count=str(len(values)),stride='3')
                for component in 'XYZ':add(accessor,'param',name=component,type='float')
            vertices=add(mesh,'vertices',id=identity+'-vertices');add(vertices,'input',semantic='POSITION',source=f'#{identity}-positions')
            triangles=add(mesh,'triangles',material=material,count=str(len(faces)))
            add(triangles,'input',semantic='VERTEX',source=f'#{identity}-vertices',offset='0');add(triangles,'input',semantic='NORMAL',source=f'#{identity}-normals',offset='1')
            add(triangles,'p',' '.join(f'{i} {i}' for face in faces for i in face))
            instance=add(add(scene,'node',id=identity+'-node'),'instance_geometry',url='#'+identity)
            add(add(add(instance,'bind_material'),'technique_common'),'instance_material',symbol=material,target='#'+material)
        add(add(root,'scene'),'instance_visual_scene',url='#Scene')
        ET.indent(root)
        path.write_bytes(ET.tostring(root,encoding='utf-8',xml_declaration=True)+b'\n')


def shells():
    result=[Shell() for _ in range(7)]
    x,y,z=(1,0,0),(0,1,0),(0,0,1)
    # Tapered mounting bell with a machined foot and visible mounting bolts.
    b=result[0];b.body((0,0,.009),z,.018,.095,'steel',.002)
    b.lathe((0,0,0),z,[(.018,0),(.018,.094),(.026,.094),(.050,.088),(.080,.077),(.102,.074),(.107,.073),(.107,0)],'charcoal')
    b.bolts((0,0,.020),z,.085,4,.0045)
    # Rounded large shoulder casting and black rotary seal bands.
    b=result[1];b.body((0,0,.045),z,.100,.075,'charcoal',.012)
    b.body((0,-.051,.100),y,.122,.075,'charcoal',.012)
    b.ring((0,0,.003),z,.009,.0758,.070,'seal')
    b.body((0,-.113,.100),y,.006,.066,'charcoal',.002)
    b.ring((0,-.111,.100),y,.004,.0748,.070,'seal')
    b.bolts((0,-.117,.100),y,.052,4,.0028)
    # Long dark lower arm: broad cast transitions into a slim central tube.
    b=result[2];b.body((0,-.047,0),y,.106,.075,'charcoal',.012)
    b.ring((0,-.003,0),y,.008,.0756,.070,'seal')
    b.lathe((0,-.100,0),z,[(.015,0),(.015,.071),(.060,.073),(.100,.068),(.145,.060),(.190,.059),(.565,.059),(.610,.061),(.660,.062),(.702,.059),(.715,.055),(.715,0)],'charcoal')
    b.ring((0,-.100,.115),z,.005,.065,.058,'seal')
    b.ring((0,-.100,.640),z,.004,.0625,.056,'seal')
    # Elbow + orange upper arm. All outlines are hand-authored approximations.
    b=result[3];b.body((0,.034,0),y,.092,.059,'charcoal',.010)
    b.ring((0,.003,0),y,.009,.0595,.054,'seal')
    b.body((0,-.014,0),y,.006,.051,'charcoal',.002)
    b.bolts((0,-.018,0),y,.040,4,.0024)
    b.lathe((0,.0755,0),(-1,0,0),[(.015,0),(.015,.052),(.050,.055),(.080,.051),(.115,.044),(.160,.043),(.420,.043),(.465,.044),(.505,.045),(.535,.041),(.535,0)],'orange')
    b.ring((-.089,.0755,0),x,.007,.0505,.043,'seal')
    b.ring((-.474,.0755,0),x,.006,.0456,.040,'seal')
    # Orange wrist castings with rounded covers, trim seams and orthogonal necks.
    b=result[4];b.body((0,.0021,0),y,.1512,.045,'orange',.010)
    b.body((-.03885,-.0735,0),x,.0777,.0445,'orange',.009)
    b.ring((0,.003,0),y,.007,.0456,.041,'seal')
    b.ring((0,.067,0),y,.002,.0428,.038,'steel')
    b.body((0,.078,0),y,.004,.0355,'orange',.001)
    b.bolts((0,.0805,0),y,.028,3,.0017)
    b=result[5];b.body((.0021,0,0),x,.1512,.045,'orange',.010)
    b.body((-.0735,-.03885,0),y,.0777,.0445,'orange',.009)
    b.ring((0,0,0),x,.007,.0456,.041,'seal')
    b.ring((.067,0,0),x,.002,.0428,.038,'steel')
    b.body((.078,0,0),x,.004,.0355,'orange',.001)
    b.bolts((.0805,0,0),x,.028,3,.0017)
    b=result[6];b.body((0,.032,0),y,.091,.045,'orange',.009)
    b.ring((0,0,0),y,.007,.0456,.041,'seal')
    b.body((0,-.031,0),y,.046,.0445,'steel',.003)
    b.ring((0,-.017,0),y,.004,.0454,.041,'led')
    b.ring((0,-.022,0),y,.002,.0448,.041,'seal')
    # Annular front flange: centre bore and dark recessed fastener marks.
    b.ring((0,-.0585,0),y,.010,.0315,.01575,'steel')
    b.ring((0,-.0636,0),y,.0005,.0315,.01575,'steel')
    b.bolts((0,-.064,0),y,.025,4,.003)
    # Tool I/O side connector housings, decorative only.
    for zz in (-.013,.013):
        b.body((.042,-.034,zz),x,.012,.006,'fastener',.001)
        b.ring((.0485,-.034,zz),x,.003,.005,.0028,'seal')
    return result


def write_revision(output):
    # Expanded URDF provenance must change when these generated visuals change.
    digest=hashlib.sha256()
    for index in range(7):
        digest.update((output/f'link{index}.dae').read_bytes())
    (output/'revision.xacro').write_text(
        '<?xml version="1.0"?>\n'
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">\n'
        '  <xacro:macro name="hcr14_visual_revision">\n'
        f'    <!-- Authored HCR-14 visual meshes SHA256: {digest.hexdigest()} -->\n'
        '  </xacro:macro>\n</robot>\n')


def main():
    output=Path(__file__).resolve().parents[1]/'visual'
    output.mkdir(exist_ok=True)
    for index,shell in enumerate(shells()):shell.save(output/f'link{index}.dae')
    write_revision(output)


if __name__=='__main__':main()
