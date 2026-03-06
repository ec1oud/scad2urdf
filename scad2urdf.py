#!/usr/bin/env python

import math
import os
import string

import numpy as np
import pymeshlab as ml

NONE_STATEMENT = 0
JOIN_STATEMENT = 1
LINK_STATEMENT = 2
current_task = NONE_STATEMENT

link_db = {}

filepath = "walker.scad"
printable = set(string.ascii_letters).union(set(string.digits))
names = []
rgb = "['1', '1', '0']"

urdfout = "robot.urdf"
robotname = "pointone"

wf = open(urdfout, "w")
wf.write('<?xml version="1.0" encoding="utf-8"?>\n')
wf.write('<robot name="' + robotname + '">\n')
wf.write("\n")


def write_join(joins, jtrans=[0, 0, 0], jrota=[0, 0, 0]):

    # get parent translation
    parent = joins[0].strip()
    child = joins[1].strip()
    ptrans = link_db[parent]
    ctrans = link_db[child]
    print("HINGE -------------------")
    print(parent, child)

    print("Parent Origin", ptrans)
    print("Child Origin", ctrans)
    print("Hinge Origin", jtrans)

    trans = [ctrans[i] - jtrans[i] for i in [0, 1, 2]]
    print("Artefact Origin", trans)

    ja_out = '"'
    for i in trans:
        ja_out = ja_out + str(i) + " "
    ja_out = ja_out + '"'

    print("Translation Option", ja_out)

    print()

    # ja_out = '"0 0.15 0"'

    # create vector from rotation
    a = np.radians(int(jrota[0]))
    b = np.radians(int(jrota[1]))
    g = np.radians(int(jrota[2]))

    sa = math.sin(a)
    ca = math.cos(a)

    sb = math.sin(b)
    cb = math.cos(b)

    sg = math.sin(g)
    cg = math.cos(g)

    axis = np.array([1, 0, 0])
    rot_mat = np.array(
        [
            [ca * cb, ca * sb * sg - sa * cg, ca * sb * cg + sa * sg],
            [sa * cb, sa * sb * sg + ca * cg, sa * sb * cg - ca * sg],
            [-sb, cb * sg, cb * cg],
        ]
    )
    rot_axis = np.dot(axis, rot_mat)
    rot_axis = [str(round(i, 4)) for i in rot_axis]
    ra_out = '"'
    for i in rot_axis:
        ra_out = ra_out + i + " "
    ra_out = ra_out + '"'

    ra_out = '"1 0 0"'

    [par, child] = joins
    par = "".join(par.split())
    child = "".join(child.split())
    wf.write('<joint name="' + par + "2" + child + '" type="revolute">\n')
    wf.write("<origin xyz=" + ja_out + "/>\n")
    wf.write("<axis xyz=" + ra_out + "/>\n")
    wf.write('<parent link="' + par + '"/>\n')
    wf.write('<child link="' + child + '"/>\n')
    wf.write("</joint>\n")
    wf.write("\n")


def get_inertial_data(mesh, measures):
    """Extract inertia tensor and center of mass from mesh and geometric measures.

    Args:
        mesh: The mesh object from pymeshlab
        measures: Geometric measures dictionary from compute_geometric_measures()

    Returns:
        tuple: (inertia_xml, com_xml) where:
            - inertia_xml: The <inertia> tag with inertia tensor values
            - com_xml: The <origin> tag with center of mass position
    """
    # Get center of mass from measures
    com = measures['center_of_mass']
    com_xyz = [str(float(com[i])) for i in range(3)]
    com_xml = '<origin rpy="0 0 0" xyz="' + " ".join(com_xyz) + '"/>\n'

    # Get inertia tensor from measures
    inertia = measures['inertia_tensor']
    ixx = float(inertia[0][0])
    ixy = float(inertia[0][1])
    ixz = float(inertia[0][2])
    iyy = float(inertia[1][1])
    iyz = float(inertia[1][2])
    izz = float(inertia[2][2])

    # Format inertia values to reasonable precision
    def fmt(val):
        return f"{val:.6f}".rstrip('0').rstrip('.')

    inertia_xml = (
        f'<inertia ixx="{fmt(ixx)}" ixy="{fmt(ixy)}" ixz="{fmt(ixz)}" '
        f'iyy="{fmt(iyy)}" iyz="{fmt(iyz)}" izz="{fmt(izz)}" />\n'
    )

    return inertia_xml, com_xml


def write_link(linkname, trans, rgb, filename_stl, measures):
    ftrans = [float(i) for i in trans]
    link_db[linkname] = ftrans

    wf.write('<link name="' + linkname + '">\n')
    wf.write("<visual>\n")
    wf.write(
        ' <origin rpy="0 0 0" xyz="'
        + trans[0]
        + " "
        + trans[1]
        + " "
        + trans[2]
        + '"/>\n'
    )
    wf.write("  <geometry>\n")
    wf.write('    <mesh filename="' + filename_stl + '"/>\n')
    wf.write("  </geometry>\n")
    wf.write('  <material name="">\n')
    wf.write('    <color rgba="' + rgb[0] + " " + rgb[1] + " " + rgb[2] + ' 1"/>\n')
    wf.write("  </material>\n")
    wf.write("</visual>\n")
    wf.write("<collision>\n")
    wf.write(
        '  <origin rpy="0 0 0" xyz="'
        + trans[0]
        + " "
        + trans[1]
        + " "
        + trans[2]
        + '"/>\n'
    )
    wf.write("  <geometry>\n")
    wf.write('    <mesh filename="' + filename_stl + '"/>\n')
    wf.write("  </geometry>\n")
    wf.write("</collision>\n")
    wf.write("<inertial>\n")
    # Compute mass from mesh volume (assuming density of 1 for now)
    # You can adjust the density multiplier as needed
    density = 1.0
    mass = measures['mesh_volume'] * density
    wf.write(f'  <mass value="{mass:.6f}"/>\n')

    im, com = get_inertial_data(ms.current_mesh(), measures)
    wf.write("  " + com)
    wf.write("  " + im)
    wf.write("</inertial>\n")
    wf.write("</link>\n")
    wf.write("\n")


with open(filepath) as fp:
    s = 0
    p = 0
    line = " "
    outline = ""
    idio = 0
    while line:
        a = line.count("{") - line.count("}")
        s = s + a

        if line.startswith("//"):  ## two possible statements Join and Link
            # JOIN needs two links
            if "join" in line:
                joins = line.strip().split(":")[1].split(",")
                current_task = JOIN_STATEMENT

            # LINK is defined by link name and mesh
            if "link" in line:
                myline = line.strip().split(":")[1]
                myname = "".join(filter(lambda x: x in printable, myline))
                print(myname)
                current_task = LINK_STATEMENT

        # JOIN get only "translate" and "rotate" lines
        if current_task == JOIN_STATEMENT:
            if "translate" in line:
                ns = line.split("[")[1].split("]")[0].split(",")
                jtrans = [float(i) for i in ns]

            if "rotate" in line:
                jrota = line.split("[")[1].split("]")[0].split(",")

            # if chunk is read from file - then join it
            if s == 0 and p == 1:
                write_join(joins, jtrans, jrota)

        # LINK get chunk of scad file and export it to an Binary STL File
        if current_task == LINK_STATEMENT:
            # Store co-ords and remove from intermediate mesh
            if ("translate" in line) and (s == 0):
                trans = line.split("[")[1].split("]")[0].split(",")
                line = "translate([0,0,0])"

            if "color" in line:
                rgb = line.split("[")[1].split("]")[0].split(",")

            # outline is chunk of SCAD commands
            outline = outline + line

            # if chunk is read from file - then export it
            if s == 0 and p == 1:
                if myname == None:
                    filename_scad = str(idio) + "mesh.scad"
                    filename_stl = str(idio) + "mesh.stl"

                else:
                    if myname in names:
                        filename_scad = myname + str(idio) + ".scad"
                        filename_stl = myname + str(idio) + ".stl"
                        filename_stlout = myname + str(idio) + "2.stl"
                        filename_inertia = myname + str(idio) + "i.stl"
                        linkname = myname + str(idio)
                    else:
                        filename_scad = myname + ".scad"
                        filename_stl = myname + ".stl"
                        filename_stlout = myname + "2.stl"
                        filename_inertia = myname + "i.stl"
                        linkname = myname
                        names.append(myname)

                    myname = None

                idio = idio + 1

                f = open(filename_scad, "w")
                f.write(outline)
                f.close()

                os.system("openscad " + filename_scad + " -o " + filename_stl)

                ms = ml.MeshSet()
                ms.load_new_mesh(filename_stl)
                # Compute geometric measures for inertial data
                measures = ms.get_geometric_measures()

                os.system("rm " + filename_stl)
                os.system("rm " + filename_scad)

                # write urdf with stl file
                write_link(linkname, trans, rgb, filename_stl, measures)

                outline = ""
        p = s
        line = fp.readline()


wf.write("</robot>\n")
wf.close()

print(link_db)
