#
# This is the default configuration for the jig generator tool,
# and are chosen to be useful defaults that can reliably work
# for most users. Naturally, they won't necessarily be optimal
# for individual setups. With time, we can keep tuned values
# for specific common cases (e.g. 3D printers, components, etc)
# in the repository.
#
# Keep this well commented, and current.  This will help tool
# users understand and tune.
#
def get():
    return '''
# All dimensions are specified in millimeters
#
# Please see documentation for meaning of "gap", "overlap", and "perimeter"
#
[pcb]
thickness = 1.6
tesellate_edge_cuts_curve = 0.1

[holder]
# PCB holder holds both the PCB and components in place, in proper alignment
# This allows the user to use both hands while soldering, achieving the
# highest quality results.

# PCB rests at the top of the PCB holder. "pcb_overlap" decides how much
# plastic will be under the PCB on its edges, supporting it on the holder.
# A small overlap is enough to ensure that the PCB won't fall into the jig.
pcb_overlap = 0.3

# PCBs have an xy dimension accuracy of manufacturing, which shall be
# specified by the manufacturer. Beyond that, you also need to consider the
# printing accuracy of the 3D printer. "pcb_gap" ensures the PCB can be
# placed in the jig comfortably, without flexing the jig in any dimension.
pcb_gap = 0.3

# Drawings on the EdgeCuts layer provide the PCB outline. At every vertex
# on the PCB outline, a "lip" is created, to hold the PCB. Lips are created
# at these points:
#  - start end points of lines
#  - corners of rectangles
#  - start, mid and end points of arcs
#  - north, east, west, south points on circles
#  - each vertex on a polygon
#
# lips are generated only on the external edge of the PCB, i.e. they are not
# generated on interior drawings such as holes.
#
# Use 0 to enforce a lip along the entire PCB edge
#
lip_size = 15

# In some cases, you may want to force addition of lips at specific points.
#Use this. Note lip will be centerd on this point.
forced_lips = [
  #  [ X, Y ] coordinates from KiCAD PCB
]

# Holder will have a solid border around the edge points of the PCB. This
# provides rigidity to the jig structure.
perimeter = 1.6

[holder.base]
# Holder has a base. The base provides structural rigidity, and can be
# used for purposes such as annotation.

# Type of the base can be
# - "mesh". This is a sparse structure built of thick lines. This helps
#   reduce plastic usage, and can reduce print time. This also improves
#   visibility of components when they are placed in the jig.
# - "solid". A flat plate. More space for annotation.
type = "mesh"

# Thickness of the base. Higher value will improve rigidity in the
# xy dimension (apart from the lips)
thickness = 1

# A "perimeter" can be added on top of the base. This also improves the
# rigidity of the structure.
#
# Note that this dimension is additional to the base thickness.
perimeter_height = 2

[holder.base.mesh]
# Lines of the mesh are generated with this width. Best to use at-least 4
# times your nozzle thickness, if 3D printing. Thicker lines will use
# more material and increase printing time.
line_width = 2.0

[refs]
# Generic properties around component refs.
do_not_process = [
  # list of refs that we should ignore
]
process_only_these = [
  # list of refs to process
  # this takes precedence over "do_not_process"
  # this is exclusive with do_not_process
]

[component_shell]
# Around each through hole component (ref), the jig generator creates a "shell"
# that serves as a component holder at its exact location on the board.

# shell can have one of a few styles
# - outline => outline of the external shape
#              (convex hull). Component has a bit of
#              a wiggle room
# - fitting => multiple outlines, like a step-well
#              each level helps hold the component
#              in place, and also reduces wiggle room
# - tight   => step-well of concave hulls.
#
# "fitting" and "tight" are not implemented yet.
type = "outline"

# component will typically be inserted from the top side (w.r.t # the PCB, and
# the jig). However, they can also be inserted from the bottom of the jig.
# bottom insertion means that base will have a hole to allow the component to
# be inserted. The shell type is forced to "outline" in this case.
# valid values : "top" or "bottom"
component_insertion = "top"

# Shells are basically a skin of plastic around the component, of this
# minimum thickness along the Z axis.
thickness = 1.2

# You a small xy gap for each component to slide into the shell, and it must
# ideally sit snug without moving. Component sizes have tolerance. So does
# you 3D. Consider both to set this up.
gap = 0.1

# If you have SMD components on the board, you may need a gap between the
# shells and the PCB. The gap ensures SMD components don't touch the shells.
#
# Think of this as a vertical "keep-out" distance between the PCB and the
# shells
clearance_from_pcb = 1

[jig]
#
# Jigs of various types can be generated:
#
# - "th_soldering" creates a jig to help solder
#   through hole (TH) components. This creates
#   the PCB holder, the base, and the shells
#   for all components.
#
# - "component_fitting" creates only shells,
#   and the base, without creating the holder.
#
type = "th_soldering"

# NOTE:
#
# Many of the parameters here map to OpenSCAD, and can be tuned there.
# E.g., parameters that are related to printing/manufacturing tolerances can be
# tuned in OpenSCAD, without access to the original PCB file.
#
# Parameters that result in geometry generation in the tool (e.g. lips)
# aren't tunable from OpenSCAD. Also, the shapes of the shells aren't
# tunable from OpenSCAD as parameters, but thickness and height can be
# easily changed. Please tune parameters carefully, and always 
# cross check generated models before printing/manufacturing.
#
'''
