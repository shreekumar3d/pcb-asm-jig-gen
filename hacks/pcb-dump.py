# Test to understand pcbnew's python interface, units
# etc
import pcbnew
import sys

board = pcbnew.LoadBoard(sys.argv[1])
fp_list = board.Footprints()
for fp in fp_list:
    print('Component ', fp.GetReference(), ' at ', fp.GetPosition())
    print('    Attribs = ', fp.GetAttributes()) # What are these ?
    print('    TH ? ', fp.HasThroughHolePads())
    for mod3d in fp.Models():
        print('    3D Model File = ', mod3d.m_Filename)
        print('        Rot = ', mod3d.m_Rotation)
        print('        Scale = ', mod3d.m_Scale)
        print('        Offset = ', mod3d.m_Offset)

