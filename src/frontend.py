"""
Kaiy Muhammad
vilo
frontend.py
Frontend: Image processing and feature extraction
"""

"""
Subscriber callback for Oak-D camera monochrome stereo
#TODO Ensure time synchronization
/oakd/stereo_ir/left/image_rect /oakd/stereo_ir/right/image_rect

"""


"""
Point selectionsel
- Direct dense
- Direct sparse
- Feature-based
"""
def point_selection(selection_type)
    raise NotImplementedError("This method has not been implemented yet")


"""
Image alignment
Get point correspondences
Get transformation between two images
Update keyframe if transformation is significant
Get aproximate pose estimate to give to backend as initial guess
"""

"""
Point refinement
Elimiate points projected outside of the image
"""


"""
Landmark management
Create landmarks if alignment succeeds
"""