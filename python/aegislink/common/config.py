"""Shared settings: the frame rate, the camera image size and the default UDP ports.

The default ports match this proxy command line (links are numbered 1, 2, 3 in this order):

    ew_chaos_proxy --link 5600:5601 --link 5602:5603 --link 5604:5605 --control 5700

Link 1 carries the foveated stream, link 2 the full-frame baseline, link 3 the receiver reports
back to the UAV. With --direct the nodes skip the proxy and talk to each other's ports.
"""

FPS = 30
FRAME_WIDTH, FRAME_HEIGHT = 1280, 720

FOVEA_TO_PROXY, FOVEA_TO_GCS = 5600, 5601
BASELINE_TO_PROXY, BASELINE_TO_GCS = 5602, 5603
REPORTS_TO_PROXY, REPORTS_TO_UAV = 5604, 5605
PROXY_CONTROL = 5700

FOVEA_LINK, BASELINE_LINK, REPORTS_LINK = 1, 2, 3
