# lepton
Linux Driver for Lepton Thermal Camera

# Command to view thermal images with mplayer
mplayer -x 240 -y 180 -zoom 100 tv:// -tv driver:v4l2:width=80:height=60:device=/dev/video0:outfmt=rgb24 -fps 23 
