References: 
1) http://sourceforge.net/projects/makeavi/
2) http://makeavi.sourceforge.net/

MakeAVI

Introduction
------------------------------------------------------------------------
This is a quick and dirty app that I wrote because I wanted to do some 
work with creating time-lapse movies.  I set up a digital camera to take 
a picture every 10 seconds for several hours, then string the JPGs
together into an AVI, and do any video work from there.

I took a quick look around, but didn't find any free implementations for
Windows.  So I looked around and found some code to base this on, and
about 4 hours later, here it is.

The core of the AVI generation is the Microsoft/Windows AVI API.  Wrapped
around that is a thin wrapper, which I found at www.codeguru.com, but 
it may be from a Microsoft sample app, I don't know.

The image loading end is handled by the FreeImage library, which is
currently at http://www.6ixsoft.com/.  It can handle many different image
types, but not GIF.  GIF is patent burdened and will not be supported, so
don't bother asking.

License:
MakeAVI is free software, licensed under the GPL (GNU General Public 
License).  Significant portions of MakeAVI (specifically, the 
FreeImage library) are also licensed under the GPL.  For more information, 
please read the LICENSE.TXT file that should have come with this release.  
If you do not have a copy of this file, you can find it at 
http://www.gnu.org/licenses/gpl.txt


INSTALLATION
------------------------------------------------------------------------
Unzip all this stuff into one directory.  Run MAKEAVI.EXE.  

There, it's installed.  If you want to get cute, right click on MakeAVI,
hit "Create shortcut" and drag the shortcut onto your desktop.




Using MakeAVI
------------------------------------------------------------------------
You want to select all your images into the list on the left.  Use the
"Add Files" button to choose your files.

Use the "Add Files", "Delete File", "Sort Files", and "Move file Up/Down" 
buttons to get the images in the order you want them.

Any image that is selected on the left will be previewed in the middle.
If it isn't previewed properly, then the program is unable to read
the image (can't access the file, or the file is corrupt, or something).

CROPPING:
All the images have to be the same size to go into an AVI file
If you have images of varying sizes, or you have junk at the edge that
you want to eliminate from the final AVI, you can use the "Cropping"
interface at the bottom.  Check the "Crop to" box, and select the 
size you want the final images to be cropped to in the boxes to the right.

NOTE that the original images are NOT cropped on disk, they are only
cropped after they're loaded, and before they're added to the AVI.

NOTE that currently, images are cropped towards the center, you can't
select where the cropping occurs.

As an aid to filling in these values, if you add new images to the list,
the size of the last image added is put into those edit boxes.  Also,
you can double click on any image in the list at any time and the size
of that image will be copied in to the boxes.


Ready to go...
When you've gotten the images all in order, input your frame rate in
the box provided; when you play the final AVI file, that many images
will be played per second.

Then, press the "Begin!" button.  The feedback box will be displayed,
and you will be asked which video format you wish to save to.  I won't
try to talk about video formats here, except for a few observations:

If you will be continuing to work with the video, perhaps encoding it
to other formats, then you may want to use "Uncompressed" - though
that generates HUGE files.  In general time lapse stuff is not that big,
so your file will probably only be a few gigabytes, so this is not
a bad choice, but it should NOT be used for the final product; the 
data rate is so high that most computers will not be able to play
the video smoothly.

If you are going for a final AVI product, Indeo 4.5 or higher is a good
choice, since everyone either has it or can get the codec automatically
when they start to play the video under Windows media player.


Possible future enhancements:
Cropping other than from the center
Resizing images
Fixing support for a few codecs that don't work for some reason