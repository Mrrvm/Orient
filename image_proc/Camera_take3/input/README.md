# Obtain data

Data may be obtained through a motion capture software ([OptiTrack Motive](https://v20.wiki.optitrack.com/index.php?title=Quick_Start_Guide:_Getting_Started) for example), or using a grid or even a chessboard.

#  Set data for testing

To test real data under the Matlab Estimator, it must be set on the following formats.

Each axis to be tested for each distance should have 2 files: **rotations.mat** and **data.mat**. 
The **first file** contains all the rotations to be tested, the structure contains the rotation vector in head-fixed coordinates ``rotations.rot``,  the translation
vector ``rotations.tr``, two indexes ``rotations.indImg1`` and ``rotations.inImg2`` to access the images structure that correspond to the images being compared and finally the angle ``rotations.angle`` of rotation in degrees for debug purposes.
The **second file** contains the gray images ``data.img`` and, in the case of Motive data, the position of the markers placed on the camera ``data.marker`` for debug purposes.

This files should be structured on the following way through the data directory to be provided as input to the Matlab Estimator,

```
input/
      d5/
            x/
                rotations.mat
                data.mat
            y/
                rotations.mat
                data.mat
            z/
                rotations.mat
                data.mat
        d10/
                ...
```
where ``d5`` means the depth of 5 meters.
There is no need to include all axis directories, the Matlab Estimator can run using solely ``d5/x/`` for example, but it needs to be structured that way!

As an argument for the Matlab Estimator ``runAll``, the path to the directory with the distances directories should be provided if the type is ``REAL_DISTANCES`` as explained previously, in this case that would be ``real.distance.inputDir = input/``. For a specific distance when testing the error by angle, it should be ``real.inputDir = input/d5/`` for example.

To obtain the input data on the necessary format, scripts for the 3 different methods metioned are provided as follows. For alternative methods, alternative code should be developed.

## OptiTrack Motive data

1) For each distance and axis, the image files should be inside a directory and named with their date of creation. To change their names,
the script [converting.sh](https://github.com/Mrrvm/Visual-Odometry/blob/master/image_proc/Camera_take3/input/Motive/converting.sh) may be used
as ``./converting.sh [images directory]``.

2) Having done that run [filteredataMotive.m](https://github.com/Mrrvm/Visual-Odometry/blob/master/image_proc/Camera_take3/input/Motive/filterdataMotive.m)
as ``filteredataMotive(imgDir, csvFilepath, savePath)``, where ``imgDir`` is the directory with the images, ``cvsFilepath`` is the path to the csv file and ``savePath`` is the path where to save the two files, ``rotations.mat`` and ``data.mat``.

This script finds the position of the rigid body when the image was taken and generates ``data.mat``. Then it computes every possible rotation given
all the images and generates ``rotations.mat``.

## Grid data

1) For each distance and axis, the image files should be inside a directory and named with an unique number, a sign to define the rotation direction and
the angle of rotation, for example ``1+10.jpg``, means image 1 rotated 10 degrees positively.

2) Having done that run [filteredataGrid.m](https://github.com/Mrrvm/Visual-Odometry/blob/master/image_proc/Camera_take3/input/Grid/filterdataGrid.m)
as ``filteredataGrid((imgDir, axis, savePath)``, where ``imgDir`` is the directory with the images, ``axis`` is the axis ``'x'``, ``'y'`` or ``'z'`` and ``savePath`` is the
path where to save the two files, ``rotations.mat`` and ``data.mat``.

This script computes every possible rotation given all the images and their names and generates ``rotations.mat``. As well as images.mat to simplify and make
the Matlab Estimator more generalistic.
