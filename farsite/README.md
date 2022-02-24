

#################  Project General Overview  #################

FARSITE is a fire growth model. See firelab.org/https://www.firelab.org/project/farsite for the general description of Farsite.

There are three versions of Farsite that we know of. On firelab.org under Apps and Products/Fire Behavior/Farsite/Farsite Software you can download an installer (so not source code) for Windows. This version has a nice GUI unlike the one on github, and is actually a little bit farther ahead with some of the features than the github version. This version can be helpful to understand the ideas of the inputs for Farsite, for creating ignition files, and for viewing results from the github version. While you can get an understanding of the inputs to Farsite with this firelab.org version of Farsite, the inputs do not necessarily match up between different versions of Farsite. You almost need to double check the github code directly to understand what it uses for inputs.

Technically Farsite is dead code. The third location is probably the most up to date version, and it is managed by Stuart Brittain. It is definitely not open to the public, and the github Farsite is basically just a section of this Assembla version ripped out to only run on Linux (but ripped out a few years back). It's a bit weird, cause this Assembla version is built for Windows, and is actually more for Flammap than for Farsite, but it still has the Farsite in it as the Assembla version appears to be a collection of a bunch of different Fire Behavior related packages. Probably what happened was that Flammap and Farsite were always very similar (both give spatial fire behavior vs the single point fire behavior of behave, but Flammap doesn't do behavior with time like Farsite), so they were worked on separately, then eventually Farsite got merged back into Flammap. This means that the structure exists to put a lot of Flammap stuff into Farsite, but Farsite has really just kind of been sitting there in that code.

While the Assembla version is not open to the public, it may be possible to get a binary version executable for Windows from Stuart. Stuart maintains the Assembla version of Farsite for use in FlamMap6 desktop and cloud modeling services being developed by IBM. The binaries described in this paragraph were originally written for WFDSS and subsequently integrated into FlamMap desktop. The reason the Assembla version and thus this github version of Farsite have so many extra parts is cause the Assembla code base is not intended just for FlamMap, but for an entire suite of modeling programs, of which Farsite is a distinct project and model.

Also, it looks like the github version was ripped out of the Assembla version sometime relatively close to release of the latest firelab.org version, so it has most of the features of that version, but is behind by maybe a version or two. Eventually, we are hoping that someone will go through and manually update as much as possible to this github version, using the most up to date Assembla version. For now, the github version "should" be good enough for the purposes of general fires with gridded winds.

So here is where the github version is compared to the Windows version and Assembla version: It doesn't have post frontal combustion, it does have moisture stuff, but maybe not the best version, it needs the rogue trajectory spotting fixed by a later update, it needs the ignition shapefile processing to handle old fire perimeters, it needs the ignition shapefiles to handle a single point instead of a small circle of points, it needs gdal stuff builtin to handle shapefiles with projections correctly (now you manually need to make sure everything matches projections and it is fine).

The WindNinja stuff you see in the github version of Farsite, if you open the set of files in an IDE like qt creator and right click on the functions with the "find usages" tool, you will see that at some point the WindNinja stuff exists there, but is not linked in or used anywhere. This is also true of the Flammap stuff, which you probably will see has more inputs than what is used in Farsite. These WindNinja files are an old version of the WindNinja API, so just ignore it. Instead of trying to get that set of code to work, we took the ideas from Assembla farsite to put in functionality for gridded wind files so you can just use output from WindNinja as inputs to Farsite.


Some extra notes from Stu:
The version in Git is not maintained by me, but it was indeed taken from the version I maintain in assembla with the removal of GDAL and WindNinja libraries, and merged with the Fuel Conditioning code.

Neither the assembla nor git versions utilize post-frontal combustion, nor do they allow for attacking the fire. I do not believe anything else was stripped from the git version as compared to the assembla version. Since GDAL and WindNinja have been removed from the git version, that version is not projection aware nor can it run WindNinja on it's own.


#################  General Overview of Examples  #################

Not exactly sure where the files for the Panther example come from, it is apparent that there are two .lcp files for which the panther case is run on, namely 1177973.lcp and cust.lcp. The 1177973 case has to be run as is, while the cust case has the option to try out a different ignition file and custom fuels file by changing some of the input files.

The cougarCreek was recently added after work on a separate repo to automate farsite runs using WRF files as inputs to WindNinja: https://github.com/latwood/WRF-WindNinja-FarsiteScript. The cougarCreek case was generated in the first working run of said other repo script and uses gridded wind outputs from WindNinja.

The test1177973.input file is an example input file that Loren put together in the summer of 2018, as he tried to figure out what inputs are actually used in this version of farsite, and what is actually done by the code at different steps. Probably is not an easy read, but might be helpful. To make WindNinja happy during these experiments, the 1177973 case was renamed test1177973.

At some point in time an attempt to convert .raws files to .wnd and .wtr was done using ideas from the test1177973.input file. The script doing the conversion was lost, but the method should have been to simply extract values from the .raws file for the .wnd file, and to do averaging of .raws values to setup the .wnd file. While said script for the conversion was lost, the resulting .wnd and .wtr files were added to the test1177973 case for future use.

The flatland case was given to Loren by Stu after it was discovered that Linux farsite was not running gridded WindNinja winds properly. It acts as the ideal test case because terrain does not affect, which makes it easy to compare using different kinds of inputs on the results. Because of this, when the flatland case was added to the farsite repo, it contains a ton of different case comparisons, as well as pictures showing that the gridded wind fixes to Linux farsite were effective. Note that using gridded WindNinja winds with a .atm file did not work, but gridded WindNinja winds with a .atm file and a .raws file for the weather (ignoring the .raws winds) did work. Part of the reason for this is that the .raws input triggers an interpolation method in Linux farsite to extend the .raws input weather to cover the entire required weather time period, but the same is not done for .atm inputs. Note that it is hard to tell whether the atm vs the RAWS file is working correctly, but the otherRAWS case helps as an additional comparison. When Linux Farsite did not work with WindNinja gridded winds, the atmRAWS and RAWS looked the same regardless of the atm files used.


#################  Step by Step Guide for Building and Running the Script  #################

The following worked on fresh installations of Ubuntu 16.04 and 18.04:
```
cd <pathToFarsiteRepo>/farsite/src
make
./TestFARSITE ../examples/Panther/runPanther.txt 2>&1 | tee scriptRun.log
```

Before any of the examples will run, you need to replace all occurrences of `$scriptRoot/farsite` in many of the examples files with `<pathToRepo>/farsite`. For example, if your username is `john` and you have placed the farsite repo in a directory called `src` in your `$HOME` directory, you need to replace `$scriptRoot/farsite` with `/home/john/src/farsite`. (notice that `farsite` is repeated in both texts to change, so can drop the `/farsite` when running keyword replacer command line utilities).

Here is some command line code you can use to modify the paths:
```
editingDir="<pathToFarsiteRepo>/farsite/examples"
textToReplace="\$scriptRoot"
replacementText="<pathToFarsiteRepo>"
## prepare the text by replacing all "/" chars with "\/" chars
preppedTextToReplace=$(sed 's/\//\\\//g' <<<"$textToReplace")
preppedReplacementText=$(sed 's/\//\\\//g' <<<"$replacementText")
## prepare the text by replacing all "=" chars with "\=" chars
preppedTextToReplace=$(sed 's/\=/\\\=/g' <<<"$preppedTextToReplace")
preppedReplacementText=$(sed 's/\=/\\\=/g' <<<"$preppedReplacementText")
grep -rl "${preppedTextToReplace}" "${editingDir}" --exclude-dir=.git --exclude-dir=src --exclude=readme | xargs sed -i 's/'"${preppedTextToReplace}"'/'"${preppedReplacementText}"'/g'
success=$?
echo $success
```

Note that a 0 means success, a 123 means nothing needed replaced, anything else is probably a fail. You'll probably have to play around a bit to get the right paths.


If you hate that changing the paths means running "git status" results in a bunch of changed files you really don't want to keep track of, since git has to keep track of the example files but they change in a way that means they should be ignored unless you are directly developing them since the paths all changed, you can use something like the following:
```
examplesDir="<pathToFarsiteRepo>/farsite/examples"
git ls-files -- ${examplesDir} | xargs -l git update-index --assume-unchanged
```

If you realize you need to make development changes to all the example files and want to retrack these ignored but tracked example files, use:
```
git ls-files -- ${examplesDir} | xargs -l git update-index --no-assume-unchanged
success=$?
```

If you just want to retrack a single example file that you've been doing development work on, use:
```
git update-index --no-assume-unchanged <fileToRetrack>
```

Note that running the farsite examples will overwrite past output with new output, which only should be tracked if you are doing development work on the example files, so the above git ignore stuff might be even more useful than you think. The main reason the old output is tracked in these example files is so you can look over what the normal outputs of farsite are like without running farsite.


#################  General Overview for Viewing Results on Windows Machine  #################

If you don't want to install farsite, you're on your own, just need some kind of GIS software:

go to https://www.firelab.org/, on this webpage look for "apps and products/fire behavior/farsite". On this farsite page, look for "farsite software". This leads you to a farsite for windows installer.

After installing farsite, run farsite, and in the menu search for "view/view landscape file (.LCP)/2D window". Use the "cougarCreekFire.lcp" file.

After the file loads, right click the section of the box labeled "Visible Theme" in the section with the numbers and colors. This should pop up a menu "Choose Color Ramp". Select "Load Color File (.CLR)" and use the "colorInput.CLR" file. Note you need to make your own if doing a different .lcp file than this example, I randomly changed colors till it looked good, then saved this color scheme.

In the menu search for "view/View Vector File". Change the extensions it is looking for to ".shp", then you can view the "cougarCreekFire_Perimeters.shp" fire perimeters and "cougarCreekFire_Spots.shp" probabilistic spot location files. Need to select the boxes labeled "2D" to get the files to actually show the pictures, left click the lines or dots to choose colors, right click to choose the sizes.

Note that you can see other files as well, but might need to hunt down their specific types using either "vector" or "raster" input from the "view" menu.




