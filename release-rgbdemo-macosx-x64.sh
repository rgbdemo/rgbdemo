#!/bin/sh

dump_ansi_colors ()
{
    for attr in 0 1 4 5 7 ; do
        echo "----------------------------------------------------------------"
        printf "ESC[%s;Foreground;Background - \n" $attr
        for fore in 30 31 32 33 34 35 36 37; do
            for back in 40 41 42 43 44 45 46 47; do
                printf '\033[%s;%s;%sm %02s;%02s  ' $attr $fore $back $fore $back
            done
        printf '\n'
        done
        printf '\033[0m'
    done
}

ansi_red()
{
    printf '\033[1;31;40m'
}

ansi_purple()
{
    printf '\033[1;34;40m'
}

ansi_clear()
{
    printf '\033[0m'
}

try_run ()
{
    echo `ansi_purple` "[RUN]" `ansi_clear` " $@"
    if ! "$@"; then
        echo `ansi_red` "[ERR]" `ansi_clear` " $@"
    fi
}

run ()
{
    echo `ansi_purple` "[RUN]" `ansi_clear` " $@"
    if ! "$@"; then
        echo `ansi_red` "[ERR]" `ansi_clear` " $@"
        exit 1
    fi
}

dbg_vars () # name
{
    for name in $@; do
        echo "[DBG] $name=\"${!name}\""
    done
}

#-------------------------------------------------------------------------------

product () # NAME VERSION PLATFORM
{
    NAME=$1
    VERSION=$2
    PLATFORM=$3

    PRODUCT=$NAME-$VERSION-$PLATFORM

    HERE_DIR="$(pwd)"

    SOURCES_DIR="$HERE_DIR"
    RELEASE_DIR="$HERE_DIR/release"
    BUILD_DIR="$RELEASE_DIR/builds/$PRODUCT"
    PACKAGES_DIR="$RELEASE_DIR/packages"

    PRODUCT_DIR="$RELEASE_DIR/products/$PRODUCT"
    PRODUCT_INCLUDE_DIR="$PRODUCT_DIR/include"
    PRODUCT_LIBRARY_DIR="$PRODUCT_DIR/lib/$PLATFORM"
    PRODUCT_EXECUTABLE_DIR="$PRODUCT_DIR/bin/$PLATFORM"

    dbg_vars                 \
    NAME                     \
    VERSION                  \
    PRODUCT                  \
    SOURCES_DIR              \
    RELEASE_DIR              \
    BUILD_DIR                \
    PRODUCT_DIR              \
    PRODUCT_INCLUDE_DIR      \
    PRODUCT_LIBRARY_DIR      \
    PRODUCT_EXECUTABLE_DIR
}

build_cleanup ()
{
    run rm -rf "$BUILD_DIR"
}

product_cleanup ()
{
    run rm -rf "$PRODUCT_DIR"
}

product_configuration () # CMAKE_ARG ...
{
    run mkdir -p "$BUILD_DIR"
    run cd       "$BUILD_DIR"

    run cmake "$@" "$SOURCES_DIR"
    run cmake .

    run cd -
}

product_targets () # TARGET ...
{
    run cd  "$BUILD_DIR"

    run make "$@"

    run cd -
}

# FIXME: Make this work. Try and let CMake spit target paths.
cmake_target_path () # TARGET
{
    TARGET=$1
    CMAKE_PATH_SCRIPT="$(mktemp -t $PRODUCT-$TARGET-path.cmake)"

    CAT << EOF > "$CMAKE_PATH_SCRIPT"
get_property(TARGET_LOCATION TARGET ${TARGET} PROPERTY LOCATION)
message("\${TARGET_LOCATION}")
EOF

    CMAKE_CACHE_SCRIPT="$(mktemp -t $PRODUCT-$TARGET-cache.cmake)"

    cat "$BUILD_DIR/CMakeCache.txt" \
    | awk 'BEGIN { FS = "[=:]" } ; /^[-_[:alnum:]]+:[-_[:alnum:]]+=.*$/ { print "SET(" $1 " \"" $3 "\" CACHE " $2 " \"\")" }' \
    > "$CMAKE_CACHE_SCRIPT"

#    cat "${CMAKE_CACHE_SCRIPT}"
#    exit

    run cmake -C "$CMAKE_CACHE_SCRIPT" -P "$CMAKE_PATH_SCRIPT" "$BUILD_DIR"
}

product_libraries () # LIBRARY ...
{
    run mkdir -p "$PRODUCT_LIBRARY_DIR"

    for LIBRARY in "$@"; do
        run cp -a "$LIBRARY" "$PRODUCT_LIBRARY_DIR"
    done
}

product_executables () # EXECUTABLE ...
{
    run mkdir -p "$PRODUCT_EXECUTABLE_DIR"

    for EXECUTABLE in "$@"; do
        run cp -a "$EXECUTABLE" "$PRODUCT_EXECUTABLE_DIR"
    done
}

product_applications () # APPLICATION ...
{
    run mkdir -p "$PRODUCT_DIR"

    for APPLICATION in "$@"; do
        NAME="$(basename $APPLICATION)"
        run rsync -avL "$APPLICATION/" "$PRODUCT_DIR/$NAME/"
    done
}

product_dmg ()
{
    run rm -rf "$PACKAGES_DIR/$PRODUCT"
    run mkdir -p "$PACKAGES_DIR/$PRODUCT/$NAME"
    run rsync -avl "$PRODUCT_DIR/" "$PACKAGES_DIR/$PRODUCT/$NAME/"
    run cd "$PACKAGES_DIR/$PRODUCT"
    run ln -s /Applications .
    run cd "$PACKAGES_DIR"
    run zip --symlinks -r "${PRODUCT}.zip" "$PRODUCT"
}

product_zip ()
{
    run mkdir -p "$PACKAGES_DIR"

    run rm -f $PRODUCT.zip
    run cd "$PRODUCT_DIR"
    run zip -r "$RELEASE_DIR"/$PRODUCT.zip .
    run cd -
}

#-------------------------------------------------------------------------------

copy_rgbdemo_libraries()
{
    run mkdir -p "$PRODUCT_DIR"/Common/Contents/MacOS

    run cd "$PRODUCT_DIR"/Common/Contents/MacOS

    # FIXME: Do not blindly copy ALL of the built dynamic libraries, but only rgbdemo ones instead.
    # FIXME: OpenNI module paths are referenced from a dynamically generated configuration file and MUST be placed next to the program executable, (instead of in Contents/Plugins) for now.
    run cp -a "$BUILD_DIR"/lib/*.dylib .

    for bin in *.dylib; do
        for lib in *.dylib; do
            run install_name_tool -change "$BUILD_DIR"/lib/$lib @executable_path/$lib $bin
            run install_name_tool -change                  $lib @executable_path/$lib $bin
        done
    done

    run cd -

    run mkdir -p "$PRODUCT_DIR"/Common/Contents/Frameworks

    run cd "$PRODUCT_DIR"/Common/Contents/Frameworks

    USR_LIBS="\
        libbz2.1.0.dylib \
        libz.1.dylib \
        libflann_cpp.1.6.dylib \
        libboost_date_time-mt.dylib \
        libboost_filesystem-mt.dylib \
        libboost_system-mt.dylib \
        libboost_thread-mt.dylib \
        libboost_iostreams-mt.dylib \
        libcminpack.1.1.3.dylib \
    "

    for usr_lib in $USR_LIBS; do
        run cp /usr/lib/$usr_lib .
    done

    OPENCV_VERSION=2.3

    USR_LOCAL_LIBS="\
        libjpeg.8.dylib \
        libqhull6.dylib \
        libpcl_io.1.4.dylib \
        libpcl_octree.1.4.dylib \
        libpcl_surface.1.4.dylib \
        libpcl_registration.1.4.dylib \
        libpcl_features.1.4.dylib \
        libpcl_filters.1.4.dylib \
        libpcl_segmentation.1.4.dylib \
        libpcl_kdtree.1.4.dylib \
        libpcl_sample_consensus.1.4.dylib \
        libpcl_keypoints.1.4.dylib \
        libpcl_range_image_border_extractor.1.4.dylib \
        libpcl_range_image.1.4.dylib \
        libpcl_common.1.4.dylib \
        libpcl_search.1.4.dylib \
        libpcl_visualization.1.4.dylib \
        libpcl_tracking.1.4.dylib \
        libopencv_calib3d.$OPENCV_VERSION.dylib \
        libopencv_contrib.$OPENCV_VERSION.dylib \
        libopencv_core.$OPENCV_VERSION.dylib \
        libopencv_features2d.$OPENCV_VERSION.dylib \
        libopencv_flann.$OPENCV_VERSION.dylib \
        libopencv_gpu.$OPENCV_VERSION.dylib \
        libopencv_highgui.$OPENCV_VERSION.dylib \
        libopencv_imgproc.$OPENCV_VERSION.dylib \
        libopencv_legacy.$OPENCV_VERSION.dylib \
        libopencv_ml.$OPENCV_VERSION.dylib \
        libopencv_objdetect.$OPENCV_VERSION.dylib \
        libopencv_video.$OPENCV_VERSION.dylib \
        vtk-5.8/libvtkHybrid.5.8.dylib \
        vtk-5.8/libvtkRendering.5.8.dylib \
        vtk-5.8/libvtkVolumeRendering.5.8.dylib \
        vtk-5.8/libvtkWidgets.5.8.dylib \
        vtk-5.8/libvtkGraphics.5.8.dylib \
        vtk-5.8/libvtkverdict.5.8.dylib \
        vtk-5.8/libvtkImaging.5.8.dylib \
        vtk-5.8/libvtkfreetype.5.8.dylib \
        vtk-5.8/libvtkIO.5.8.dylib \
        vtk-5.8/libvtkFiltering.5.8.dylib \
        vtk-5.8/libvtkCommon.5.8.dylib \
        vtk-5.8/libvtksys.5.8.dylib \
        vtk-5.8/libvtkDICOMParser.5.8.dylib \
        vtk-5.8/libvtkNetCDF.5.8.dylib \
        vtk-5.8/libvtkNetCDF_cxx.dylib \
        vtk-5.8/libvtkmetaio.5.8.dylib \
        vtk-5.8/libvtksqlite.5.8.dylib \
        vtk-5.8/libvtkpng.5.8.dylib \
        vtk-5.8/libvtkzlib.5.8.dylib \
        vtk-5.8/libvtkjpeg.5.8.dylib \
        vtk-5.8/libvtktiff.5.8.dylib \
        vtk-5.8/libvtkexpat.5.8.dylib \
        vtk-5.8/libvtkftgl.5.8.dylib \
        vtk-5.8/libvtkexoIIc.5.8.dylib \
    "

    for usr_local_lib in $USR_LOCAL_LIBS; do
        run cp /usr/local/lib/$usr_local_lib .
    done

    OTHER_LIBS="\
        /usr/X11/lib/libpng12.0.dylib \
        /usr/local/Cellar/libtiff/3.9.5/lib/libtiff.3.dylib \
        /usr/local/Cellar/jasper/1.900.1/lib/libjasper.1.dylib"

    for other_lib in $OTHER_LIBS; do
        run cp $other_lib .
    done

    run chmod -R u+w .

    # Boost and pcl libraries have braindead install names that requires fixing.
    for bin in ../MacOS/"$@" *.dylib; do
        try_run install_name_tool -change /usr/local/Cellar/jpeg/8c/lib/libjpeg.8.dylib @executable_path/../Frameworks/libjpeg.8.dylib $bin
        try_run install_name_tool -change /usr/local/Cellar/jpeg/8b/lib/libjpeg.8.dylib @executable_path/../Frameworks/libjpeg.8.dylib $bin
	try_run install_name_tool -change /usr/local/lib/libqhull6.6.2.0.1385.dylib @executable_path/../Frameworks/libqhull6.dylib $bin
        try_run install_name_tool -change /usr/lib/libcminpack.1.1.3.dylib @executable_path/../Frameworks/libcminpack.1.1.3.dylib $bin

	for lib in QtGui QtCore QtNetwork QtOpenGL QtSvg QtXml QtTest; do
            try_run install_name_tool -change ${lib}.framework/Versions/4/${lib} @executable_path/../Frameworks/${lib}.framework/Versions/4/${lib} $bin
	done

        for lib in *.dylib; do
            try_run install_name_tool -id                         @executable_path/../Frameworks/$lib $bin
            try_run install_name_tool -change $lib                @executable_path/../Frameworks/$lib $bin
            try_run install_name_tool -change lib/$lib            @executable_path/../Frameworks/$lib $bin
            try_run install_name_tool -change /opt/local/lib/$lib @executable_path/../Frameworks/$lib $bin
            try_run install_name_tool -change /usr/local/lib/$lib @executable_path/../Frameworks/$lib $bin
            try_run install_name_tool -change /usr/lib/$lib       @executable_path/../Frameworks/$lib $bin
            try_run install_name_tool -change /usr/local/Cellar/vtk/5.8.0/lib/vtk-5.8/$lib @executable_path/../Frameworks/$lib $bin
            try_run install_name_tool -change /usr/X11/lib/$lib @executable_path/../Frameworks/$lib $bin
            try_run install_name_tool -change /usr/local/Cellar/libtiff/3.9.5/lib/$lib @executable_path/../Frameworks/$lib $bin
            try_run install_name_tool -change /usr/local/Cellar/jasper/1.900.1/lib/$lib @executable_path/../Frameworks/$lib $bin
        done
    done

    # For some reasons QtOpenGL does not get copied by macdeployqt
    run mkdir -p QtOpenGL.framework/Versions/4
    run cp -r /Library/Frameworks/QtOpenGL.framework/Versions/4/QtOpenGL QtOpenGL.framework/Versions/4/QtOpenGL
    run install_name_tool -id @executable_path/../Frameworks/QtOpenGL.framework/Versions/4/QtOpenGL QtOpenGL.framework/Versions/4/QtOpenGL
    for lib in QtCore QtGui; do
	run install_name_tool -change ${lib}.framework/Versions/4/$lib @executable_path/../Frameworks/${lib}.framework/Versions/4/${lib} QtOpenGL.framework/Versions/4/QtOpenGL
    done

    run cd -
}

link_rgbdemo_libraries()
{
    run mkdir -p "$PRODUCT_DIR"/"$@".app/Contents/MacOS

    run cd "$PRODUCT_DIR"/"$@".app/Contents/MacOS

    for lib in ../../../Common/Contents/MacOS/*.dylib; do
	ln -s $lib .
    done

    for bin in ../MacOS/"$@"; do
        for lib in *.dylib; do
            run install_name_tool -change "$BUILD_DIR"/lib/$lib @executable_path/$lib $bin
            run install_name_tool -change                  $lib @executable_path/$lib $bin
        done
    done

    run cd -

    run cd "$PRODUCT_DIR"/"$@".app/Contents

    run ln -s ../../Common/Contents/Frameworks .

    run cd Frameworks

    # Boost and pcl libraries have braindead install names that requires fixing.
    for bin in "$PRODUCT_DIR"/"$@".app/Contents/MacOS/"$@"; do
        try_run install_name_tool -change /usr/local/Cellar/jpeg/8c/lib/libjpeg.8.dylib @executable_path/../Frameworks/libjpeg.8.dylib $bin
        try_run install_name_tool -change /usr/local/Cellar/jpeg/8b/lib/libjpeg.8.dylib @executable_path/../Frameworks/libjpeg.8.dylib $bin
	try_run install_name_tool -change /usr/local/lib/libqhull6.6.2.0.1385.dylib @executable_path/../Frameworks/libqhull6.dylib $bin
        try_run install_name_tool -change /usr/lib/libcminpack.1.1.3.dylib @executable_path/../Frameworks/libcminpack.1.1.3.dylib $bin

	for lib in QtGui QtCore QtNetwork QtOpenGL QtSvg QtXml QtTest; do
            try_run install_name_tool -change ${lib}.framework/Versions/4/${lib} @executable_path/../Frameworks/${lib}.framework/Versions/4/${lib} $bin
	done

        for lib in *.dylib; do
            try_run install_name_tool -id                         @executable_path/../Frameworks/$lib $bin
            try_run install_name_tool -change $lib                @executable_path/../Frameworks/$lib $bin
            try_run install_name_tool -change lib/$lib            @executable_path/../Frameworks/$lib $bin
            try_run install_name_tool -change /opt/local/lib/$lib @executable_path/../Frameworks/$lib $bin
            try_run install_name_tool -change /usr/local/lib/$lib @executable_path/../Frameworks/$lib $bin
            try_run install_name_tool -change /usr/lib/$lib       @executable_path/../Frameworks/$lib $bin
            try_run install_name_tool -change /usr/local/Cellar/vtk/5.8.0/lib/vtk-5.8/$lib @executable_path/../Frameworks/$lib $bin
            try_run install_name_tool -change /usr/X11/lib/$lib @executable_path/../Frameworks/$lib $bin
            try_run install_name_tool -change /usr/local/Cellar/libtiff/3.9.5/lib/$lib @executable_path/../Frameworks/$lib $bin
            try_run install_name_tool -change /usr/local/Cellar/jasper/1.900.1/lib/$lib @executable_path/../Frameworks/$lib $bin
        done
    done

    run cd -
}

copy_rgbdemo_resources()
{
    run mkdir -p ${PRODUCT_DIR}/"$@".app/Contents/Resources

    run cd ${PRODUCT_DIR}/"$@".app/Contents/Resources

    # run cp ${SOURCES_DIR}/apps/skanect/skanect.icns .

    run cd -
}

#-------------------------------------------------------------------------------

product RGBDemo 0.7.0 Darwin

#skip()
{

#build_cleanup

# FIXME: When specifying explicit target universal binary architectures,
# CMake adds the -isysroot GCC flag for the current MacOSX SDK.
# This makes <SDK_PATH>/usr/include shadow /usr/include, thus removing
# custom-installed libraries (namely: pcl dependencies) from the include
# search paths.
# For now, just build for the default architecture.
#    -DCMAKE_OSX_ARCHITECTURES:STRING=i386;x86_64 \

products_configuration \
    -DCMAKE_VERBOSE_MAKEFILE:BOOL=TRUE \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_MACOSX_BUNDLE:BOOL=TRUE \
    -DNESTK_USE_PCL:BOOL=TRUE \

# FIXME: Because rgbdemo does not depend on dynamically loaded OpenNI modules, all targets must be built, so that the following copy operation can succeed:
# cp release/builds/rgbdemo-0.7.0-macosx-x64/bin/lib{nimCodecs,nimMockNodes,nimRecorder,XnDevicesSensorV2,XnDeviceFile,XnVFeatures,XnVHandGenerator}.dylib release/products/rgbdemo-0.7.0-macosx-x64/skanect.app/Contents/MacOS
product_targets       \
    all               \
    glew              \
    glview            \
    OpenNI            \
    nimCommon         \
    nimCodecs         \
    nimMockNodes      \
    nimRecorder       \
    XnCore            \
    XnFormats         \
    XnDDK             \
    XnDeviceFile      \
    XnDevicesSensorV2
}

product_cleanup

apps="rgbd-viewer \
      rgbd-reconstructor \
      rgbd-multikinect \
      rgbd-people-tracker \
      rgbd-scan-markers \
      rgbd-scan-topview \
      rgbd-skeletor \
      calibrate_kinect_ir \
      calibrate-openni-intrinsics \
      calibrate_projector \
      calibrate-multiple-kinects \
      "

copy_rgbdemo_libraries

for app in $apps; do 
    product_applications ${BUILD_DIR}/bin/${app}.app
    link_rgbdemo_libraries $app
done

copy_rgbdemo_resources

#product_zip

product_dmg
