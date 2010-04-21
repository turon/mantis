#!/bin/bash

echo "MANTIS Project Distribution Builder"
echo ""

ORIG_CWD=`pwd`

if ! test "$1"
then
    echo "Please pass the directory name for the first arg"
    exit
elif ! test "$2"
then
    echo "Please pass the version for the second arg"
    exit
elif ! test -d "$1"
then
    printf "Directory '$1' not found, would you like to create it? [y/n] "
    read result
    if test "$result" != "y"
    then
	echo "Aborting"
	exit
    fi
    mkdir $1
fi

echo "Using version '$2'"
printf "Ok to proceed? [y/n] "
read result
if test "$result" != "y"
then
    echo "Aborting"
    exit
fi

VERSION=$2
cd $1
CWD=`pwd`
MANTIS_DIR="mantis-$VERSION"
MANTIS_DOCS_DIR="mantis-docs-$VERSION"
MANTIS_DEVEL_DIR="mantis-devel-$VERSION"

if test -d "mantis"
then
    echo "'mantis' dir already exists in '$1', please fix"
    exit
fi
if test -d "$MANTIS_DIR"
then
    echo "'$MANTIS_DIR' dir already exists in '$1', please fix"
    exit
fi
if test -d "mantis-docs"
then
    echo "'mantis-docs' dir already exists in '$1', please fix"
    exit
fi
if test -d "$MANTIS_DOCS_DIR"
then
    echo "'$MANTIS_DOCS_DIR' dir already exists in '$1', please fix"
    exit
fi
if test -d "mantis-devel"
then
    echo "'mantis-devel' dir already exists in '$1', please fix"
    exit
fi
if test -d "$MANTIS_DEVEL_DIR"
then
    echo "'$MANTIS_DEVEL_DIR' dir already exists in '$1', please fix"
    exit
fi


echo ""
echo "Performing svn checkout"

SVN=`which svn`

if ! test -e "$SVN"
then
    echo ""
    echo "Couldn't find subversion executable 'svn'"
    echo "Please install Subversion from http://subversion.tigris.org"
    echo ""
fi

MANTIS="svn+ssh://mantis.cs.colorado.edu/home/svn/repos/mantis"
MANTIS_DOCS="svn+ssh://mantis.cs.colorado.edu/home/svn/repos/mantis-docs"
MANTIS_DEVEL="svn+ssh://mantis.cs.colorado.edu/home/svn/repos/mantis-devel"

echo "Checking out mantis source"
$SVN co $MANTIS
if test "$?" -ne 0
then
    echo "Problem checking out mantis source"
fi

echo "Checking out mantis docs"
$SVN co $MANTIS_DOCS
if test "$?" -ne 0
then
    echo "Problem checking out mantis-docs source"
fi

echo "Checking out mantis devel tools"
$SVN co $MANTIS_DEVEL
if test "$?" -ne 0
then
    echo "Problem checking out mantis-devel source"
fi

mv mantis $MANTIS_DIR
cd $MANTIS_DIR
./autogen.sh
find -name .svn -type d | xargs rm -rf
cd ..
echo "Creating $MANTIS_DIR tarball"
tar czf $MANTIS_DIR.tar.gz $MANTIS_DIR

mv mantis-docs $MANTIS_DOCS_DIR
cd $MANTIS_DOCS_DIR
find -name .svn -type d | xargs rm -rf
cd ..
echo "Creating $MANTIS_DOCS_DIR tarball"
tar czf $MANTIS_DOCS_DIR.tar.gz $MANTIS_DOCS_DIR

mv mantis-devel $MANTIS_DEVEL_DIR
cd $MANTIS_DEVEL_DIR
find -name .svn -type d | xargs rm -rf
cd ..
echo "Creating $MANTIS_DEVEL_DIR tarball"
tar czf $MANTIS_DEVEL_DIR.tar.gz $MANTIS_DEVEL_DIR

echo ""
echo "Done creating distribution tarballs"
printf "Would you like to branch for the new version? [y/n] "
read result
if test "$result" == "y"
then
    $SVN cp $MANTIS $MANTIS-$VERSION
    $SVN cp $MANTIS_DOCS $MANTIS_DOCS-$VERSION
    $SVN cp $MANTIS_DEVEL $MANTIS_DEVEL-$VERSION
fi

echo "$0 completed successfully"
exit 0
