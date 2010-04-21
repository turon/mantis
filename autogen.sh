#!/bin/bash

export WANT_AUTOMAKE=1.8

aclocal --version | grep aclocal | sed s/'aclocal (GNU automake) '// | grep -q 1\.9.*
if [ $0 ] 
then
    aclocal  | grep -v "underquoted definition"
    autoheader
    automake -a --warnings=none
else
    echo "Automake 1.8 is being used"
    aclocal --version | grep aclocal | sed s/'aclocal (GNU automake) '// | grep -q 1\.8.*
    if [ $0 ]
    then
	aclocal-1.8  | grep -v "underquoted definition"
	autoheader
	automake-1.8 -a --warnings=none
    else
	aclocal --version | grep aclocal | sed s/'aclocal (GNU automake) '// | grep -q 1\.5.*
	if [ $0 ]
	then
	    aclocal-1.5  | grep -v "underquoted definition"
	    autoheader
	    automake-1.5 -a --warnings=none
	else
	    echo "Error: automake 1.5 or newer is required"
	fi
    fi
fi

autoconf

