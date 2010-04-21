#!/usr/bin/env python

import os
import time

TEST_COUNT = 8
print "Running Automated Test Application...\n"

os.system("touch ../build/mica2/src/tests/automated/test.log")
os.remove("../build/mica2/src/tests/automated/test.log")
os.system("touch ../build/mica2/src/tests/automated/test.log")

os.chdir("../src/tests/automated/")

for n in range(1, TEST_COUNT+1):
    os.chdir("test_"+str(n))
    os.system("./test_"+str(n)+".py")
    os.chdir("../")

os.system("cp ../../../build/mica2/src/tests/automated/test.log ../../../scripts")

print "\nTests complete. See test.log for details.\n"
