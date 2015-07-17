#!/usr/bin/env python
"""
@file    version.py
@author  Michael Behrisch
@author  Daniel Krajzewicz
@author  Jakob Erdmann
@date    2007
@version $Id: version.py 11671 2012-01-07 20:14:30Z behrisch $

This script rebuilds "../../src/version.h", the file which
 lets the applications know the version of their build.
If a version file exists, it is kept, otherwise the current
 SVN revision is used as version information.

SUMO, Simulation of Urban MObility; see http://sumo.sourceforge.net/
Copyright (C) 2008-2012 DLR (http://www.dlr.de/) and contributors
All rights reserved
"""

import sys
from os.path import dirname, exists, getmtime, join

versionDir = entriesDir = join(dirname(__file__), '..', '..', 'src')
if len(sys.argv) > 1:
    versionDir = sys.argv[1]
if len(sys.argv) > 2:
    entriesDir = sys.argv[2]
versionFile = join(versionDir, 'version.h')
entriesFile = join(entriesDir, '.svn', 'entries')

svnRevision = "UNKNOWN"
if exists(entriesFile):
    if exists(versionFile) and getmtime(versionFile) > getmtime(entriesFile):
        sys.exit()
    entries = open(entriesFile)
    for i, l in enumerate(entries):
        if i == 3 and l.strip().isdigit():
            svnRevision = l.strip()
        revIndex = l.find('revision="')
        if revIndex >= 0:
            revIndex += 10
            svnRevision = l[revIndex:l.index('"', revIndex)]
            break
    entries.close()

print('generating %s from revision in %s' % (versionFile, entriesFile))
version = open(versionFile, 'w')
print >> version, '#define VERSION_STRING "dev-SVN-r%s"' % svnRevision
version.close()
