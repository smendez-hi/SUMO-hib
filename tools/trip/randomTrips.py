#!/usr/bin/env python
# -*- coding: iso-8859-1 -*-

"""
@file    randomTrips.py
@author  Michael Behrisch
@date    2010-03-06
@version $Id: randomTrips.py 11671 2012-01-07 20:14:30Z behrisch $

Generates random trips for the given network.

SUMO, Simulation of Urban MObility; see http://sumo.sourceforge.net/
Copyright (C) 2010-2012 DLR (http://www.dlr.de/) and contributors
All rights reserved
"""

import os, sys, random, bisect, subprocess
from datetime import datetime as dt
from optparse import OptionParser
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import sumolib.net

def randomEdge(edges, cumWeights):
    r = random.random() * cumWeights[-1]
    return edges[bisect.bisect(cumWeights, r)]
    
def vprint (*args): vprint1(*args)
def vprint1(*args):
  if options.verbose:
    for arg in args:
      print arg
def vprint3(*args):
  if options.verbosity>2:
    for arg in args:
      print arg

optParser = OptionParser()
optParser.add_option("-n", "--net-file", dest="netfile",
  help="define the net file (mandatory)")
optParser.add_option("-o", "--output-trip-file", dest="tripfile",
  default="trips.trips.xml", help="define the output trip filename")
optParser.add_option("-r", "--route-file", dest="routefile",
  help="generates route file with duarouter")
optParser.add_option("-t", "--trip-id-prefix", dest="tripprefix",
  default="SHNormVeh", help="prefix for the trip ids")
# added by uprego 2012 Aug 14
optParser.add_option("-T", "--fev-trip-id-prefix", dest="fevtripprefix",
  default="SHElecVeh", help="prefix for the fev trip ids")
optParser.add_option("-a", "--trip-parameters", dest="trippar",
  default="", help="additional trip parameters")
optParser.add_option("-b", "--begin", type="int", default=0, help="begin time")
optParser.add_option("-e", "--end", type="int", default=3600, help="end time")
optParser.add_option("-p", "--period", type="int", default=1,
 help="repetition period")
optParser.add_option("-V", "--verbosity",type="int",default=0,help="verbosity")
optParser.add_option("-s", "--seed", type="int", help="random seed")
optParser.add_option("-l", "--length", action="store_true",
  default=False, help="weight edge probability by length")
optParser.add_option("-L", "--lanes", action="store_true",
  default=False, help="weight edge probability by number of lanes")
optParser.add_option("-v", "--verbose", action="store_true",
  default=False, help="tell me what you are doing")
optParser.add_option('-S','--start',dest='startTime',default='0',
 help='start hour for the generation of traffic, goes from 0 (default) to 23 '
 '(max)')
optParser.add_option('-D', '--specify-day', dest = 'day', type = 'int',
 default = 0, help = 'Specify a day from 0 for monday to 6 for sunday (defaul'
 't monday)')
# tdchar stands4 trafficDemandCharacter
optParser.add_option('-c','--tdchar',dest='tdchar',default='unspecified',
  help='Characterization option for the traffic demand')
# added by uprego 2012 Aug 10
optParser.add_option("-m", "--multiplier", type="int", default=1,
 help="multiplier for the number of vehicles")
# added by uprego 2012 Aug 14
optParser.add_option("-P", "--fevsPercentage", type="int", default=10,
 help="F.E.V.s percentage (two digit-ted number) to simulate")
optParser.add_option('-I', '--force-italy', action = 'store_true',
 default = False)
optParser.add_option('-G', '--force-germany', action = 'store_true',
 default = False)
(options, args) = optParser.parse_args()
vprint('options.startTime{'+options.startTime+'}')
if not options.netfile:
  optParser.print_help()
  sys.exit()
net = sumolib.net.readNet(options.netfile)
if options.seed:
  random.seed(options.seed)
probs=[]
total = 0
for edge in net._edges:
  prob = 1
  if options.length:
    prob *= edge.getLength()
  if options.lanes:
    prob *= edge.getLaneNumber()
  total += prob
  probs.append(total)

idx = 0
jdx = 0
fouttrips = file(options.tripfile, 'w')
print >> fouttrips, """<?xml version="1.0"?>
<!-- generated on %s by $Id: randomTrips.py revNo? 201?-??-?? author? -->
<trips>""" % dt.now()
vehicles = (range(options.begin, options.end, options.period).__len__() * 
 options.multiplier)
vprint('Generating trips for '+str(vehicles)+' vehicles')
version='0.0.2'
if version=='0.0.1':
  for depart in range(options.begin,options.end,options.period):
    vprint3('depart{'+depart+'} in range(options.begin,options.end,options.p'
     'eriod){'+range(options.begin,options.end,options.period)+'}')
    # added by uprego 2012 Aug 10
	# this loop multiplies by that factor the number of trips to simulate
    for i in range(options.multiplier):
      vprint3('i{'+i+'} in range(options.multiplier){'+
       range(options.multiplier)+'}')
      # declare
      label = ""
      r = random.randint(1,100)
      if r < 5 or r > 95:
	    vprint('your random is '+str(r)+
	     '. options.fevsPercentage is '+str(options.fevsPercentage)+'.')
      if r > options.fevsPercentage:
        # Then is not a fev
        prefix = options.tripprefix
        dx = idx = idx + 1
      else: # r <= options.fevsPercentage
        # Then is a fev
        prefix = options.fevtripprefix
        dx = jdx = jdx + 1
      if vehicles < 10 :
        label = "%s%d" % (prefix, dx-1)
      elif vehicles < 100 and dx-1 < 10:
        label = "%s0%d" % (prefix, dx-1)
      elif vehicles < 100 and dx-1 < 100:
        label = "%s%d" % (prefix, dx-1)
      elif vehicles < 1000 and dx-1 < 10:
        label = "%s00%d" % (prefix, dx-1)
      elif vehicles < 1000 and dx-1 < 100:
        label = "%s0%1d" % (prefix, dx-1)
      elif vehicles < 1000 and dx-1 < 1000:
        label = "%s%d" % (prefix, dx-1)
      elif vehicles < 10000 and dx-1 < 10:
        label="%s000%d"%(prefix,dx-1)
      elif vehicles < 10000 and dx-1 < 100:
        label="%s00%d"%(prefix,dx-1)
      elif vehicles < 10000 and dx-1 < 1000:
        label="%s0%d"%(prefix,dx-1)
      elif vehicles < 10000 and dx-1 < 10000:
        label="%s%d"%(prefix,dx-1)
      elif vehicles < 100000 and dx-1 < 10:
        label = "%s0000%d" % (prefix, dx-1)
      elif vehicles < 100000 and dx-1 < 100:
        label = "%s000%d" % (prefix, dx-1)
      elif vehicles < 100000 and dx-1 < 1000:
        label = "%s00%d" % (prefix, dx-1)
      elif vehicles < 100000 and dx-1 < 10000:
        label = "%s0%d" % (prefix, dx-1)
      elif vehicles < 100000 and dx-1 < 100000:
        label = "%s%d" % (prefix, dx-1)
      elif vehicles < 1000000 and dx-1 < 10:
        label = "%s00000%d" % (prefix, dx-1)
      elif vehicles < 1000000 and dx-1 < 100:
        label = "%s0000%d" % (prefix, dx-1)
      elif vehicles < 1000000 and dx-1 < 1000:
        label = "%s000%d" % (prefix, dx-1)
      elif vehicles < 1000000 and dx-1 < 10000:
        label = "%s00%d"%(prefix,dx-1)
      elif vehicles < 1000000 and dx-1 < 100000:
        label = "%s0%d" % (prefix, dx-1)
      elif vehicles < 1000000 and dx-1 < 1000000:
        label = "%s%d" % (prefix, dx-1)
      else : # Damit...
        label = "%s%9d" % (prefix, dx-1)
      #print label
      sourceEdge = randomEdge(net._edges, probs)
      sinkEdge = randomEdge(net._edges, probs)
      # patch uprego 2012 Aug 10, original
      print >> (fouttrips, '    <trip id="%s" depart="%s" '\
	   'from="%s" to="%s" %s/>' %
	   (label, depart, sourceEdge.getID(), sinkEdge.getID(), options.trippar))
      # patch uprego 2012 Aug 10, new
      #print >> fouttrips, '    <trip id="%s" depart="%s" from="%s" to="%s"'
	  # '%s/>' \
      #                    % (label, 2*depart+i, sourceEdge.getID(),
	  #                       sinkEdge.getID(), options.trippar)
    ##end for 'for i in range(options.multiplier)'
  ##end for 'for depart in range(options.begin, options.end, options.period)'
elif version=='0.0.2':
  if options.tdchar=='random':
    demandsVector=[0 for i in range(24)]
    for i in range(0,24):
      demandsVector[i]=int(random.random()*4)
  else: # is reallife or undefined
    '''
    demandsVector=
       NIGHT             MORNING           AFTERNOON         EVENING
       0-1h    3-4h      6-7h    9-10h     12-13h  15-16h    18-19h  21-22h
      [0,0,0,  0,0,0,    1,2,3,  2,1,1,    1,2,3,  3,2,1,    1,2,2,  2,1,1]
    '''
    #              0      3       6      9       12     15      18     21
    demandsVector=[1,0,0, 0,0,1,  2,4,8, 4,2,2,  2,3,4, 3,3,2,  3,4,5, 4,3,2]
    #options.day = 4 # FIXME
	#   According to our source documents, weekend flow is different, and
	# depends also on day of the weekend, saturday is more trafficked (may
	# mainly be because of people working saturday)
    if options.day == 5 or options.day == 6:
      '''
      demandsVector=
         NIGHT             MORNING           AFTERNOON         EVENING
         0-1h    3-4h      6-7h    9-10h     12-13h  15-16h    18-19h  21-22h
        [0,0,0,  0,0,0,    1,2,3,  2,1,1,    1,2,3,  3,2,1,    1,2,2,  2,1,1]
      '''
      #              0     3     6     9     12    15    18    21
      demandsVector = [1,0,0,0,0,1,2,3,4,4,3,2,2,3,3,3,3,3,4,5,5,4,3,2]
      if options.day == 5:
        #              0     3     6     9     12    15    18    21
        demandsVector = [1,0,0,0,0,1,2,3,4,4,3,2,2,3,3,3,3,3,3,4,4,4,3,2]
    # Flows are different for friday
    if options.day == 4:
      #                0     3     6     9     12    15    18    21
      demandsVector = [1,0,0,0,0,1,2,3,6,3,2,2,2,3,4,3,3,3,4,5,5,4,3,2]
    # There also less people at tuesday, wednesday and thursday morning
    if options.day == 1 or options.day == 2 or options.day == 3:
      demandsVector = [1,0,0,0,0,1,2,3,7,3,2,2,2,3,4,3,3,2,3,4,5,4,3,2]
  #options.force_germany = True # FIXME
  #options.force_italy = True # FIXME
  if options.force_germany:
	# More multiplier factoring because of a bigger net
    for i in range(0, 24):
      demandsVector[i] += 1
      demandsVector[i] *= 4
  if options.force_italy:
    for i in range(0, 24):
      demandsVector[i] += 1
      #demandsVector[i] *= 3 # FIXME
  # elif options.force_italy is just ok
  vprint3('str(demandsVector){'+str(demandsVector)+'}')
  for depart in range(options.begin,options.end,options.period):
    if depart%3600==0:
      vprint(str(dt.now())+': Now entering the '+
       str((int(options.startTime)+depart/3600)%24)+' timeframe')
    # added by uprego 2012 Aug 10
    actualMultiplier=options.multiplier
    vprint3('str(int(options.startTime)+depart/3600){'+
	 str((int(options.startTime)+depart/3600)%24)+'}')
    vprint3('str(demandsVector[int(options.startTime)+depart/3600]){'+
	 str(demandsVector[(int(options.startTime)+depart/3600)%24])+'}')
    actualMultiplier+=demandsVector[(int(options.startTime)+depart/3600)%24]
    vprint3('str(options.multiplier){'+str(options.multiplier)+'}')
    vprint3('str(actualMultiplier){'+str(actualMultiplier)+'}')
    # Force random 'halving'
    #if random.randint(0,1):
	# This loop multiplies the number of trips to simulate by that factor
    for i in range(actualMultiplier):
      if random.randint(0,4) < 2:
        # declare
        label = ""
        r = random.randint(1,100)
        if r < 5 or r > 95:
		  vprint('your random is '+str(r)+'. options.fevsPercentage is '+
		   str(options.fevsPercentage)+'.')
        if r > options.fevsPercentage:
          # Then is not a fev
          prefix = options.tripprefix
          dx = idx = idx + 1
        else: # r <= options.fevsPercentage
          # Then is a fev
          prefix = options.fevtripprefix
          dx = jdx = jdx + 1
        '''
        if vehicles < 10 :
          label = "%s%d" % (prefix, dx-1)
        elif vehicles < 100 and dx-1 < 10:
          label = "%s0%d" % (prefix, dx-1)
        elif vehicles < 100 and dx-1 < 100:
          label = "%s%d" % (prefix, dx-1)
        elif vehicles < 1000 and dx-1 < 10:
          label = "%s00%d" % (prefix, dx-1)
        elif vehicles < 1000 and dx-1 < 100:
          label = "%s0%1d" % (prefix, dx-1)
        elif vehicles < 1000 and dx-1 < 1000:
          label = "%s%d" % (prefix, dx-1)
        elif vehicles < 10000 and dx-1 < 10:
          label="%s000%d"%(prefix,dx-1)
        elif vehicles < 10000 and dx-1 < 100:
          label="%s00%d"%(prefix,dx-1)
        elif vehicles < 10000 and dx-1 < 1000:
          label="%s0%d"%(prefix,dx-1)
        elif vehicles < 10000 and dx-1 < 10000:
          label="%s%d"%(prefix,dx-1)
        elif vehicles < 100000 and dx-1 < 10:
          label = "%s0000%d" % (prefix, dx-1)
        elif vehicles < 100000 and dx-1 < 100:
          label = "%s000%d" % (prefix, dx-1)
        elif vehicles < 100000 and dx-1 < 1000:
          label = "%s00%d" % (prefix, dx-1)
        elif vehicles < 100000 and dx-1 < 10000:
          label = "%s0%d" % (prefix, dx-1)
        elif vehicles < 100000 and dx-1 < 100000:
          label = "%s%d" % (prefix, dx-1)
        elif vehicles < 1000000 and dx-1 < 10:
          label = "%s00000%d" % (prefix, dx-1)
        elif vehicles < 1000000 and dx-1 < 100:
          label = "%s0000%d" % (prefix, dx-1)
        elif vehicles < 1000000 and dx-1 < 1000:
          label = "%s000%d" % (prefix, dx-1)
        elif vehicles < 1000000 and dx-1 < 10000:
          label = "%s00%d"%(prefix,dx-1)
        elif vehicles < 1000000 and dx-1 < 100000:
          label = "%s0%d" % (prefix, dx-1)
        elif vehicles < 1000000 and dx-1 < 1000000:
          label = "%s%d" % (prefix, dx-1)
        else : # Damit...
          label = "%s%9d" % (prefix, dx-1)
        '''
        if dx-1<10:
          label="%s00000%d"%(prefix,dx-1)
        elif dx-1<100:
          label="%s0000%d"%(prefix,dx-1)
        elif dx-1<1000:
          label="%s000%d"%(prefix,dx-1)
        elif dx-1<10000:
          label="%s00%d"%(prefix,dx-1)
        elif dx-1<100000:
          label="%s0%d"%(prefix,dx-1)
        elif dx-1<1000000:
          label="%s%d"%(prefix,dx-1)
        else: # Damit...
          label="%s%9d"%(prefix,dx-1)
        vprint3(label)
        sourceEdge = randomEdge(net._edges, probs)
        sinkEdge = randomEdge(net._edges, probs)
        # patch uprego 2012 Aug 10, original
        print >> fouttrips, '    <trip id="%s" depart="%s" from="%s" to="%s"'\
		 '%s/>' % (label, depart, sourceEdge.getID(), sinkEdge.getID(), 
     options.trippar)
        # patch uprego 2012 Aug 10, new
        #print >> fouttrips, '    <trip id="%s" depart="%s" from="%s" to="%s"'
		# '%s/>' \
        #                    % (label, 2*depart+i, sourceEdge.getID(),
        #                       sinkEdge.getID(), options.trippar)
      ##end for 'for i in range(options.multiplier)'
    ##end if 'random limiting'
  ##end for 'for depart in range(options.begin, options.end, options.period)'  
##end if 'version selection'  
fouttrips.write("</trips>")
fouttrips.close()

if options.routefile:
  subprocess.call([
   'duarouter',
   '-n', options.netfile,
   '-t', options.tripfile,
   '-o', options.routefile])
