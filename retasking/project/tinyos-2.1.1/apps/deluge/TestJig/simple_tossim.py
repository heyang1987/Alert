#! /usr/bin/python

import sys
from TOSSIM import *

t = Tossim([])
m = t.getNode(1);
m.bootAtTime(45654)

f = open("log.txt", "w")
t.addChannel("Boot", f)
t.addChannel("Boot", sys.stdout)

while not m.isOn():
    t.runNextEvent()
