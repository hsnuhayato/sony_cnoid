#!/usr/local/bin/hrpsysjy

import sys,os
import time
from java.lang import System
from java.awt import *
from javax.swing import *
from javax.swing.border import BevelBorder 


#from guiinfo import *
from userJVRC import *
import math

FILENAME_ROBOTHOST='.robothost'
txt = None 

def reconnect():
  robotHost = txt.getText().strip()
  System.setProperty('NS_OPT', '-ORBInitRef NameService=corbaloc:iiop:'+ robotHost +':2809/NameService')
  try:
    rtm.initCORBA()
    f = open(FILENAME_ROBOTHOST, 'w')
    f.write(robotHost+'\n')
    f.close()
    print ('reconnected')
    return True
  except IOError:
    print ('can not open to write: '+ FILENAME_ROBOTHOST)
  except:
    print ('can not connect to '+ robotHost)
  return False

def setupRobot():
  if reconnect():
    init()
    #setupLogger()

def restart():
  waitInputConfirm('!! Caution !! \n Push [OK] to restart rtcd ')
  if reconnect():
    ms = rtm.findRTCmanager()
    ms.restart()

def createButton(name, func):
    if not func.__class__.__name__ == 'function':
       return None
    exec('def tmpfunc(e): import time; t1=time.time();'+func.__name__+'(); t2=time.time(); print "['+name+']", t2- t1 ,"[sec]"')
    btn = JButton(label=name, actionPerformed = tmpfunc)
    del tmpfunc
    return btn

def createFrame():
  global funcList,txt,pnl
  funcList = [["setup rt-system", setupRobot]] + funcList
  frm = JFrame("sample GUI JVRC", defaultCloseOperation = JFrame.EXIT_ON_CLOSE)
  frm = JFrame("sample GUI JVRC")
  frm.setAlwaysOnTop(True)
  pnl = frm.getContentPane()
  pnl.layout = BoxLayout(pnl, BoxLayout.Y_AXIS)
  pnlId = 0
  pnlCount = 0
  MAX_OBJ_NUM = 30
  for i,func in enumerate(funcList):
    obj = None
    if func.__class__.__name__ == 'str':
      if func == '\n':
        pnlCount = MAX_OBJ_NUM
      else:
        obj = JLabel(func)
    elif func.__class__.__name__ == 'function':
      obj = createButton(func.__name__, func)
    elif func.__class__.__name__ == 'list':
      obj = createButton(func[0], func[1])
  
    if obj != None:
      if pnl.getComponentCount() < pnlId+1:
        p = JPanel()
        p.setBorder(BevelBorder(BevelBorder.RAISED))
        p.layout = BoxLayout(p, BoxLayout.Y_AXIS)
        p.setAlignmentY(Component.TOP_ALIGNMENT)
        if pnlId == 0:
          p.add(JLabel("HOSTNAME of ROBOT"))
          if os.path.isfile(FILENAME_ROBOTHOST):
            f = open(FILENAME_ROBOTHOST, "r")
            txt = JTextField(f.readline())
          else:
            txt = JTextField("localhost")
          p.add(txt)
        pnl.add(p)
      pnl.getComponent(pnlId).add(obj)
  
    pnlCount += 1
    if pnlCount > MAX_OBJ_NUM:
      print func
      pnlCount = 0 
      pnlId += 1
  return frm

frm = createFrame()

##button event
obj=JLabel("User Interface")
pnl.add(obj)

def buttonEvent():
    x=in1.getText()
    setObjectV(str(x))

def buttonEvent2():
    x=in2.getText()
    setObjectV(str(x))

def setRotGainEvent():
    x=inRot.getText()
    setRotGain(str(x))

def setPosGainEvent():
    x=inPos.getText()
    setPosGain(str(x))

def setAttGainEvent():
    x=inAtt.getText()
    setAttGain(str(x))

def setTimeEvent():
    x=inTime.getText()
    setconstTime(str(x))

##Attach
obj = createButton("testMove", testMove)
pnl.add(obj)

obj = createButton("start", start)
pnl.add(obj)

obj = createButton("stOn", stOn)
pnl.add(obj)

obj = createButton("stStop", stOff)
pnl.add(obj)

obj = createButton("stepping", stepping)
pnl.add(obj)

obj = createButton("setObjectV", buttonEvent)
pnl.add(obj)
in1=JTextField("5 0 0 0 0 0",15)
pnl.add(in1)

obj = createButton("setObjectV", buttonEvent2)
pnl.add(obj)
in2=JTextField("0 0 0 0 0 0",15)
pnl.add(in2)

obj = createButton("setPosSwitch", setPosSwitch)
pnl.add(obj)

#obj = createButton("stPara", getSTparameter)
#pnl.add(obj)

obj = createButton("setPosGain", setPosGainEvent)
pnl.add(obj)
inPos=JTextField("8000 0.01",15) #70000 0.1
pnl.add(inPos)

obj = createButton("setRotSwitch", setRotSwitch)
pnl.add(obj)

obj = createButton("setRotGain", setRotGainEvent)
pnl.add(obj)
inRot=JTextField("500 500 0.05",15)
pnl.add(inRot)

obj = createButton("setAttSwitch", setAttSwitch)
pnl.add(obj)

obj = createButton("setAttGainRollPitch", setAttGainEvent)
pnl.add(obj)
inAtt=JTextField("0.1 0.1 0.003 0.003",15)
pnl.add(inAtt)

#obj = createButton("setconstTimeP&A", setTimeEvent)
#pnl.add(obj)
#inTime=JTextField("0.1 0.1 ",15)
#pnl.add(inTime)


#obj = createButton("st_hogex", st_hogex)
#pnl.add(obj)

#obj = createButton("clip", clip)
#pnl.add(obj)


frm.pack()
frm.show()

