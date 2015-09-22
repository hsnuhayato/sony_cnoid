import os

import sys
import socket
import traceback
import math
import time
import java.lang.System

import rtm
import waitInput
#import bodyinfo
import OpenHRP
#from OpenHRP.RobotHardwareServicePackage import SwitchStatus

global posswitch,rotswitch,attswitch
posswitch=1
rotswitch=1
attswitch=1

def init(robotHost=None):
    if robotHost != None:
      print 'robot host = '+robotHost
      java.lang.System.setProperty('NS_OPT',
          '-ORBInitRef NameService=corbaloc:iiop:'+robotHost+':2809/NameService')
      rtm.initCORBA()

    print "creating components"
    rtcList = createComps(robotHost)

    print "connecting components"
    connectComps()

    print "activating components"
    activateComps(rtcList)

    print "initialized successfully"

def activateComps(rtcList):
    rtm.serializeComponents(rtcList)
    for r in rtcList:
        r.start()

def initRTC(module, name):
    ms.load(module)
    return ms.create(module, name)


"""
def goActual():
    sh_svc.goActual()

def setJointAngle(jname, angle, tm, wait=True):
    seq_svc.setJointAngle(jname, angle*pi/180.0, tm)
    if wait:
        seq_svc.waitInterpolation()

def setJointAnglesDeg(pose, tm, wait=True):
    seq_svc.setJointAngles(bodyinfo.makeCommandPose(pose), tm)
    if wait:
        seq_svc.waitInterpolation()


def goInitial(tm=bodyinfo.timeToInitialPose):
    if isServoOn(): 
      putRobotUp()
      waitInputConfirm( \
        '!! Robot Motion Warning !!\n\n' +\
        'Push [OK] to move to the Initial Pose')
    setJointAnglesDeg(bodyinfo.initialPose, tm)

def loadPattern(basename, tm=3.0):
    seq_svc.loadPattern(basename, tm)


def goHalfSitting(tm=bodyinfo.timeToHalfsitPose, wait=True):
    if isServoOn():
      putRobotUp()
      waitInputConfirm( \
        '!! Robot Motion Warning !!\n\n' +\
        'Push [OK] to move to the Half-Sit Pose')
    setJointAnglesDeg(bodyinfo.halfsitPose, tm, False)
    waistpos = [0,0,bodyinfo.halfsitWaistHeight]
    if not seq_svc.setBasePos(waistpos, tm):
        print "setBasePos() failed"
    zmp = [0,0,-bodyinfo.halfsitWaistHeight]
    if not seq_svc.setZmp(zmp, tm):
        print "setZmp() failed"
    if wait==True:
        seq_svc.waitInterpolation()
    stOn()

#def exPos(tm=bodyinfo.timeToHalfsitPose, wait=True):
def exPos(tm=1.0, wait=True):
    if isServoOn():
      putRobotUp()
      #waitInputConfirm( \
      #  '!! Robot Motion Warning !!\n\n' +\
      #  'Push [OK] to move to the exPose')
    setJointAnglesDeg(bodyinfo.exPose, tm, False)
    #waistpos = [0,0,bodyinfo.halfsitWaistHeight]
    #if not seq_svc.setBasePos(waistpos, tm):
    #    print "setBasePos() failed"
    zmp = [0,0,-bodyinfo.halfsitWaistHeight]
    if not seq_svc.setZmp(zmp, tm):
        print "setZmp() failed"
    if wait==True:
        seq_svc.waitInterpolation()
    #stOn()
"""

def stOn():
    #st.setProperty("isEnabled", "1")
    #st_svc.startStabilizer()
    print "no comment"

def stOff():
    #st.setProperty("isEnabled", "0")
    print "no comment"   

def createComps(hostname=socket.gethostname()):
    global ms, user, user_svc, log, rh, servo, joystick, kf, st, st_svc
    ms = rtm.findRTCmanager(hostname)
    rh = rtm.findRTC("JVRC-1")
    if ms==None:
        print "no ms"
    else:
        user = initRTC("sony", "wpg")
        joystick= initRTC("GamepadRTC","joystick")
        kf= initRTC("KalmanFilter","kf")
        st= initRTC("Stabilizer","st")
      
    servo= rtm.findRTC("creekPdServo0")

    if servo==None:
        print "no servo component"
        return

    if rh==None:
        print "no robot component"
        return

    if user==None:
        print "no user component"
        return

    user_svc = OpenHRP.sonyServiceHelper.narrow(user.service("service0"))
    if user_svc==None:
        print "no svc"

    st_svc = OpenHRP.StabilizerServiceHelper.narrow(st.service("service0"))
    if st_svc==None:
        print "no st svc"

    if joystick==None:
        print "no joystick component"

    rtcs=[rh, joystick, kf, user, st]
    #rtcs=[rh, joystick, kf, user]
    
    return rtcs

def connectComps():
    #rtm.connectPorts(user.port("refq"),    servo.port("qRefIn"))
    #rtm.connectPorts(user.port("refq"),    user.port("mc"))#tempepory
    #rtm.connectPorts(rh.port("q"),    user.port("mc"))#tempepory
    rtm.connectPorts(rh.port("rfsensor"), user.port("rfsensor"))
    rtm.connectPorts(rh.port("lfsensor"), user.port("lfsensor"))
    rtm.connectPorts(rh.port("rhsensor"), user.port("rhsensor"))
    rtm.connectPorts(rh.port("lhsensor"), user.port("lhsensor"))
    #rtm.connectPorts(user.port("light"), rh.port("light"))

    if joystick!=None:
        print "connect to gamepad"
        rtm.connectPorts(joystick.port("axes"),user.port("axes"))
        rtm.connectPorts(joystick.port("buttons"),user.port("buttons"))

    if kf != None:
        rtm.connectPorts(rh.port("gsensor"),  kf.port("acc"))
        rtm.connectPorts(rh.port("gyrometer"),  kf.port("rate"))
        
    if st!= None:
    #if 0:
        rtm.connectPorts(kf.port("rpy"), st.port("rpy"))
        rtm.connectPorts(rh.port("rfsensor"), st.port("forceR"))
        rtm.connectPorts(rh.port("lfsensor"), st.port("forceL"))
        rtm.connectPorts(rh.port("q"), st.port("qCurrent"))
        rtm.connectPorts(user.port("refq"), st.port("qRef"))
        rtm.connectPorts(user.port("rzmp"), st.port("zmpRef"))
        rtm.connectPorts(user.port("basePosOut"), st.port("basePosIn"))
        rtm.connectPorts(user.port("baseRpyOut"), st.port("baseRpyIn"))
        rtm.connectPorts(user.port("contactStates"), st.port("contactStates"))
        #rtm.connectPorts(user.port("localEEpos"), st.port("localEEpos"))
        rtm.connectPorts(st.port("q"),  servo.port("qRef"))
        rtm.connectPorts(st.port("q"),    user.port("mc"))
    else:
        rtm.connectPorts(user.port("refq"),  servo.port("qRef"))

"""    
def connectComps():
    rtm.connectPorts(rh.port("q"), [sh.port("currentQIn"),
                                st.port("q")])
   
    if servo != None:
        rtm.connectPorts(rh.port("rfsensor"), st.port("forceR"))
        rtm.connectPorts(rh.port("lfsensor"), st.port("forceL"))
        rtm.connectPorts(rh.port("gyrometer"), [st.port("rate"),
                                            kf.port("rate")])
        rtm.connectPorts(rh.port("gsensor"), [kf.port("acc"),
                                          st.port("acc")])
        #rtm.connectPorts(rh.port("gyrometer"),kf.port("rate"))
        #rtm.connectPorts(rh.port("gsensor"),kf.port("acc"))

    #
    rtm.connectPorts(kf.port("rpy"), st.port("rpy"))
    #
    #rtm.connectPorts(seq.port("qRef"),    sh.port("qIn"))
    rtm.connectPorts(seq.port("qRef"),    st.port("qRefIn"))#new
    rtm.connectPorts(seq.port("qRef"),    st.port("qRefIn"))
    rtm.connectPorts(seq.port("basePos"), sh.port("basePosIn"))
    rtm.connectPorts(seq.port("baseRpy"), sh.port("baseRpyIn"))
    #rtm.connectPorts(seq.port("accRef"),  kf.port("accRef"))
    #rtm.connectPorts(seq.port("zmpRef"),  st.port("zmpRef"))
    
    #origin
    #rtm.connectPorts(sh.port("qOut"),       [st.port("qRefIn"),
    #                                     seq.port("qInit")])
    rtm.connectPorts(sh.port("qOut"),seq.port("qInit"))
    rtm.connectPorts(sh.port("basePosOut"), [seq.port("basePosInit")])
    #rtm.connectPorts(sh.port("baseRpyOut"), [st.port("rpyRef"),
    #                                         seq.port("baseRpyInit")])
    rtm.connectPorts(sh.port("baseRpyOut"),seq.port("baseRpyInit"))
    #
    if servo: # simulation_mode with dynamics or real robot mode
        rtm.connectPorts(st.port("qRefOut"), servo.port("qRefIn"))#origin is qRef here.careful  
        print "servoMode"
    else:     # simulation mode without dynamics
        rtm.connectPorts(sh.port("qOut"), rh.port("qIn"))
    #
    
##user's port
    rtm.connectPorts(rh.port("rhsensor"), user.port("rhsensor"))
    rtm.connectPorts(rh.port("lhsensor"), user.port("lhsensor"))
    rtm.connectPorts(rh.port("rfsensor"), user.port("rfsensor"))
    rtm.connectPorts(rh.port("lfsensor"), user.port("lfsensor"))
    rtm.connectPorts(user.port("rzmp"),  st.port("zmpRef"))
    rtm.connectPorts(rh.port("q"), user.port("q"))
    #rtm.connectPorts(user.port("refq"),    sh.port("qIn"))
    rtm.connectPorts(user.port("refq"),    st.port("qRefIn"))#new

    #rtm.connectPorts(user.port("basePOS"),  sh.port("basePosIn"))
    #rtm.connectPorts(user.port("baseRPY"),  sh.port("baseRpyIn"))
    #rtm.connectPorts(sh.port("basePosOut"), user.port("basePOSInit"))
    #rtm.connectPorts(sh.port("baseRpyOut"), user.port("baseRPYInit"))
    #rtm.connectPorts(seq.port("basePos"), user.port("basePOSInit"))
    #rtm.connectPorts(seq.port("baseRpy"), user.port("baseRPYInit"))
    #rtm.connectPorts(seq.port("zmpRef"), user.port("refZMPInit"))
    #for mc
    rtm.connectPorts(st.port("qRefOut"), user.port("mc"))
   
   
def setupLogger():
    global log_svc
    log_svc = OpenHRP.DataLoggerServiceHelper.narrow(log.service("service0"))
    log_svc.add("TimedDoubleSeq", "q")
    log_svc.add("TimedDoubleSeq", "qRef")
    log_svc.add("TimedDoubleSeq", "zmpRefSeq")
    log_svc.add("TimedDoubleSeq", "basePos")
    log_svc.add("TimedDoubleSeq", "rpy")
    rtm.connectPorts(rh.port("q"), log.port("q"))
    #rtm.connectPorts(st.port("qRefOut"), log.port("qRef"))
    rtm.connectPorts(seq.port("zmpRef"), log.port("zmpRefSeq"))
    rtm.connectPorts(sh.port("basePosOut"), log.port("basePos"))
    #rtm.connectPorts(kf.port("rpy"), log.port("rpy"))
    ##userlog
    #log_svc.add("TimedDoubleSeq", "wZMP")
    #rtm.connectPorts(user.port("wZMP"), log.port("wZMP"))
 

def saveLog(fname='sample'):
    if log_svc == None:
      waitInputConfirm("Setup DataLogger RTC first.")
      return
    #log_svc.save(fname)
    cwd=os.getcwd()
    log_svc.save(cwd+"/log/%s_wutest_%s"%(bodyinfo.modelName.lower(), time.strftime('%Y%m%d%H%M')))
    print 'saved'

##
## for real robot
##
import thread
def getActualState():
  stateH = OpenHRP.RobotHardwareServicePackage.RobotStateHolder()
  rh_svc.getStatus(stateH)
  return stateH.value

def isServoOn(jname = "any"):
  if simulation_mode:
    return True
  ss = getActualState().servoState
  if jname == 'any':
    for s in ss:
      if (s&2) > 0:
        return True
    return False

  if jname == 'all':
    for s in ss:
      if (s&2) == 0:
        return False
    return True

  return False

def isCalibDone():
  if simulation_mode:
    return True
  ss = getActualState().servoState
  for s in ss:
    if (s&1) == 0:
      return False
  return True

def countDownLoop(t):
  while (t >= 0):
      print t
      t = t - 1
      time.sleep(0.98)

def countDown(t):
  thread.start_new_thread(countDownLoop, (t, ) )

def putRobotDown(msg=""):
  if simulation_mode:
    print 'omit putRobotDown'
    return
  f = getActualState().force
  while f[0][2] < 50.0 and f[1][2] < 50.0:
    waitInputConfirm(msg + "Put the robot down.")
    f = getActualState().force

def putRobotUp(msg=""):
  if simulation_mode:
    return True
  f = getActualState().force
  while 30.0 < f[0][2] or 30.0 < f[1][2]:
    waitInputConfirm(msg + "Make sure the Robot in the Air.")
    f = getActualState().force

def servoOn(jname = 'all', destroy = 1):
    if not isCalibDone():
      waitInputConfirm('!! Do CheckEncoders First !!\n\n')
      return -1
 
    if isServoOn():
      return 1
    if jname == '':
        jname = 'all'
    putRobotUp()
    rh_svc.power('all', SwitchStatus.SWITCH_ON)

    try:
      waitInputConfirm(\
          '!! Robot Motion Warning (SERVO ON) !!\n\n' +\
          'Confirm turn RELAY ON.\n' +\
          'Push [OK] to servo ON ('+jname+').')
    except:
      rh_svc.power('all', SwitchStatus.SWITCH_OFF)
      raise
    #if destroy:
    #  self.destroyRTCs(0, [self.Robot])
    #  self.initRTSystem()
    try: 
      stOff()
      goActual()
      time.sleep(0.1)
      rh_svc.servo(jname, SwitchStatus.SWITCH_ON)
      time.sleep(5)
      if not isServoOn(jname):
        print 'servo on failed.'
        raise
    except:
      print "exception occured"

    return 1

def servoOff(jname = 'all'):
    if simulation_mode:
      print "omit servo off"
      return
    if not isServoOn():
      if jname == 'all':
        rh_svc.power('all', SwitchStatus.SWITCH_OFF)   
      return 1
    if jname == '':
      jname = 'all'

    putRobotUp()

    waitInputConfirm(\
      '!! Robot Motion Warning (Servo OFF)!!\n\n' + 
      'Push [OK] to servo OFF ('+jname+').')#:

    try:
      rh_svc.servo('all', SwitchStatus.SWITCH_OFF)   
      if jname == 'all':
        rh_svc.power('all', SwitchStatus.SWITCH_OFF)   
      return 1
    except:
      print 'servo off : communication error'
      return -1

def reboot():
    waitInputConfirm("Reboot the robot host ?")
    adm_svc.reboot("")

def shutdown():
    waitInputConfirm("Shutdown the robot host ?")
    adm_svc.shutdown("")

def overwriteEncoders(jname='all', option='-overwrite'):
    checkEncoders(jname, option)

def checkEncoders(jname='all', option=''):
    if isServoOn():
      waitInput('Servo Off first.')
      return

    #self.destroyRTCs(0, [self.Robot])
    #self.systemInit()

    rh_svc.power('all', SwitchStatus.SWITCH_ON)
    msg = '!! Robot Motion Warning !!\n' +\
          'Turn Relay ON.\n' +\
          'Then Push [OK] to '
    if option == '-overwrite':
      msg = msg + 'calibrate(OVERWRITE MODE) '
    else:
      msg = msg + 'check '

    if jname == 'all':
      msg = msg + 'the Encoders of all.'
    else:
      msg = msg + 'the Encoder of the Joint "'+jname+'".'

    try:
      waitInputConfirm(msg)
    except:
      rh_svc.power('all', SwitchStatus.SWITCH_OFF)
      return 0
    print 'calib-joint ' + jname + ' ' + option
    rh_svc.initializeJointAngle(jname, option)
    print 'done'
    rh_svc.power('all', SwitchStatus.SWITCH_OFF)

def calibSensors():
    servoOn() # destroy user plugins
    goInitial()
    try:
      waitInputConfirm( \
        'Make sure the robot in the Air\n\n' +\
        'Then push [OK] to calibrate FORCE SENSORs.')
      rh_svc.removeForceSensorOffset()
       
      putRobotDown()
      if waitInputSelect('Push [Yes] to calibrate ATTITUDE SENSORs.'):
        countDown(10)
        rh_svc.calibrateInertiaSensor()
    except:
      exceptionType, exceptionValue, exceptionTraceback = sys.exc_info()
      traceback.print_exception(exceptionType, exceptionValue, exceptionTraceback,limit=2, file=sys.stdout)
    servoOff()

def execTestPattern():
    servoOn()
    goHalfSitting()
    stOn()
    putRobotDown()
    try:
      waitInputConfirm(
        '!! Robot Motion Warning !! \n'+\
        'Check ST then, Push [OK] to start stepping(about 8 sec).')
    except: 
      stOff()
      servoOff()
      return 0

    startStep()
    time.sleep(8) # TODO use sleep funtion which can count simulation time
    stopStep()

    stOff()
    #saveLog("/tmp/%s_steptest_%s"%(bodyinfo.modelName.lower(), time.strftime('%Y%m%d%H%M')))
    servoOff()

def execTestPattern2():
    servoOn()
    goHalfSitting()
    stOn()
    putRobotDown()
    try:
      waitInputConfirm(
        '!! Robot Motion Warning !! \n'+\
        'Check ST then, Push [OK] to start walking(1.0m).')
    except: 
      stOff()
      servoOff()
      return 0

   
    stOff()
    #saveLog("/tmp/%s_walktest_%s"%(bodyinfo.modelName.lower(), time.strftime('%Y%m%d%H%M')))
    servoOff()

def userTest():
    goHalfSitting()
    stOn()
    putRobotDown()
    try:
      waitInputConfirm(
        '!! Robot Motion Warning !! \n'+\
        'Check ST then, Push [OK] to start.')
    except: 
      stOff()
      servoOff()
      return 0

    waitInputConfirm('!! Robot Motion Warning !! \n'+\
        'Check ST then, Push [OK] to start.')

    ### ===> edit from here for quick test
    comH = OpenHRP.StateHolderServicePackage.CommandHolder()

    # move 1 axis
    seq_svc.setJointAngle("RARM_JOINT0",   0*math.pi/180.0, 2)
    seq_svc.waitInterpolation()
    seq_svc.setJointAngle("RARM_JOINT0", -10*math.pi/180.0, 2)
    seq_svc.waitInterpolation()

    # move few axis
    sh_svc.getCommand(comH)
    com = comH.value.jointRefs
    #com[bodyinfo.jointId("HEAD_JOINT0")] =  20 * math.pi/180.0
    #com[bodyinfo.jointId("HEAD_JOINT1")] =  20 * math.pi/180.0
    seq_svc.setJointAngles(com, 2)
    seq_svc.waitInterpolation()

    ### <=== edit from here for quick test

    #saveLog("/tmp/hrp_testpat_"+time.strftime('%Y%m%d%H%M'))
"""

def getSTparameter():
    stParam =OpenHRP.StabilizerServicePackage.stParamHolder()
    st_svc.getParameter(stParam)
    return stParam.value
    #print stParam.value.eefm_rot_damping_gain

def setRotGain(x):
    itemlist = x.split()
    numbers = [ float(item) for item in itemlist ]
    stParam=getSTparameter()
    stParam.eefm_rot_damping_gain[0]=numbers[0]
    stParam.eefm_rot_damping_gain[1]=numbers[1]
    stParam.eefm_rot_time_const=numbers[2] 
    st_svc.setParameter(stParam)
    stParam=getSTparameter()
    print stParam.eefm_rot_damping_gain

def setPosGain(x):
    itemlist = x.split()
    numbers = [ float(item) for item in itemlist ]
    stParam=getSTparameter()
    stParam.eefm_pos_damping_gain=numbers[0]
    stParam.eefm_pos_time_const_support=eefm_pos_time_const_swing=numbers[1] 
    st_svc.setParameter(stParam)
    #stParam=getSTparameter()
    #print stParam.eefm_pos_damping_gain

def setAttGain(x):
    itemlist = x.split()
    numbers = [ float(item) for item in itemlist ]
    stParam=getSTparameter()
    stParam.eefm_body_attitude_control_gain[0]=numbers[0] #roll
    stParam.eefm_body_attitude_control_gain[1]=numbers[1] #pitch
    stParam.eefm_body_attitude_control_time_const[0]=numbers[2]
    stParam.eefm_body_attitude_control_time_const[1]=numbers[3] 
    st_svc.setParameter(stParam)
    #stParam=getSTparameter()
    #print stParam.eefm_body_attitude_control_gain[0]," ",stParam.eefm_body_attitude_control_gain[1]

def setconstTime(x):
    itemlist = x.split()
    numbers = [ float(item) for item in itemlist ]
    stParam=getSTparameter()
    stParam.eefm_rot_time_const=numbers[0] 
    stParam.eefm_pos_time_const_support=eefm_pos_time_const_swing=numbers[0] 
    stParam.eefm_body_attitude_control_time_const[0]=numbers[1]
    stParam.eefm_body_attitude_control_time_const[1]=numbers[1] 
    st_svc.setParameter(stParam)
   
def setPosSwitch():
    global posswitch
    posswitch=not posswitch
    #print "pos switch", posswitch
    stParam=getSTparameter()
    stParam.eefm_pos_control_switch=posswitch 
    st_svc.setParameter(stParam)

def setRotSwitch():
    global rotswitch
    rotswitch=not rotswitch
    #print "rot switch", rotswitch
    stParam=getSTparameter()
    stParam.eefm_rot_control_switch=rotswitch 
    st_svc.setParameter(stParam)

def setAttSwitch():
    global attswitch
    attswitch=not attswitch
    #print "att switch", attswitch
    stParam=getSTparameter()
    stParam.eefm_att_control_switch=attswitch 
    st_svc.setParameter(stParam)

##########
#grxuer method

def start():
    user_svc.start()
  
def stOn():
    st_svc.startStabilizer()

def stOff():
    st_svc.stopStabilizer()

def setObjectV(x):
    itemlist = x.split() #import numpy 
    numbers = [ float(item) for item in itemlist ]
    #print "wa= ",numbers[0]+numbers[2]
    user_svc.setObjectV(numbers[0], numbers[1], numbers[2], numbers[3], numbers[4], numbers[5])

def testMove():
    user_svc.testMove()

def sonyStop():
    user_svc.stop()

def omniWalkSwitch():
    user_svc.omniWalkSwitch()

def stepping():
    user_svc.stepping()

def setFootPosR(x):
    itemlist = x.split() #import numpy 
    numbers = [ float(item) for item in itemlist ]
    user_svc.setFootPosR(numbers[0], numbers[1], numbers[2], numbers[3], numbers[4], numbers[5])

def setFootPosL(x):
    itemlist = x.split() #import numpy 
    numbers = [ float(item) for item in itemlist ]
    user_svc.setFootPosL(numbers[0], numbers[1], numbers[2], numbers[3], numbers[4], numbers[5])

##st method
#def st_hogex():
#    st_svc.hogex()


###########
funcList = [
  #"Robot Hardware Setup",
  #checkEncoders,
  #calibSensors,
  #["Step test", execTestPattern],
  #["Walk test", execTestPattern2],
  #"OnOff",
  #servoOn,
  #servoOff,
  #stOn,
  #stOff,
  #"ChangePose",
  #goInitial,
  #goHalfSitting,
  #exPos,
  #"etc:",
  #saveLog,
  #reboot,
  #shutdown,
#  " ",
]
"""
expert_mode = os.getenv("EXPERT_MODE")

if expert_mode and expert_mode.lower() == 'on':
  funcList += \
  [
    " ",
    "For expert",
    overwriteEncoders,
    " ",
    servoOn,
    goHalfSitting,
    stOn,
    startStep,
    stopStep,
    walk05,
    stOff,
    servoOff,
  ]
"""
