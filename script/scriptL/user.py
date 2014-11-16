#!/opt/grx/bin/hrpsyspy
import os

import sys
import socket
import traceback
import math
import time
import java.lang.System

import rtm
import waitInput
import bodyinfo
import OpenHRP
from OpenHRP.RobotHardwareServicePackage import SwitchStatus

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

    print "initialize bodyinfo"
    bodyinfo.init(rtm.rootnc)

    print "initialized successfully"

def activateComps(rtcList):
    rtm.serializeComponents(rtcList)
    for r in rtcList:
        r.start()

def initRTC(module, name):
    ms.load(module)
    return ms.create(module, name)

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

def stOn():
    st.setProperty("isEnabled", "1")

def stOff():
    st.setProperty("isEnabled", "0")

def walk05():
    kpg_svc.setTargetPos(0.449, 0.0, 0.0)

def startStep():
    kpg_svc.startStepping()

def stopStep():
    kpg_svc.stopStepping()

def createComps(hostname=socket.gethostname()):
    global ms, adm_svc, rh, rh_svc, seq, seq_svc, sh, sh_svc, servo, kpg, kpg_svc, st, kf, log, simulation_mode
    global sensor
    ms = rtm.findRTCmanager(hostname)
    rh = rtm.findRTC("RobotHardware0")
    if rh != None:
        simulation_mode = 0
        rh_svc = OpenHRP.RobotHardwareServiceHelper.narrow(rh.service("service0"))
        servo = rh
        adm = rtm.findRTC("SystemAdmin0")
        if adm != None:
          adm.start()
          adm_svc = OpenHRP.SystemAdminServiceHelper.narrow(adm.service("service0"))
    else:
        simulation_mode = 1
        rh = rtm.findRTC(bodyinfo.modelName+"Controller(Robot)0")
        servo = rtm.findRTC("PDservo0")
    seq = initRTC("SequencePlayer", "seq")
    seq_svc = OpenHRP.SequencePlayerServiceHelper.narrow(seq.service("service0"))
    sh  = initRTC("StateHolder", "StateHolder0")
    sh_svc = OpenHRP.StateHolderServiceHelper.narrow(sh.service("service0"))

    kf  = initRTC("KalmanFilter", "kf")
    kpg = initRTC("WalkGenerator", "kpg")
    kpg_svc = OpenHRP.WalkGeneratorServiceHelper.narrow(kpg.service("service0"))
    st  = initRTC("Stabilizer", "st")
    st.setConfiguration(bodyinfo.hstConfig)

    log = initRTC("DataLogger", "log")
#    user = initRTC("Nolan", "user")
    
#    if user==None:
#        print "no component"
#        return
#    user_svc = OpenHRP.NolanServiceHelper.narrow(user.service("service0"))
# 
    if simulation_mode:
        sensor = rtm.findRTC(bodyinfo.modelName+"Controller(Robot)0")
        
    return [rh, seq, kf, kpg, sh, st, log]#, user]
    
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
    #
    rtm.connectPorts(kf.port("rpy"), st.port("rpy"))
    #
    rtm.connectPorts(seq.port("qRef"),    sh.port("qIn"))
    rtm.connectPorts(seq.port("basePos"), sh.port("basePosIn"))
    rtm.connectPorts(seq.port("baseRpy"), sh.port("baseRpyIn"))
    rtm.connectPorts(seq.port("accRef"),  kf.port("accRef"))
    rtm.connectPorts(seq.port("zmpRef"),  st.port("zmpRef"))
    #
    rtm.connectPorts(kpg.port("qRef"),   sh.port("qIn"))
    rtm.connectPorts(kpg.port("pRef"),   sh.port("basePosIn"))
    rtm.connectPorts(kpg.port("rpyRef"), sh.port("baseRpyIn"))
    rtm.connectPorts(kpg.port("accRef"), kf.port("accRef"))
    rtm.connectPorts(kpg.port("zmpRef"), st.port("zmpRef"))
    #
    rtm.connectPorts(sh.port("qOut"),       [st.port("qRefIn"),
                                         seq.port("qInit"),
                                         kpg.port("qInit")])
    rtm.connectPorts(sh.port("basePosOut"), [seq.port("basePosInit"),
                                         kpg.port("pInit")])
    rtm.connectPorts(sh.port("baseRpyOut"), [st.port("rpyRef"),
                                         seq.port("baseRpyInit"),
                                         kpg.port("rpyInit")])
    #
    if servo: # simulation_mode with dynamics or real robot mode
        rtm.connectPorts(st.port("qRefOut"), servo.port("qRef"))
    else:     # simulation mode without dynamics
        rtm.connectPorts(sh.port("qOut"), rh.port("qIn"))
    #
"""
##user's port
    rtm.connectPorts(rh.port("rhsensor"), user.port("rhsensor"))
    rtm.connectPorts(rh.port("lhsensor"), user.port("lhsensor"))
    rtm.connectPorts(rh.port("rfsensor"), user.port("rfsensor"))
    rtm.connectPorts(rh.port("lfsensor"), user.port("lfsensor"))
#    rtm.connectPorts(user.port("rzmp"),  st.port("zmpRef"))
    rtm.connectPorts(rh.port("q"), user.port("q"))
    rtm.connectPorts(seq.port("SequencePlayerService"), user.port("ToSequencePlayerService"))
"""

def setupLogger():
    global log_svc
    log_svc = OpenHRP.DataLoggerServiceHelper.narrow(log.service("service0"))
    log_svc.add("TimedDoubleSeq", "q")
    log_svc.add("TimedDoubleSeq", "qRef")
    log_svc.add("TimedDoubleSeq", "zmpRefSeq")
    log_svc.add("TimedDoubleSeq", "zmpRefKpg")
    log_svc.add("TimedDoubleSeq", "basePos")
    log_svc.add("TimedDoubleSeq", "rpy")
    rtm.connectPorts(rh.port("q"), log.port("q"))
    rtm.connectPorts(st.port("qRefOut"), log.port("qRef"))
    rtm.connectPorts(seq.port("zmpRef"), log.port("zmpRefSeq"))
    rtm.connectPorts(kpg.port("zmpRef"), log.port("zmpRefKpg"))
    rtm.connectPorts(sh.port("basePosOut"), log.port("basePos"))
    rtm.connectPorts(kf.port("rpy"), log.port("rpy"))

def saveLog(fname='sample'):
    if log_svc == None:
      waitInputConfirm("Setup DataLogger RTC first.")
      return
    log_svc.save(fname)
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
    saveLog("/tmp/%s_steptest_%s"%(bodyinfo.modelName.lower(), time.strftime('%Y%m%d%H%M')))
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

    kpg_svc.setTargetPos(1.0, 0.0, 0.0)

    stOff()
    saveLog("/tmp/%s_walktest_%s"%(bodyinfo.modelName.lower(), time.strftime('%Y%m%d%H%M')))
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

    kpg_svc.setTargetPos(1.0,0,0)
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
    com[bodyinfo.jointId("HEAD_JOINT0")] =  20 * math.pi/180.0
    com[bodyinfo.jointId("HEAD_JOINT1")] =  20 * math.pi/180.0
    seq_svc.setJointAngles(com, 2)
    seq_svc.waitInterpolation()

    ### <=== edit from here for quick test

    saveLog("/tmp/hrp_testpat_"+time.strftime('%Y%m%d%H%M'))
##########
#grxuer method

#def start():
#    user_svc.start()





###########
#for LabExhibition
def wuGo():

    goHalfSitting()
    stOn()
    waitInputConfirm( \
        '!! Robot Motion Warning !!\n\n' +\
            'Push [OK] to step')
    startStep()
    time.sleep(8) # TODO use sleep funtion which can count simulation time

    kpg_svc.setWalkingVelocity(0.5,0,0)
    time.sleep(6)
    kpg_svc.setWalkingVelocity(0.3,0.3,0)
    time.sleep(6)
    kpg_svc.setWalkingVelocity(-0.3,0,0)
    time.sleep(8)
    kpg_svc.setWalkingVelocity(-0.3,-0.3,0)
    time.sleep(8)
    kpg_svc.setWalkingVelocity(0.3,-0.3,0)
    time.sleep(8)
    kpg_svc.setWalkingVelocity(0,0.3,0)
    time.sleep(8)
    stopStep()

    furiGo()
##

 

def furiGo():
#    goHalfSitting()
#    stOn()

    ### ===> edit from here for quick test
    comH = OpenHRP.StateHolderServicePackage.CommandHolder()
    # move few axis
    sh_svc.getCommand(comH)
    com = comH.value.jointRefs
    """
    com[bodyinfo.jointId("RARM_JOINT0")] =  -60 * math.pi/180.0
    com[bodyinfo.jointId("RARM_JOINT3")] =  -100 * math.pi/180.0
    seq_svc.setJointAngles(com, 4)
    seq_svc.waitInterpolation()
    seq_svc.setJointAngle("HEAD_JOINT0", -35*math.pi/180.0, 2)
    seq_svc.waitInterpolation()
    seq_svc.setJointAngle("HEAD_JOINT0", 35*math.pi/180.0, 3)
    seq_svc.waitInterpolation()

    com[bodyinfo.jointId("HEAD_JOINT0")] =  0* math.pi/180.0
    com[bodyinfo.jointId("RARM_JOINT0")] =  15* math.pi/180.0
    com[bodyinfo.jointId("RARM_JOINT3")] =  -30* math.pi/180.0
    seq_svc.setJointAngles(com, 3)
    seq_svc.waitInterpolation()
    """
#    setJointAnglesDeg(bodyinfo.initialPose, 4)
    stOn()
    # stOff()
    com[bodyinfo.jointId("CHEST_JOINT1")] =  10* math.pi/180.0
    com[bodyinfo.jointId("RARM_JOINT3")] =   -35* math.pi/180.0
    com[bodyinfo.jointId("LARM_JOINT3")] =   -35* math.pi/180.0
    com[bodyinfo.jointId("HEAD_JOINT1")] =   10* math.pi/180.0
    seq_svc.setJointAngles(com, 3)
    seq_svc.waitInterpolation()
    time.sleep(2)
    com[bodyinfo.jointId("CHEST_JOINT1")] =  0* math.pi/180.0
    com[bodyinfo.jointId("RARM_JOINT3")] =  -30* math.pi/180.0
    com[bodyinfo.jointId("LARM_JOINT3")] =  -30* math.pi/180.0
    com[bodyinfo.jointId("HEAD_JOINT1")] =   0* math.pi/180.0
    seq_svc.setJointAngles(com, 3)
    seq_svc.waitInterpolation()

def setV(x):
    #user_svc.setGain(1.0, 0.0000001)
    #print "wa= ",float(x)+4
    itemlist = x.split() #import numpy 
    numbers = [ float(item) for item in itemlist ]
    #print "wa= ",numbers[0]+numbers[1]
    print "Vin= ",numbers    
    kpg_svc.setWalkingVelocity(numbers[0],numbers[1],numbers[2])

##########
funcList = [
  "Robot Hardware Setup",
  checkEncoders,
  calibSensors,
  ["Step test", execTestPattern],
  ["Walk test", execTestPattern2],
  "OnOff",
  servoOn,
  servoOff,
  "ChangePose",
  goInitial,
  goHalfSitting,
  "etc:",
  saveLog,
  reboot,
  shutdown,
#  " ",
]

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
