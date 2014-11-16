import os
import sys
import math
import org.omg.CosNaming
import jp.go.aist.hrp.simulator

modelName ='HRP2A'
url='file:///opt/grx/%s/model/%smain.wrl'%(modelName, modelName)

# setup path
sys.path.append('/usr/local/share/OpenHRP-3.1/java/openhrpstubskel.jar')
EXTRA_JAR_PATH='/opt/grx/HRP2A/share/hrpsys/jar/'
for f in os.listdir(EXTRA_JAR_PATH):
  if f.endswith(".jar") and not sys.path.count(EXTRA_JAR_PATH+f):
    sys.path.insert(0, EXTRA_JAR_PATH+f)
    print 'bodyinfo.py: '+f+' is added.'

timeToHalfsitPose = 3.0 # [sec]
halfsitPose = [0,0,-26,50,-24,0, 
               0,0,-26,50,-24,0,
               0,0,
               0,0,
               15,-10,0,-30, 80,0,0,0,
               15, 10,0,-30,-80,0,0,0]

exPoseOri = [0,0,-26,50,-24,0, 
          0,0,-26,50,-24,0,
          0,0,
          0,0,
          21.7, -18.2, 6.5, -114.4, 75.7, 0, 13.1, 0, 
          21.7, 18.2, -6.5, -114.4,-75.7, 0, 13.1, 0]

exPose = [0,0.1915,-27.7195,50.0245,-22.2165,-0.1895, 
          0,0.1915,-27.7195,50.0245,-22.2165,-0.1895,
          0,0,
          0,0,
          21.7, -18.2, 6.5, -114.4, 75.7, 0, 13.1, 1.72, 
          21.7, 18.2, -6.5, -114.4,-75.7, 0, 13.1, -1.72]

dof = len(halfsitPose)

timeToInitialPose = 3.0 # [sec]
initialPose = [0] * dof

ankleHeight = 0.105     # [m]
legLinkLen = [0.3, 0.3] # [m]
#halfsitWaistHeight = ankleHeight + legLinkLen[0]*math.cos(halfsitPose[2])+legLinkLen[1]*math.cos(halfsitPose[4])
halfsitWaistHeight = ankleHeight + legLinkLen[0]*math.cos(halfsitPose[2]*math.pi/180)+legLinkLen[1]*math.cos(halfsitPose[4]*math.pi/180)
hstConfig = [["paramA", "0.008,0.008,0"],
             ["paramB", "1,1,0"],
             ["paramC", "0.019,0.019,0"],
             ["paramD", "2.0,2.0,0,2.0,2.0,0"],
             ["paramE", "0.4,0.4,0,0.4,0.4,0"],
             ["paramF", "1.3,0.4,1.3,0.4"],
             ["paramG", "0.01"],
             ["paramH", "3.927"],
             ["paramI", "3.927"],
             ["paramJ", "8"],
             ["paramK", "3.927"]]

# this is for compatibility between different robots
def makeCommandPose(pose):
    return [jv*math.pi/180.0 for jv in pose]

# get bodyInfo from modelloader
def init(nameContext):
  global url, linkInfo
  obj = nameContext.resolve([org.omg.CosNaming.NameComponent('ModelLoader', '')])
  mdlldr = jp.go.aist.hrp.simulator.ModelLoaderHelper.narrow(obj)
  bodyInfo = mdlldr.getBodyInfo(url)
  linkInfo = bodyInfo.links()

def jointId(jointName):
  for l in linkInfo:
    if jointName == l.name:
      return l.jointId

def jointName(jointId):
  for l in linkInfo:
    if jointId == jid:
      return l.name
