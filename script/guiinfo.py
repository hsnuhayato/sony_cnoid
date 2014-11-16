import os
from sample import *

funcList = [
  "Robot Hardware Setup",
  checkEncoders,
  calibSensors,
  ["Step test", execTestPattern],
  ["Walk test", execTestPattern2],
  " ",
  "OnOff",
  servoOn,
  servoOff,
  " ",
  "ChangePose",
  goInitial,
  goHalfSitting,
  " ",
  "etc:",
  saveLog,
  reboot,
  shutdown,
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
