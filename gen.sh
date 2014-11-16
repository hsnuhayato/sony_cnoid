#!/bin/sh
. ./config.sh

#mkdir -p temp
#cp ${COMP_NAME}Service.idl temp
#cp SequencePlayerService.idl temp
#cd temp
/opt/grx/bin/rtc-template -bcxx \
    --module-name=${COMP_NAME} \
    --module-type='sonyComponent' \
    --module-desc='sonyComponent' \
    --module-version=1.0 \
    --module-vendor='tohoku' \
    --module-category=example \
    --module-comp-type=DataFlowComponent \
    --module-act-type=SPORADIC \
    --module-max-inst=1 \
    --inport=q:TimedDoubleSeq \
    --inport=rhsensor:TimedDoubleSeq \
    --inport=lhsensor:TimedDoubleSeq \
    --inport=rfsensor:TimedDoubleSeq \
    --inport=lfsensor:TimedDoubleSeq \
    --inport=mc:TimedDoubleSeq \
    --outport=rzmp:TimedDoubleSeq \
    --outport=refq:TimedDoubleSeq \
    --service=${COMP_NAME}Service:service0:${COMP_NAME}Service \
    --service-idl=${COMP_NAME}Service.idl \
   #--consumer=ToSequencePlayerService:serviceSeq0:SequencePlayerService \
   #--consumer-idl=SequencePlayerService.idl
rm *_vc*
rm *.bat
rm user_config.vsprops
mv ${COMP_NAME}ServiceSVC_impl.h ${COMP_NAME}Service_impl.h
mv ${COMP_NAME}ServiceSVC_impl.cpp ${COMP_NAME}Service_impl.cpp
rm Makefile.${COMP_NAME}

removeSVC() {
    sed -e "s/ServiceSVC/Service/g" $1 > /tmp/.$1
    sed -e "s/SERVICESVC/SERVICE/g" /tmp/.$1 > $1
}

removeSVC ${COMP_NAME}.h
removeSVC ${COMP_NAME}Service_impl.h
removeSVC ${COMP_NAME}Service_impl.cpp

removeOpenHRP() {
    sed -e "s/OpenHRP_//g" $1 > /tmp/.$1
    sed -e "s/${COMP_NAME}ServiceSkel.h/${COMP_NAME}Service.hh/g" /tmp/.$1 > $1
}
removeOpenHRP ${COMP_NAME}Service_impl.h
removeOpenHRP ${COMP_NAME}Service_impl.cpp

#sed '16 a\#include "JointDataTypesStub.h"' ${COMP_NAME}.h > .${COMP_NAME}.h
#mv .${COMP_NAME}.h ${COMP_NAME}.h
