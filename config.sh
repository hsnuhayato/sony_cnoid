#!/bin/sh
export COMP_NAME="sony"
#export RTM_ROOT="/opt/grx"
#export PATH=/usr/pkg/bin:/opt/grx/bin:$PATH
export PATH=/usr/pkg/bin:$PATH
#export PKG_CONFIG_PATH=/opt/grx/lib/pkgconfig:/usr/pkg/lib/pkgconfig
export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig
#export LD_LIBRARY_PATH=/usr/pkg/lib:/opt/grx/lib:/opt/grx/lib/OpenRTM-aist
#export LD_LIBRARY_PATH=/usr/pkg/lib:/usr/local/lib:/usr/lib/OpenRTM-aist
export LD_LIBRARY_PATH=//usr/local/lib:/usr/lib/openrtm-1.1
#export LD_LIBRARY_PATH=/usr/pkg/lib:/usr/local/lib:/opt/grx/lib/OpenRTM-aist