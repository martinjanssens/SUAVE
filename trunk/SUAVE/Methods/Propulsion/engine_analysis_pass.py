
""" Propulsion.py: Methods for Propulsion Analysis """

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

import numpy as np
from SUAVE.Core import Data
from SUAVE.Attributes import Constants
# import SUAVE

# ----------------------------------------------------------------------
#  Mission Methods
# ----------------------------------------------------------------------


def engine_analysis_pass(Turbofan,State):    
    
    
    mach=State.M
    a=State.alt    
    
    #mach=Segment.mach
    #a=Segment.alt
    sfc_sfcref=Turbofan.sfc_sfcref
    sls_thrust=Turbofan.thrust_sls
    eng_type=Turbofan.type


    if eng_type==1:
        sfc= sfc_sfcref*(0.335780271274146 +  0.029873325206606*mach +  0.081209003474378*a- 0.821844062318631*mach**2 - 0.012313371529160*a*mach- 0.013980789615436*a**2 + 1.941911084998007*mach**3 - 0.058052828837187*mach**2*a +  0.004256576580281*mach*a**2 +  0.001331152570038*a**3 - 1.201666666666645*mach**4 + 0.044941876252323*mach**3*a + 0.000021022540684*mach**2*a**2 - 0.000294083634720*mach*a**3 - 0.000032451442616*a**4)
        th=sls_thrust*(0.868009508296048  -0.166557104971858*mach  -0.144651575623525*a  -1.407030667976488*mach**2  + 0.093905630521542*mach*a +  0.027666192971266*a**2 +  0.853125587544063*mach**3 +  0.118638337242548*mach**2*a  -0.019855919345256*mach*a**2  -0.002651612710499*a**3  -0.112812499999986*mach**4  -0.052144616158441*mach**3*a -0.000980638640291*mach**2*a**2   +0.001005413843178*mach*a**3 +  0.000094582673830*a**4)

    if eng_type==2:
        sfc= sfc_sfcref*(0.607700000000000  -0.000000000000003*mach+   0.054425447119591*a +  0.000000000000018*mach**2  -0.000000000000000*mach*a  -0.009605314839191*a**2  -0.000000000000037*mach**3  -0.000000000000000*mach**2*a +  0.000000000000000*mach*a**2 +  0.000949737253235*a**3+   0.000000000000024*mach**4  -0.000000000000000*mach**3*a  -0.000000000000000*mach**2*a**2  -0.000000000000000*mach*a**3  -0.000025397493590*a**4)
        th=sls_thrust*(0.804113755084826  -0.035627902201493*mach  -0.096184884986570*a  -2.779712131707290*mach**2  + 0.095737606320651*mach*a +  0.016854969440649*a**2 +  3.691579501909484*mach**3+   0.153160834493603*mach**2*a -0.018080377409672*mach*a**2  -0.001507189240658*a**3  -1.577283906249969*mach**4  -0.153161698513016*mach**3*a+   0.002018571573980*mach**2*a**2+   0.000869874382566*mach*a**3+   0.000048066992847*a**4)

    if eng_type==3:
        sfc= sfc_sfcref*(0.285436237761663 +  0.024502655477171*mach +  0.068999277904472*a  -0.697385662078613*mach**2 -0.010544773458370*mach*a  -0.011884083398418*a**2 +  1.653048305914588*mach**3  -0.049001864268300*mach**2*a +   0.003616272596893*mach*a**2 +  0.001131761236922*a**3 -1.024651041666653*mach**4 +  0.037773643074563*mach**3*a +   0.000062961864908*mach**2*a**2  -0.000252876560383*mach*a**3  -0.000027565941442*a**4)
        th=sls_thrust*(0.860753545648466  -0.254090878749431*mach  -0.108037058286478*a  -1.010034245425557*mach**2+  0.069717277528921*mach*a +0.011645535013526*a**2+  0.423541666666671*mach**3 + 0.047130921483669*mach**2*a +  -0.006331867045162*mach*a**2  -0.000460002484144*a**3)

    if eng_type==5:
        sfc= sfc_sfcref*(0)
        th=sls_thrust*(0.860707560213307  -0.251373072375285*mach  -0.107900030324592*a  -1.019215393654531*mach**2+  0.069827313560592*mach*a +  0.011615366286529*a**2+  0.430875000000004*mach**3 +  0.046861671760037*mach**2*a -0.006319498957339*mach*a**2  -0.000458427089846*a**3)
 

    if eng_type==6:
        sfc= sfc_sfcref*(1.039279928346275+ 0.154354343003498*mach+ 0.117486771130496*a -0.265820962970648*mach**2 + 0.011679904232747*mach*a  -0.065853428512314*a**2 +  0.112879463923913*mach**3  -0.005232011798524*mach**2*a  -0.000864238935027*mach*a**2 +  0.009811438002540*a**3  -0.015448922821971*mach**4  + 0.000896085056928*mach**3*a +  0.000148921068721*mach**2*a**2  -0.000014581672329*a**3*mach  -0.000387827542944*mach**4)
        th=sls_thrust*(0.871003609000932  -0.503449519567159*mach + 0.245774798277167*a + 0.038477468960137*mach**2  -0.139342839995028*mach*a +  0.000585397434489*a**2 +  0.023113033234127*mach**3  -0.000098763071611*mach**2*a+  0.011975525050602*mach*a**2  -0.001938941125545*a**3)


#-------------put in engine geometry specifications----

    Turbofan.sfc=sfc
    Turbofan.thrust=th


    return Turbofan
