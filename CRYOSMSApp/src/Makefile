TOP = ../..
include $(TOP)/configure/CONFIG

LIBRARY_IOC += cryosms

cryosms_INC += CRYOSMSDriver.h
cryosms_SRCS += CRYOSMSDriver.cpp
cryosms_LIBS += $(EPICS_BASE_IOC_LIBS)
cryosms_LIBS += asyn
cryosms_DBD += drvAsynIPPort.dbd
cryosms_DBD += asyn.dbd

DBD += CRYOSMS.dbd

include $(TOP)/configure/RULES

