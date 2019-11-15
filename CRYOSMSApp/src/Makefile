TOP = ../..
include $(TOP)/configure/CONFIG

LIBRARY_IOC += CRYOSMS

DBD += CRYOSMS.dbd

CRYOSMS_DBD += drvAsynIPPort.dbd
CRYOSMS_DBD += asyn.dbd

CRYOSMS_INC += CRYOSMSDriver.h
CRYOSMS_SRCS += CRYOSMSDriver.cpp

CRYOSMS_LIBS += asyn
CRYOSMS_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
