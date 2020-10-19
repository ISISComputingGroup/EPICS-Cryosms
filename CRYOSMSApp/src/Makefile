TOP = ../..
include $(TOP)/configure/CONFIG
USR_INCLUDES += -I"$(TOP)/../../../libraries/master/boost/include/"

LIBRARY_IOC += CRYOSMS

DBD += CRYOSMS.dbd

ifneq ($(findstring windows,$(EPICS_HOST_ARCH)),)
ifeq ($(findstring 10.0,$(VCVERSION)),)
SRC_DIRS += $(TOP)/CRYOSMSApp/src/tests
GTESTPROD_HOST += runner
runner_INC += CRYOSMSDriver.h
runner_LIBS += $(EPICS_BASE_IOC_LIBS) asyn
runner_SRCS += CRYOSMSTests.cc CRYOSMSDriver.cpp
endif
endif

CRYOSMS_DBD += drvAsynIPPort.dbd
CRYOSMS_DBD += asyn.dbd

CRYOSMS_INC += CRYOSMSDriver.h
CRYOSMS_SRCS += CRYOSMSDriver.cpp

CRYOSMS_LIBS += asyn
CRYOSMS_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES

ifdef T_A
install: dllPath.bat

dllPath.bat:
	$(CONVERTRELEASE) -a $(T_A) -T $(IOCS_APPL_TOP) $@
endif
