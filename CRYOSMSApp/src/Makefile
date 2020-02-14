TOP = ../..
include $(TOP)/configure/CONFIG

LIBRARY_IOC += CRYOSMS

DBD += CRYOSMS.dbd

ifneq ($(findstring windows,$(EPICS_HOST_ARCH)),)
ifeq ($(findstring 10.0,$(VCVERSION)),)
SRC_DIRS += $(TOP)/CRYOSMSApp/src/tests
TESTPROD_HOST += runner
USR_CPPFLAGS += -I$(GTEST)/googletest/include 
runner_INC += CRYOSMSDriver.h
runner_LIBS += gtest $(EPICS_BASE_IOC_LIBS) asyn
runner_SRCS += run_all_tests.cc CRYOSMSTests.cc CRYOSMSDriver.cpp
endif
endif

include $(TOP)/configure/RULES
CRYOSMS_DBD += drvAsynIPPort.dbd
CRYOSMS_DBD += asyn.dbd

CRYOSMS_INC += CRYOSMSDriver.h
CRYOSMS_SRCS += CRYOSMSDriver.cpp

CRYOSMS_LIBS += asyn
CRYOSMS_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
