TOP = ../..
include $(TOP)/configure/CONFIG
USR_INCLUDES += -I"$(TOP)/../../../libraries/master/boost/include/"

LIBRARY_IOC += cryosms

cryosms_INC += CRYOSMSDriver.h
cryosms_SRCS += CRYOSMSDriver.cpp
cryosms_LIBS += $(EPICS_BASE_IOC_LIBS)
cryosms_LIBS += asyn
cryosms_DBD += drvAsynIPPort.dbd
cryosms_DBD += asyn.dbd

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

