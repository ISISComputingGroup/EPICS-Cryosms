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

TESTPROD_HOST += runner
USR_CPPFLAGS += -I$(GTEST)/googletest/include 
runner_SRCS += run_all_tests.cc CRYOSMSTests.cpp
runner_LIBS += gtest

include $(TOP)/configure/RULES

