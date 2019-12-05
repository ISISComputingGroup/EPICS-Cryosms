#include "gtest/gtest.h"
#include "CRYOSMSDriver.h"
#include <stdlib.h>
#include <string>
#include <tuple>
#include <Windows.h>



namespace {
	class StartupTests : public ::testing::Test
	{
	public:
		static void SetUpTestCase() {
			testDriver = new CRYOSMSDriver("L0", "TESTING:CRYOSMS_01:");
		}
		static void TearDownTestCaes() {
			delete testDriver;
			testDriver = NULL;
		}
		static CRYOSMSDriver* testDriver;
	};

	class TToAParametrisedTests : public StartupTests, public ::testing::WithParamInterface<std::tuple<const char*, const char*, double>>
	{
	};

	CRYOSMSDriver* StartupTests::testDriver = NULL;

	TEST_P(TToAParametrisedTests, test_GIVEN_IOC_WHEN_T_To_A_checked_THEN_correct_commands_issued) 
	{
		testDriver->envVarMap["T_TO_A"] = "0.2";
		testDriver->envVarMap["WRITE_UNIT"] = std::get<0>(GetParam());
		testDriver->envVarMap["DISPLAY_UNIT"] = std::get<1>(GetParam());
		testDriver->checkTToA();
		ASSERT_EQ(testDriver->writeToDispConversion, std::get<2>(GetParam()));
	}


	INSTANTIATE_TEST_CASE_P(
		TToATests, TToAParametrisedTests,
		::testing::Values(
			std::make_tuple("AMPS", "GAUSS", 50000.0), // 10,000/0.2
			std::make_tuple("AMPS", "AMPS", 1.0),
			std::make_tuple("AMPS", "TESLA", 5.0), // 1/0.2
			std::make_tuple("TESLA", "GAUSS", 10000.0), //1T = 10,000G
			std::make_tuple("TESLA", "AMPS", 0.2), //specified conversion
			std::make_tuple("TESLA", "TESLA", 1.0)
		)
	);

}