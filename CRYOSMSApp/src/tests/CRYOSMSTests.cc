#include "gtest/gtest.h"
#include "CRYOSMSDriver.h"
#include <stdlib.h>
#include <string>
#include <tuple>



namespace {
	///setup
	class StartupTests : public ::testing::Test
	{
	public:
		static void SetUpTestCase() {
			testDriver = new CRYOSMSDriver("L0", "TESTING:CRYOSMS_01:", "0.037", "AMPS", "GAUSS", "135", "5", "No", "1", "0.1", "0.0005", "5", "60", 
				"0.5", "No", "NONE", "3.7", "3.65", "10", "0.2", "300", "NONE", "No", "NONE", "5.5", "1", "No", "0", "0", "NONE", "NONE", "NONE");
		}
		virtual void TearDown() {
			testDriver->writeDisabled = 0;
			testDriver->testVar = 0;
		}
		static CRYOSMSDriver* testDriver;
	};

	CRYOSMSDriver* StartupTests::testDriver = NULL;

	///T to A
	TEST_F(StartupTests, test_GIVEN_IOC_WHEN_T_to_A_null_THEN_write_disabled)
	{
		testDriver->envVarMap.at("T_TO_A") = "NULL";
		testDriver->checkTToA();
		ASSERT_EQ(testDriver->writeDisabled, 1);
	}

	class TToAParametrisedTests : public StartupTests, public ::testing::WithParamInterface<std::tuple<const char*, const char*, double>>
	{
	public:
		static void SetUpTestCase() {}
		static void TearDownTestCase() {}
	};
	TEST_P(TToAParametrisedTests, test_GIVEN_IOC_WHEN_T_To_A_checked_THEN_correct_commands_issued)
	{
		testDriver->envVarMap.at("T_TO_A") = "0.2";
		testDriver->envVarMap.at("WRITE_UNIT") = std::get<0>(GetParam());
		testDriver->envVarMap.at("DISPLAY_UNIT") = std::get<1>(GetParam());
		testDriver->checkTToA();
		ASSERT_EQ(testDriver->writeToDispConversion, std::get<2>(GetParam()));
	}

	INSTANTIATE_TEST_CASE_P(
		TToATests, TToAParametrisedTests,
		::testing::Values(
			std::make_tuple("AMPS", "GAUSS", 2000.0), // 10,000 * 0.2
			std::make_tuple("AMPS", "AMPS", 1.0),
			std::make_tuple("AMPS", "TESLA", 0.2), // specified conversion, "T_TO_A" is a dividitave conversion factor, named "TO" per convention
			std::make_tuple("TESLA", "GAUSS", 10000.0), //1T = 10,000G
			std::make_tuple("TESLA", "AMPS", 5.0), // 1/0.2
			std::make_tuple("TESLA", "TESLA", 1.0)
		)
	);

	///Max Curr
	TEST_F(StartupTests, test_GIVEN_IOC_WHEN_max_current_null_THEN_write_disabled)
	{
		testDriver->envVarMap.at("MAX_CURR") = "NULL";
		testDriver->checkMaxCurr();
		ASSERT_EQ(testDriver->writeDisabled, 1);
	}

	TEST_F(StartupTests, test_GIVEN_IOC_WHEN_max_current_not_null_THEN_write_enabled)
	{
		testDriver->envVarMap.at("MAX_CURR") = "10.2";
		testDriver->checkMaxCurr();
		ASSERT_EQ(testDriver->testVar, 1);
	}

	///Allow persist
	class AllowPersistParametrisedTests : public StartupTests, public ::testing::WithParamInterface<std::string>
	{
	public:
		static void SetUpTestCase() {}
		static void TearDownTestCase() {}
	};

	INSTANTIATE_TEST_CASE_P(
		AllowPersistTests, AllowPersistParametrisedTests,
		::testing::Values(
			"FAST_FILTER_VALUE",
			"FILTER_VALUE",
			"NPP",
			"FAST_PERSISTENT_SETTLETIME",
			"PERSISTENT_SETTLETIME",
			"FAST_RATE"));

	TEST_P(AllowPersistParametrisedTests, test_GIVEN_allow_persist_yes_WHEN_a_required_value_null_THEN_write_disabled)
	{
		testDriver->envVarMap.at("ALLOW_PERSIST") = "Yes";
		testDriver->envVarMap.at("FAST_FILTER_VALUE") = "";
		testDriver->envVarMap.at("FILTER_VALUE") = "";
		testDriver->envVarMap.at("NPP") = "";
		testDriver->envVarMap.at("FAST_PERSISTENT_SETTLETIME") = "";
		testDriver->envVarMap.at("PERSISTENT_SETTLETIME") = "";
		testDriver->envVarMap.at("FAST_RATE") = "";
		testDriver->envVarMap.at(GetParam()) = "NULL";
		testDriver->checkAllowPersist();
		ASSERT_EQ(testDriver->writeDisabled, 1);
	}

	TEST_F(StartupTests, test_GIVEN_allow_persist_yes_WHEN_required_values_exist_THEN_write_enabled)
	{
		testDriver->envVarMap.at("ALLOW_PERSIST") = "Yes";
		testDriver->envVarMap.at("FAST_FILTER_VALUE") = "";
		testDriver->envVarMap.at("FILTER_VALUE") = "";
		testDriver->envVarMap.at("NPP") = "";
		testDriver->envVarMap.at("FAST_PERSISTENT_SETTLETIME") = "";
		testDriver->envVarMap.at("PERSISTENT_SETTLETIME") = "";
		testDriver->envVarMap.at("FAST_RATE") = "";
		testDriver->checkAllowPersist();
		ASSERT_EQ(testDriver->testVar, 1);
	}

	TEST_F(StartupTests, test_GIVEN_IOC_WHEN_allow_persist_no_THEN_correct_values_set)
	{
		testDriver->envVarMap.at("ALLOW_PERSIST") = "NO";
		testDriver->checkAllowPersist();
		ASSERT_EQ(testDriver->testVar, 2);
	}

	///use switch
	class UseSwitchParametrisedTests : public StartupTests, public ::testing::WithParamInterface<std::string>
	{
	public:
		static void SetUpTestCase() {}
		static void TearDownTestCase() {}
	};

	INSTANTIATE_TEST_CASE_P(
		UseSwitchTests, UseSwitchParametrisedTests,
		::testing::Values(
			"SWITCH_TEMP_PV",
			"SWITCH_HIGH",
			"SWITCH_LOW",
			"SWITCH_STABLE_NUMBER",
			"HEATER_TOLERANCE",
			"SWITCH_TIMEOUT",
			"SWITCH_TEMP_TOLERANCE",
			"HEATER_OUT"));

	TEST_P(UseSwitchParametrisedTests, test_GIVEN_use_switch_yes_WHEN_a_required_value_null_THEN_write_disabled)
	{
		testDriver->envVarMap.at("USE_SWITCH") = "Yes";
		testDriver->envVarMap.at("SWITCH_TEMP_PV") = "";
		testDriver->envVarMap.at("SWITCH_HIGH") = "";
		testDriver->envVarMap.at("SWITCH_LOW") = "";
		testDriver->envVarMap.at("SWITCH_STABLE_NUMBER") = "";
		testDriver->envVarMap.at("HEATER_TOLERANCE") = "";
		testDriver->envVarMap.at("SWITCH_TIMEOUT") = "";
		testDriver->envVarMap.at("SWITCH_TEMP_TOLERANCE") = "";
		testDriver->envVarMap.at("HEATER_OUT") = "";
		testDriver->envVarMap.at(GetParam()) = "NULL";
		testDriver->checkUseSwitch();
		ASSERT_EQ(testDriver->writeDisabled, 1);
	}

	TEST_F(StartupTests, test_GIVEN_use_switch_yes_WHEN_required_values_exist_THEN_nothing_happens)
	{
		testDriver->envVarMap.at("USE_SWITCH") = "Yes";
		testDriver->envVarMap.at("SWITCH_TEMP_PV") = "";
		testDriver->envVarMap.at("SWITCH_HIGH") = "";
		testDriver->envVarMap.at("SWITCH_LOW") = "";
		testDriver->envVarMap.at("SWITCH_STABLE_NUMBER") = "";
		testDriver->envVarMap.at("HEATER_TOLERANCE") = "";
		testDriver->envVarMap.at("SWITCH_TIMEOUT") = "";
		testDriver->envVarMap.at("SWITCH_TEMP_TOLERANCE") = "";
		testDriver->envVarMap.at("HEATER_OUT") = "";
		testDriver->checkUseSwitch();
		ASSERT_EQ(testDriver->testVar, 1);
	}

	TEST_F(StartupTests, test_GIVEN_IOC_WHEN_use_switch_no_THEN_nothing_happens)
	{
		testDriver->envVarMap.at("USE_SWITCH") = "No";
		testDriver->checkUseSwitch();
		ASSERT_EQ(testDriver->testVar, 1);
	}

	///Use magnet temp
	class UseMagnetTempParametrisedTests : public StartupTests, public ::testing::WithParamInterface<std::string>
	{
	public:
		static void SetUpTestCase() {}
		static void TearDownTestCase() {}
	};

	INSTANTIATE_TEST_CASE_P(
		UseMagnetTempTests, UseMagnetTempParametrisedTests,
		::testing::Values(
			"MAGNET_TEMP_PV",
			"MAX_MAGNET_TEMP",
			"MIN_MAGNET_TEMP"));


	TEST_P(UseMagnetTempParametrisedTests, test_GIVEN_use_magnet_temp_yes_WHEN_a_required_value_null_THEN_write_disabled)
	{
		testDriver->envVarMap.at("USE_MAGNET_TEMP") = "Yes";
		testDriver->envVarMap.at("MAGNET_TEMP_PV") = "";
		testDriver->envVarMap.at("MAX_MAGNET_TEMP") = "";
		testDriver->envVarMap.at("MIN_MAGNET_TEMP") = "";
		testDriver->envVarMap.at(GetParam()) = "NULL";
		testDriver->checkUseMagnetTemp();
		ASSERT_EQ(testDriver->writeDisabled, 1);
	}

	TEST_F(StartupTests, test_GIVEN_use_magnet_temp_yes_WHEN_required_values_exist_THEN_nothing_happens)
	{
		testDriver->envVarMap.at("USE_MAGNET_TEMP") = "Yes";
		testDriver->envVarMap.at("MAGNET_TEMP_PV") = "";
		testDriver->envVarMap.at("MAX_MAGNET_TEMP") = "";
		testDriver->envVarMap.at("MIN_MAGNET_TEMP") = "";
		testDriver->checkUseMagnetTemp();
		ASSERT_EQ(testDriver->testVar, 1);
	}
	TEST_F(StartupTests, test_GIVEN_IOC_WHEN_use_magnet_temp_not_yes_THEN_nothing_happens)
	{
		testDriver->envVarMap.at("USE_MAGNET_TEMP") = "No";
		testDriver->checkUseMagnetTemp();
		ASSERT_EQ(testDriver->testVar, 1);
	}

	///COMP OFF ACT
	class CompOffActParametrisedTests : public StartupTests, public ::testing::WithParamInterface<std::string>
	{
	public:
		static void SetUpTestCase() {}
		static void TearDownTestCase() {}
	};

	INSTANTIATE_TEST_CASE_P(
		CompOffActTests, CompOffActParametrisedTests,
		::testing::Values(
			"NO_OF_COMP",
			"MIN_NO_OF_COMP_ON",
			"COMP_1_STAT_PV",
			"COMP_2_STAT_PV"));

	TEST_P(CompOffActParametrisedTests, test_GIVEN_comp_off_act_yes_WHEN_a_required_value_null_THEN_write_disabled)
	{
		testDriver->envVarMap.at("COMP_OFF_ACT") = "Yes";
		testDriver->envVarMap.at("NO_OF_COMP") = "";
		testDriver->envVarMap.at("MIN_NO_OF_COMP_ON") = "";
		testDriver->envVarMap.at("COMP_1_STAT_PV") = "";
		testDriver->envVarMap.at("COMP_2_STAT_PV") = "";
		testDriver->envVarMap.at(GetParam()) = "NULL";
		testDriver->checkCompOffAct();
		ASSERT_EQ(testDriver->writeDisabled, 1);
	}

	TEST_F(StartupTests, test_GIVEN_comp_off_act_yes_WHEN_required_values_exist_THEN_write_enabled)
	{
		testDriver->envVarMap.at("COMP_OFF_ACT") = "Yes";
		testDriver->envVarMap.at("NO_OF_COMP") = "";
		testDriver->envVarMap.at("MIN_NO_OF_COMP_ON") = "";
		testDriver->envVarMap.at("COMP_1_STAT_PV") = "";
		testDriver->envVarMap.at("COMP_2_STAT_PV") = "";
		testDriver->checkCompOffAct();
		ASSERT_EQ(testDriver->testVar, 1);
	}

	TEST_F(StartupTests, test_GIVEN_IOC_WHEN_comp_off_act_not_yes_THEN_write_enabled)
	{
		testDriver->envVarMap.at("COMP_OFF_ACT") = "No";
		testDriver->checkCompOffAct();
		ASSERT_EQ(testDriver->testVar, 1);
	}

	///ramp file
	TEST_F(StartupTests, test_GIVEN_IOC_WHEN_ramp_file_null_THEN_writes_disabled)
	{
		testDriver->envVarMap.at("RAMP_FILE") = "NULL";
		testDriver->checkRampFile();
		ASSERT_EQ(testDriver->writeDisabled, 1);
	}

	TEST_F(StartupTests, test_GIVEN_IOC_WHEN_ramp_file_invalid_THEN_writes_disabled)
	{
		testDriver->envVarMap.at("RAMP_FILE") = "";
		testDriver->checkRampFile();
		ASSERT_EQ(testDriver->writeDisabled, 1);
	}
	TEST_F(StartupTests, test_GIVEN_IOC_WHEN_ramp_file_valid_THEN_writes_enabled)
	{
		testDriver->envVarMap.at("RAMP_FILE") = "C:\\Instrument\\Apps\\EPICS\\support\\cryosms\\master\\ramps\\test.txt";
		testDriver->checkRampFile();
		ASSERT_EQ(testDriver->testVar, 1);
	}

	///max volt
	TEST_F(StartupTests, test_GIVEN_IOC_WHEN_max_volt_null_THEN_correct_code_reached)
	{
		testDriver->envVarMap.at("MAX_VOLT") = "NULL";
		testDriver->checkMaxVolt();
		ASSERT_EQ(testDriver->testVar, 1);
	}
	TEST_F(StartupTests, test_GIVEN_IOC_WHEN_max_volt_invalid_THEN_correct_code_reached)
	{
		testDriver->envVarMap.at("MAX_VOLT") = "";
		testDriver->checkMaxVolt();
		ASSERT_EQ(testDriver->testVar, 1);
	}
	TEST_F(StartupTests, test_GIVEN_IOC_WHEN_max_volt_valid_THEN_correct_code_reached)
	{
		testDriver->envVarMap.at("MAX_VOLT") = "1";
		testDriver->checkMaxVolt();
		ASSERT_EQ(testDriver->testVar, 2);
	}

	///Write unit
	TEST_F(StartupTests, test_GIVEN_IOC_WHEN_write_unit_set_to_amps_THEN_communicate_in_amps)
	{
		testDriver->envVarMap.at("WRITE_UNIT") = "AMPS";
		testDriver->checkWriteUnit();
		ASSERT_EQ(testDriver->testVar, 1);
	}
	TEST_F(StartupTests, test_GIVEN_IOC_WHEN_write_unit_no_set_to_amps_THEN_default_to_communicating_in_tesla)
	{
		testDriver->envVarMap.at("WRITE_UNIT") = "";
		testDriver->checkWriteUnit();
		ASSERT_EQ(testDriver->testVar, 2);
	}
}
