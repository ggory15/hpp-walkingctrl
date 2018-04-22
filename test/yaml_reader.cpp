#include <hpp/walkingctrl/interface/Setting.hpp>

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MAIN
#define BOOST_TEST_MODULE test_reader

#include <boost/format.hpp>
#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>


static const bool display_time_info = true;

using namespace hpp::walkingctrl;

void testReader(){
    std::string cfg_file = TEST_PATH + std::string("example_setting.yaml");
    std::cout << "Path :" << " " << cfg_file << std::endl; 
    InterfaceSetting interface_setting;
    interface_setting.initialize(cfg_file);

    std::cout<< "dt :" << " " << interface_setting.get(InterfaceDoubleParam_dt) << std::endl;
    std::cout<< "duration :" << " " << interface_setting.get(InterfaceIntParam_MAX_TEST_DURATION) << std::endl;
    std::cout<< "enable_capture_point_limit :" << " " << interface_setting.get(InterfaceBoolParam_ENABLE_CAPTURE_POINT_LIMITS) << std::endl;
    std::cout<< "constraint :" << " " << interface_setting.get(InterfaceIntVectorParam_constraint_mask) << std::endl;
    std::cout<< "q0 :" << " " << interface_setting.get(InterfaceVectorParam_q0) << std::endl;
}

 BOOST_AUTO_TEST_SUITE (test_read)
 BOOST_AUTO_TEST_CASE(test_read_yaml) {
    testReader();
 }
 BOOST_AUTO_TEST_SUITE_END()