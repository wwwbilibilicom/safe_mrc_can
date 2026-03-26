find_package(GTest REQUIRED)

function(safemrc_add_test name)
  add_executable(${name} ${name}.cpp)
  target_link_libraries(${name} safemrc_can GTest::gtest_main)
  add_test(NAME ${name} COMMAND ${name})
endfunction()

safemrc_add_test(test_mrc_protocol)
safemrc_add_test(test_mrc_state)
safemrc_add_test(test_can_device_collection)
safemrc_add_test(test_integration)
