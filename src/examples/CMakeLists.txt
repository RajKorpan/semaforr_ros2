cmake_minimum_required(VERSION 3.16)
project(semaforr_examples VERSION 0.1.0)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY core
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    semaforr_examples_contract_test
    test/test_examples_contract.py
    ENV "SEMAFORR_EXAMPLES_SOURCE_DIR=${PROJECT_SOURCE_DIR}"
  )
endif()

ament_package()
