ament_add_gtest(test_control test_control.cpp)
target_link_libraries(test_control ${PROJECT_NAME})
ament_target_dependencies(test_control ${dependencies})

# ament_add_gtest(test_component test_component.cpp)
# target_link_libraries(test_component ${PROJECT_NAME})
# ament_target_dependencies(test_component ${dependencies})
