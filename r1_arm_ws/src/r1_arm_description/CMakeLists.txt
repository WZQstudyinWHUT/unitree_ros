cmake_minimum_required(VERSION 3.8)
project(r1_arm_description)

find_package(ament_cmake REQUIRED)

# 构建期由 xacro 生成扁平 URDF，供 check_urdf、MuJoCo 导入等下游直接使用
find_program(XACRO_EXECUTABLE xacro REQUIRED)
set(GENERATED_URDF ${CMAKE_CURRENT_BINARY_DIR}/r1_arm.urdf)
add_custom_command(
  OUTPUT ${GENERATED_URDF}
  COMMAND ${XACRO_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/urdf/r1_arm.urdf.xacro -o ${GENERATED_URDF}
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/urdf/r1_arm.urdf.xacro
  COMMENT "Generating r1_arm.urdf from xacro"
)
add_custom_target(generate_urdf ALL DEPENDS ${GENERATED_URDF})

install(DIRECTORY meshes config urdf upstream
  DESTINATION share/${PROJECT_NAME}
)
install(FILES ${GENERATED_URDF}
  DESTINATION share/${PROJECT_NAME}/urdf
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest QUIET)
  if(ament_cmake_pytest_FOUND)
    ament_add_pytest_test(test_fk test/test_fk.py
      ENV R1_ARM_URDF=${GENERATED_URDF}
          R1_ARM_UPSTREAM_URDF=${CMAKE_CURRENT_SOURCE_DIR}/upstream/R1_Robotic_Arm.urdf
    )
    ament_add_pytest_test(test_joint_limits test/test_joint_limits.py
      ENV R1_ARM_URDF=${GENERATED_URDF}
          R1_ARM_JOINT_LIMITS=${CMAKE_CURRENT_SOURCE_DIR}/config/joint_limits.yaml
    )
  endif()
endif()

ament_package()
