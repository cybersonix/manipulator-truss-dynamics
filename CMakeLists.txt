cmake_minimum_required(VERSION 3.10)
project(ManipulatorTrussDynamics VERSION 1.0.0 LANGUAGES CXX)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置构建类型
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# 添加自定义安装路径
list(APPEND CMAKE_PREFIX_PATH
    "$ENV{HOME}/urdf_install"
    "$ENV{HOME}/vsgFramework/install"
    "/usr/local/chrono"
)

# 查找 Chrono 库
find_package(Chrono REQUIRED 
    COMPONENTS 
        Parsers    # 启用 PARSERS 模块
        VSG        # 启用 VSG 模块
)

if(NOT Chrono_FOUND)
    message(FATAL_ERROR "Chrono库未找到, 请先安装Chrono。")
endif()

message(STATUS "找到Chrono版本: ${CHRONO_VERSION}")
message(STATUS "Chrono include 目录: ${CHRONO_INCLUDE_DIRS}")
message(STATUS "Chrono库: ${CHRONO_LIBRARIES}")

# 设置编译选项
if(MSVC)
    add_compile_options(/W3)
else()
    add_compile_options(-Wall -Wextra -pedantic
        -Wno-unused-parameter
        -Wno-pedantic
        -Wno-sign-compare
        -Wno-delete-non-virtual-dtor
        -Wno-reorder
        -Wno-extra)
endif()

# ========== 项目目录配置 ==========
set(PROJECT_ROOT_DIR ${CMAKE_SOURCE_DIR})
set(INCLUDE_DIR ${PROJECT_ROOT_DIR}/include)
set(SRC_DIR ${PROJECT_ROOT_DIR}/src)
set(SCENARIOS_DIR ${PROJECT_ROOT_DIR}/scenarios)

# 传递项目根目录给代码（用于定位assets）
add_compile_definitions(PROJECT_ROOT_DIR="${PROJECT_ROOT_DIR}")

# ========== 核心库 ==========
add_library(satellite_core STATIC
    ${SRC_DIR}/satellite_docking_model.cpp
    ${SRC_DIR}/scenario_runner.cpp
)

target_include_directories(satellite_core PUBLIC 
    ${INCLUDE_DIR}
    ${CHRONO_INCLUDE_DIRS}
)

target_link_libraries(satellite_core 
    ${CHRONO_LIBRARIES}
)

target_compile_features(satellite_core PUBLIC cxx_std_17)

# 设置 VSG 数据目录
target_compile_definitions(satellite_core PUBLIC 
    CHRONO_DATA_DIR="${CHRONO_DATA_DIR}"
)

# ========== 场景可执行文件 ==========
file(GLOB SCENARIO_SOURCES "${SCENARIOS_DIR}/*.cpp")

foreach(scenario_src ${SCENARIO_SOURCES})
    # 获取文件名（不含路径和扩展名）
    get_filename_component(scenario_name ${scenario_src} NAME_WE)
    
    add_executable(${scenario_name} ${scenario_src})
    
    # 链接核心库
    target_link_libraries(${scenario_name} 
        satellite_core
        ${CHRONO_LIBRARIES}
    )
    
    # 包含目录
    target_include_directories(${scenario_name} PRIVATE 
        ${INCLUDE_DIR}
        ${CHRONO_INCLUDE_DIRS}
    )
    
    # C++标准
    target_compile_features(${scenario_name} PRIVATE cxx_std_17)
    
    # 输出目录
    set_target_properties(${scenario_name} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
    )
    
    message(STATUS "添加场景: ${scenario_name}")
endforeach()

install(TARGETS satellite_core
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(DIRECTORY ${INCLUDE_DIR}/ 
    DESTINATION include
    FILES_MATCHING PATTERN "*.h"
)

message(STATUS "=== 项目配置完成 ===")
message(STATUS "项目根目录: ${PROJECT_ROOT_DIR}")
message(STATUS "头文件目录: ${INCLUDE_DIR}")
message(STATUS "源文件目录: ${SRC_DIR}")
message(STATUS "场景目录: ${SCENARIOS_DIR}")
message(STATUS "生成的可执行文件在: ${CMAKE_BINARY_DIR}/bin/")