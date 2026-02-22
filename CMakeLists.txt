cmake_minimum_required(VERSION 3.10)
project(ManipulatorTrussDynamics VERSION 1.0.0 LANGUAGES CXX)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置构建类型
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

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

# 检查Chrono是否找到
if(NOT Chrono_FOUND)
    message(FATAL_ERROR "Chrono库未找到！请先安装Chrono。")
endif()

# 输出找到的信息
message(STATUS "找到Chrono版本: ${CHRONO_VERSION}")
message(STATUS "Chrono include 目录: ${CHRONO_INCLUDE_DIRS}")
message(STATUS "Chrono库: ${CHRONO_LIBRARIES}")

# 设置编译选项
if(MSVC)
    add_compile_options(/W3)
else()
    add_compile_options(-Wall -Wextra -pedantic)
endif()

# 创建函数以简化可执行文件创建过程
function(add_chrono_executable TARGET_NAME SOURCE_FILE)
    add_executable(${TARGET_NAME} ${SOURCE_FILE})
    
    # 设置包含目录
    target_include_directories(${TARGET_NAME} PRIVATE 
        ${CHRONO_INCLUDE_DIRS}
    )
    
    # 链接Chrono库
    target_link_libraries(${TARGET_NAME} 
        ${CHRONO_LIBRARIES}
    )

    # 设置 VSG 数据目录
    target_compile_definitions(${TARGET_NAME} PRIVATE CHRONO_DATA_DIR="${CHRONO_DATA_DIR}")
    
    # 设置C++标准
    target_compile_features(${TARGET_NAME} PRIVATE cxx_std_17)
    
    # 设置输出目录
    set_target_properties(${TARGET_NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

endfunction()

# 添加示例程序
add_chrono_executable(my_scene ./my_scene.cpp)

# 打印成功信息
message(STATUS "=== 项目配置完成 ===")
message(STATUS "可执行文件将在 build/bin/ 目录下生成")
message(STATUS "可用的可执行文件:")
get_property(TARGETS DIRECTORY PROPERTY BUILDSYSTEM_TARGETS)
foreach(TARGET ${TARGETS})
    message(STATUS "  - ${TARGET}")
endforeach()