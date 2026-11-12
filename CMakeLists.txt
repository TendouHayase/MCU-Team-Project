cmake_minimum_required(VERSION 3.20)

# =============================================================================
# 라인트레이서 (Arduino UNO / ATmega328P) - CMake 빌드
#
# 저장소 구조 (Arduino IDE 에서도 linetracer/ 폴더를 그대로 열 수 있다)
#   CMakeLists.txt
#   cmake/avr-toolchain.cmake
#   linetracer/
#     linetracer.ino      <- 비워 둔다. CMake 는 .ino 를 컴파일하지 않는다.
#     src/                <- 여기 아래의 .c/.cpp/.S 는 전부 컴파일된다 (Arduino 규칙과 동일)
#
# 구성 예
#   cmake -S . -B build -G Ninja -DUPLOAD_PORT=COM5
#   cmake --build build                  # 빌드 + .hex 생성 + 크기 출력
#   cmake --build build --target upload  # avrdude 업로드
#   cmake --build build --target ide-check   # arduino-cli 로 Arduino IDE 호환 빌드 확인
# =============================================================================

# ---- 툴체인 (project() 이전에 지정해야 한다) ---------------------------------
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_LIST_DIR}/cmake/avr-toolchain.cmake"
        CACHE FILEPATH "AVR cross toolchain file")
endif()

project(linetracer LANGUAGES C CXX ASM)

if(NOT CMAKE_CROSSCOMPILING OR NOT CMAKE_C_COMPILER_ID STREQUAL "GNU")
    message(FATAL_ERROR
        "avr-gcc cross toolchain is required "
        "(CMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}, C compiler=${CMAKE_C_COMPILER})")
endif()

# ---- 보드 설정 --------------------------------------------------------------
set(BOARD_MCU atmega328p CACHE STRING "-mmcu 값")
set(BOARD_F_CPU 16000000L CACHE STRING "F_CPU")
set(BOARD_VARIANT standard CACHE STRING "Arduino 코어의 variants/ 하위 폴더")
set(BOARD_DEFINE ARDUINO_AVR_UNO CACHE STRING "보드 식별 매크로")
set(ARDUINO_IDE_VERSION 10607 CACHE STRING "ARDUINO 매크로 값")

# ---- 언어 표준: Arduino AVR 코어와 동일하게 고정 ------------------------------
# 더 높은 표준을 쓰면 Arduino IDE 쪽에서 컴파일되지 않는다.
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Arduino 는 항상 -Os -g 로 빌드한다. CMAKE_BUILD_TYPE 에 따라 최적화 수준이
# 달라지지 않도록 구성별 기본 플래그를 비운다.
foreach(_lang C CXX ASM)
    foreach(_cfg DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
        set(CMAKE_${_lang}_FLAGS_${_cfg} "")
    endforeach()
endforeach()

# -flto 오브젝트를 담는 정적 라이브러리는 LTO 플러그인을 쓰는 avr-gcc-ar 로 묶어야 한다.
if(CMAKE_C_COMPILER_AR AND CMAKE_C_COMPILER_RANLIB)
    set(CMAKE_AR "${CMAKE_C_COMPILER_AR}")
    set(CMAKE_RANLIB "${CMAKE_C_COMPILER_RANLIB}")
else()
    message(WARNING "avr-gcc-ar / avr-gcc-ranlib not found; LTO archives may fail to link")
endif()

get_filename_component(_toolchain_bin "${CMAKE_C_COMPILER}" DIRECTORY)

# ---- Arduino AVR 코어 위치 ---------------------------------------------------
# Arduino IDE / arduino-cli 가 설치한 코어를 그대로 참조한다.
# 자동 탐지가 안 되면 -DARDUINO_AVR_CORE_DIR=<.../hardware/avr/1.8.x> 로 지정한다.
set(ARDUINO_AVR_CORE_DIR "" CACHE PATH "Arduino AVR 코어 루트 (cores/, variants/ 를 포함하는 폴더)")

if(NOT ARDUINO_AVR_CORE_DIR)
    set(_arduino15_roots "")
    if(DEFINED ENV{LOCALAPPDATA})
        file(TO_CMAKE_PATH "$ENV{LOCALAPPDATA}/Arduino15" _p)
        list(APPEND _arduino15_roots "${_p}")
    endif()
    if(DEFINED ENV{HOME})
        file(TO_CMAKE_PATH "$ENV{HOME}" _home)
        list(APPEND _arduino15_roots "${_home}/.arduino15" "${_home}/Library/Arduino15")
    endif()

    set(_core_candidates "")
    foreach(_root IN LISTS _arduino15_roots)
        file(GLOB _found LIST_DIRECTORIES true "${_root}/packages/arduino/hardware/avr/*")
        list(APPEND _core_candidates ${_found})
    endforeach()
    list(SORT _core_candidates COMPARE NATURAL ORDER DESCENDING) # 가장 높은 버전 우선

    foreach(_cand IN LISTS _core_candidates)
        if(EXISTS "${_cand}/cores/arduino/Arduino.h")
            set(ARDUINO_AVR_CORE_DIR "${_cand}" CACHE PATH "" FORCE)
            break()
        endif()
    endforeach()
endif()

set(_core_src "${ARDUINO_AVR_CORE_DIR}/cores/arduino")
set(_core_variant "${ARDUINO_AVR_CORE_DIR}/variants/${BOARD_VARIANT}")

if(NOT EXISTS "${_core_src}/Arduino.h" OR NOT EXISTS "${_core_variant}/pins_arduino.h")
    message(FATAL_ERROR
        "Arduino AVR core not found. Install 'Arduino AVR Boards' in Arduino IDE, "
        "or pass -DARDUINO_AVR_CORE_DIR=<path to .../hardware/avr/<version>>")
endif()
message(STATUS "Arduino AVR core: ${ARDUINO_AVR_CORE_DIR}")

# ---- 공통 플래그 (Arduino AVR platform.txt 의 플래그를 따른다) ----------------
add_library(board INTERFACE)

target_compile_definitions(board INTERFACE
    F_CPU=${BOARD_F_CPU}
    ARDUINO=${ARDUINO_IDE_VERSION}
    ${BOARD_DEFINE}
    ARDUINO_ARCH_AVR
)

target_compile_options(board INTERFACE
    -mmcu=${BOARD_MCU}
    -g -Os
    -flto
    "$<$<COMPILE_LANGUAGE:C,CXX>:-ffunction-sections;-fdata-sections>"
    "$<$<COMPILE_LANGUAGE:C>:-fno-fat-lto-objects>"
    "$<$<COMPILE_LANGUAGE:CXX>:-fpermissive;-fno-exceptions;-fno-threadsafe-statics;-Wno-error=narrowing>"
)

# SYSTEM 으로 지정하지 않는다. (-isystem 은 일부 avr-gcc 에서 헤더를 extern "C" 로 취급한다)
target_include_directories(board INTERFACE "${_core_src}" "${_core_variant}")

target_link_options(board INTERFACE
    -mmcu=${BOARD_MCU}
    -g -Os
    -flto -fuse-linker-plugin
    LINKER:--gc-sections
)

# ---- Arduino 코어 ------------------------------------------------------------
# Arduino 와 같이 정적 라이브러리(core.a)로 묶는다. 참조되는 오브젝트만 링크되므로
# 코어가 정의한 ISR(INT0, TIMER2 등)이 사용자 ISR 과 충돌하지 않는다.
file(GLOB _core_sources CONFIGURE_DEPENDS
    "${_core_src}/*.c" "${_core_src}/*.cpp" "${_core_src}/*.S")

add_library(arduino_core STATIC ${_core_sources})
target_link_libraries(arduino_core PUBLIC board)
target_compile_options(arduino_core PRIVATE -w) # 코어의 경고는 표시하지 않는다

# ---- 스케치 ------------------------------------------------------------------
set(SKETCH_DIR "${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}")

if(NOT EXISTS "${SKETCH_DIR}/${PROJECT_NAME}.ino")
    message(WARNING
        "${PROJECT_NAME}/${PROJECT_NAME}.ino is missing; Arduino IDE cannot open the sketch without it")
endif()

# Arduino 규칙: 스케치 루트의 소스 + src/ 하위 전체(재귀). .ino 는 대상에서 제외한다.
file(GLOB _sketch_root_sources CONFIGURE_DEPENDS
    "${SKETCH_DIR}/*.c" "${SKETCH_DIR}/*.cpp" "${SKETCH_DIR}/*.S")
file(GLOB_RECURSE _sketch_src_sources CONFIGURE_DEPENDS
    "${SKETCH_DIR}/src/*.c" "${SKETCH_DIR}/src/*.cpp" "${SKETCH_DIR}/src/*.S")
set(_sketch_sources ${_sketch_root_sources} ${_sketch_src_sources})

if(NOT _sketch_sources)
    message(FATAL_ERROR "No sources found under ${SKETCH_DIR} (expected e.g. src/main.cpp)")
endif()

# 소스를 실행 파일에 직접 넣는다. 정적 라이브러리로 묶으면 참조되지 않는 ISR 이
# 링크에서 빠질 수 있고, Arduino IDE 의 링크 동작과도 달라진다.
add_executable(${PROJECT_NAME} ${_sketch_sources})

set_target_properties(${PROJECT_NAME} PROPERTIES
    SUFFIX ".elf"
    LINKER_LANGUAGE C # Arduino 와 같이 avr-gcc 로 링크한다
)

# Arduino 는 스케치 루트만 인클루드 경로에 넣는다. 경로를 더 추가하면
# CMake 에서만 통과하는 #include 가 생기므로 추가하지 않는다.
target_include_directories(${PROJECT_NAME} PRIVATE "${SKETCH_DIR}")

target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)

target_link_libraries(${PROJECT_NAME} PRIVATE board arduino_core m)

target_link_options(${PROJECT_NAME} PRIVATE
    "LINKER:-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map")

# ---- .hex 생성과 크기 출력 ---------------------------------------------------
find_program(AVR_SIZE NAMES avr-size HINTS "${_toolchain_bin}" REQUIRED)

set(_hex_name "${PROJECT_NAME}.hex")

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND "${CMAKE_OBJCOPY}" -O ihex -R .eeprom "$<TARGET_FILE:${PROJECT_NAME}>" "${_hex_name}"
    # text + data = 플래시 사용량, data + bss = RAM 사용량
    COMMAND "${AVR_SIZE}" "$<TARGET_FILE:${PROJECT_NAME}>"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    BYPRODUCTS "${CMAKE_CURRENT_BINARY_DIR}/${_hex_name}"
    VERBATIM
)

# ---- 업로드 (avrdude) --------------------------------------------------------
set(UPLOAD_PORT "" CACHE STRING "직렬 포트 (예: COM5, /dev/ttyUSB0)")
set(UPLOAD_BAUD 115200 CACHE STRING "부트로더 통신 속도")
set(AVRDUDE_CONF "" CACHE FILEPATH "avrdude.conf 경로 (Arduino 에 포함된 avrdude 를 쓸 때 필요)")

find_program(AVRDUDE NAMES avrdude HINTS "${_toolchain_bin}")

if(NOT AVRDUDE)
    message(STATUS "avrdude not found: 'upload' target disabled")
elseif(NOT UPLOAD_PORT)
    add_custom_target(upload
        COMMAND "${CMAKE_COMMAND}" -E echo "UPLOAD_PORT is not set. Reconfigure with -DUPLOAD_PORT=<port>"
        COMMAND "${CMAKE_COMMAND}" -E false
        VERBATIM
    )
else()
    set(_avrdude_args -p ${BOARD_MCU} -c arduino -P "${UPLOAD_PORT}" -b ${UPLOAD_BAUD} -D)
    if(AVRDUDE_CONF)
        list(PREPEND _avrdude_args -C "${AVRDUDE_CONF}")
    endif()

    # 파일명을 상대 경로로 넘겨 Windows 드라이브 문자의 ':' 가 -U 구문과 섞이지 않게 한다.
    add_custom_target(upload
        COMMAND "${AVRDUDE}" ${_avrdude_args} -U "flash:w:${_hex_name}:i"
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
        DEPENDS ${PROJECT_NAME}
        USES_TERMINAL
        VERBATIM
    )
endif()

# ---- Arduino IDE 호환성 확인 (arduino-cli 는 Arduino IDE 2 와 같은 빌더를 쓴다) ---
find_program(ARDUINO_CLI NAMES arduino-cli)

if(ARDUINO_CLI)
    add_custom_target(ide-check
        COMMAND "${ARDUINO_CLI}" compile
        --fqbn arduino:avr:uno
        --warnings default
        --build-path "${CMAKE_CURRENT_BINARY_DIR}/arduino-cli"
        "${SKETCH_DIR}"
        USES_TERMINAL
        VERBATIM
    )
else()
    message(STATUS "arduino-cli not found: 'ide-check' target disabled")
endif()
