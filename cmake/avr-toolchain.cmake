# =============================================================================
# avr-gcc 크로스 툴체인 파일
#
# avr-gcc 탐색 순서
#   1. -DAVR_TOOLCHAIN_DIR=<설치 루트 또는 bin 폴더> 로 지정한 위치 (지정 시 이 위치만 본다)
#   2. Arduino IDE / arduino-cli 가 설치한 avr-gcc (Arduino15/packages/arduino/tools/avr-gcc)
#      -> 팀원의 Arduino IDE 와 같은 컴파일러를 쓰게 되므로 PATH 보다 우선한다.
#   3. PATH (예: MSYS2 의 avr-gcc)
#
# 탐색 결과는 캐시에 남는다. 툴체인을 바꿀 때는 빌드 디렉터리를 새로 만든다.
# =============================================================================

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR avr)

# 컴파일러 검사 단계에서 실행 파일을 링크하지 않는다. (-mmcu 없이 링크하면 실패한다)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(AVR_TOOLCHAIN_DIR "" CACHE PATH "avr-gcc 설치 루트 또는 bin 폴더")

# try_compile() 이 만드는 하위 프로젝트에도 같은 값을 전달한다.
list(APPEND CMAKE_TRY_COMPILE_PLATFORM_VARIABLES AVR_TOOLCHAIN_DIR)

# ---- avr-gcc 위치 탐색 -------------------------------------------------------
set(_avr_hints "")
set(_avr_find_opts "")

if(AVR_TOOLCHAIN_DIR)
    list(APPEND _avr_hints "${AVR_TOOLCHAIN_DIR}/bin" "${AVR_TOOLCHAIN_DIR}")
    set(_avr_find_opts NO_DEFAULT_PATH)
else()
    set(_arduino15_roots "")
    if(DEFINED ENV{LOCALAPPDATA})
        file(TO_CMAKE_PATH "$ENV{LOCALAPPDATA}/Arduino15" _p)
        list(APPEND _arduino15_roots "${_p}")
    endif()
    if(DEFINED ENV{HOME})
        file(TO_CMAKE_PATH "$ENV{HOME}" _home)
        list(APPEND _arduino15_roots "${_home}/.arduino15" "${_home}/Library/Arduino15")
    endif()

    set(_arduino_gcc_dirs "")
    foreach(_root IN LISTS _arduino15_roots)
        file(GLOB _found LIST_DIRECTORIES true "${_root}/packages/arduino/tools/avr-gcc/*")
        list(APPEND _arduino_gcc_dirs ${_found})
    endforeach()
    list(SORT _arduino_gcc_dirs COMPARE NATURAL ORDER DESCENDING) # 가장 높은 버전 우선

    foreach(_dir IN LISTS _arduino_gcc_dirs)
        list(APPEND _avr_hints "${_dir}/bin")
    endforeach()
endif()

find_program(AVR_GCC NAMES avr-gcc HINTS ${_avr_hints} ${_avr_find_opts})

if(NOT AVR_GCC)
    message(FATAL_ERROR
        "avr-gcc not found. Install 'Arduino AVR Boards' in Arduino IDE, add avr-gcc to PATH, "
        "or pass -DAVR_TOOLCHAIN_DIR=<avr-gcc install dir>")
endif()

get_filename_component(_avr_bin "${AVR_GCC}" DIRECTORY)

# ---- 나머지 도구는 avr-gcc 와 같은 폴더에서만 찾는다 (버전 혼용 방지) ----------
function(_avr_find_tool var name)
    find_program(${var} NAMES ${name} PATHS "${_avr_bin}" NO_DEFAULT_PATH)
    if(NOT ${var})
        message(FATAL_ERROR "${name} not found in ${_avr_bin}")
    endif()
endfunction()

set(CMAKE_C_COMPILER "${AVR_GCC}")
set(CMAKE_ASM_COMPILER "${AVR_GCC}")
_avr_find_tool(CMAKE_CXX_COMPILER avr-g++)

# -flto 오브젝트를 다루려면 LTO 플러그인을 넘겨 주는 gcc-* 래퍼를 써야 한다.
_avr_find_tool(CMAKE_AR avr-gcc-ar)
_avr_find_tool(CMAKE_RANLIB avr-gcc-ranlib)
_avr_find_tool(CMAKE_NM avr-gcc-nm)
_avr_find_tool(CMAKE_OBJCOPY avr-objcopy)
_avr_find_tool(CMAKE_OBJDUMP avr-objdump)

# ---- find_* 동작 -------------------------------------------------------------
# 프로그램은 호스트에서 찾고, 라이브러리와 헤더는 호스트 것을 집어 오지 않게 한다.
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
