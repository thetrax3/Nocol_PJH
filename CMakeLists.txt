cmake_minimum_required(VERSION 3.10.2) #Cmake 최소 버전 요구
cmake_policy(SET CMP0074 NEW)

project(
    CppTest #필수 프로젝트 이름
    VERSION 0.1
    DESCRIPTION "테스트 프로젝트"
    LANGUAGES CXX)

# PCL을 찾아서 설정합니다.
find_package(PCL 1.11 REQUIRED) # PCL 라이브러리 가져오기

# PCL이 설치되어 있지 않은 경우 에러 메시지를 출력합니다.
if(NOT PCL_FOUND)
    message(FATAL_ERROR "PCL library not found. Please install PCL.")
endif()

# PCL을 포함할 디렉토리를 추가합니다.
include_directories(${PCL_INCLUDE_DIRS})

# 라이브러리를 링크합니다.
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 현재 디렉토리의 모든 소스 파일 가져오기
file(GLOB SOURCES "*.cpp")
file(GLOB_RECURSE SOURCES2 "Core/**/*.cpp")

# 실행 파일 생성 및 링크
#add_executable(project main.cpp Core/FileDialog/FileDialog.cpp)
add_executable(project ${SOURCES} ${SOURCES2})
target_link_libraries(project ${PCL_LIBRARIES})