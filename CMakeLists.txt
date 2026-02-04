cmake_minimum_required(VERSION 3.12)
project(h_fsm_controller)

set(CMAKE_CXX_STANDARD 17)

# 1. 패키지 검색
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)

# 2. 라이브러리 및 헤더 경로 (기존 설정 유지)
link_directories(
  /usr/local/lib 
  /ws/unitree_sdk2/lib/x86_64
  /ws/unitree_sdk2/thirdparty/lib/x86_64
)

include_directories(
  /ws/unitree_sdk2/thirdparty/include/ddscxx
  /ws/unitree_sdk2/thirdparty/include 
  /ws/unitree_sdk2/include
  /usr/local/include
  /usr/include/eigen3
  ${CMAKE_CURRENT_SOURCE_DIR}/include # 상대 경로 보장
)

# 3. 라이브러리 생성 (FSM 로직)
add_library(${PROJECT_NAME}_lib SHARED src/FSM.cpp)

# 라이브러리 자체가 외부 라이브러리에 의존함을 명시
target_link_libraries(${PROJECT_NAME}_lib 
  PUBLIC
    unitree_sdk2
    yaml-cpp
    ddsc
    ddscxx
    ${Boost_LIBRARIES}
    fmt
    pthread
    rt
)

# 4. 실행 파일 생성
add_executable(h_fsm_ctrl src/main.cpp)

# 5. 실행 파일 링크
target_link_libraries(h_fsm_ctrl 
  PRIVATE 
    ${PROJECT_NAME}_lib
)
