add_library(mytrk SHARED
        mybackend.cpp
        myframe.cpp
        myfrontend.cpp
        mytrk_list.cpp)

target_link_libraries(mytrk
        ${THIRD_PARTY_LIBS})

add_executable(factor_manage factor_manage.cpp)
target_link_libraries(factor_manage myslam ${THIRD_PARTY_LIBS} ${catkin_LIBRYRIES})
#需要添加对本项目生成的一些东西的依赖
add_dependencies(factor_manage ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

