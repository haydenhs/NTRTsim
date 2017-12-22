link_directories(${LIB_DIR})

link_libraries(tgcreator
                util
                sensors
                core    
                terrain 
                tgOpenGLSupport)

add_executable(AppPrismSensorTest
    PrismModel.cpp
    AppPrismModel.cpp
) 

