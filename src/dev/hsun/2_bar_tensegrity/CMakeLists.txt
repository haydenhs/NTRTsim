#link_directories(${LIB_DIR})

link_libraries(tgcreator
                util
                sensors
                core    
                terrain
		controllers 
                tgOpenGLSupport)

add_executable(AppSimpleModel
    SimpleModel.cpp
    tgSimpleLogger.cpp
    SimpleController.cpp
    AppSimpleModel.cpp
)
