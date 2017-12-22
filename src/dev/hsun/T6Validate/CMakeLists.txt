#link_directories(${LIB_DIR})

link_libraries(tgcreator
	       controllers
               core
               sensors
	       util
	       tgOpenGLSupport)

add_executable(AppT6Validate
    T6Model.cpp
    AppT6Validate.cpp
) 
