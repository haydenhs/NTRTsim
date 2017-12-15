#link_directories(${LIB_DIR})

link_libraries(tgcreator
	       controllers
               core
               sensors
	       util
	       tgOpenGLSupport)

add_executable(Appsuperballmove
    T6Model.cpp
    Appsuperballmove.cpp
    T6TensionController.cpp
    tgSimpleLogger.cpp
) 


# To compile a controller, add a line like the
# following inside add_executable:
#    controllers/T6TensionController.cpp
