link_directories(${LIB_DIR})

link_libraries(tgcreator
	       controllers
               core
               sensors
	       util
               FileHelpers
	       tgOpenGLSupport)

add_executable(AppT6SineWaves
    T6Model.cpp
    AppT6SineWaves.cpp
    T6SineWaves.cpp
    tgSimpleLogger.cpp
) 

target_link_libraries(AppT6SineWaves ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers)

configure_file("controlVars.json" "controlVars.json" COPYONLY)
