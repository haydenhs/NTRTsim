link_directories(${LIB_DIR})

link_libraries(FileHelpers)

add_executable(AppJSON
	AppJSON.cpp
)

target_link_libraries(AppJSON ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers)

configure_file("controlVars.json" "controlVars.json" COPYONLY)