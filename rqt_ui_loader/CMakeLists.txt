cmake_minimum_required(VERSION 2.8.3)

project(rqt_ui_loader)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rqt_gui rqt_gui_cpp)

find_package(Qt4 COMPONENTS QtCore QtGui QtUiTools REQUIRED)

include(${QT_USE_FILE})

set(rqt_ui_loader_SRCS
  src/rqt_ui_loader/ui_loader.cpp
  src/rqt_ui_loader/ui_loader_widget.cpp
)

set(rqt_ui_loader_HDRS
  include/rqt_ui_loader/ui_loader.h
  include/rqt_ui_loader/ui_loader_widget.h
)

set(rqt_ui_loader_UIS
  src/rqt_ui_loader/ui_loader_widget.ui
)

set(rqt_ui_loader_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  INCLUDE_DIRS ${rqt_ui_loader_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rqt_gui rqt_gui_cpp
)
catkin_python_setup()

qt4_wrap_cpp(rqt_ui_loader_MOCS ${rqt_ui_loader_HDRS})
qt4_wrap_ui(rqt_ui_loader_UIS_H ${rqt_ui_loader_UIS})

include_directories(${rqt_ui_loader_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${rqt_ui_loader_SRCS} ${rqt_ui_loader_MOCS} ${rqt_ui_loader_UIS_H})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTUITOOLS_LIBRARY})

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(PROGRAMS scripts/rqt_ui_loader
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(PROGRAMS scripts/rqt_ui_loader
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
