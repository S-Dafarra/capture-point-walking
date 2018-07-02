################################################################################
#                                                                              #
# Copyright (C) 2017 Fondazione Istitito Italiano di Tecnologia (IIT)          #
# All Rights Reserved.                                                         #
#                                                                              #
################################################################################

# @authors: Stefano Dafarra <stefano.dafarra@iit.it>

# List the subdirectory (http://stackoverflow.com/questions/7787823/cmake-how-to-get-the-name-of-all-subdirectories-of-a-directory)
macro(SUBDIRLIST result curdir)
  file(GLOB children RELATIVE ${curdir} ${curdir}/*)
  set(dirlist "")
  foreach(child ${children})
    if(IS_DIRECTORY ${curdir}/${child})
      list(APPEND dirlist ${child})
    endif()
  endforeach()
  set(${result} ${dirlist})
endmacro()


macro(INSTALL_ALL_ROBOT_FILES appdir)
    # Get list of models
    subdirlist(subdirs ${appdir}/robots/)
    # Install each model
    foreach (dir ${subdirs})
        yarp_install(DIRECTORY ${appdir}/robots/${dir} DESTINATION ${YARP_ROBOTS_INSTALL_DIR}/)
    endforeach ()
endmacro()
