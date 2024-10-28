# This file will be configured to contain variables for CPack. These variables
# should be set in the CMake list file of the project before CPack module is
# included. The list of available CPACK_xxx variables and their associated
# documentation may be obtained using
#  cpack --help-variable-list
#
# Some variables are common to all generators (e.g. CPACK_PACKAGE_NAME)
# and some are specific to a generator
# (e.g. CPACK_NSIS_EXTRA_INSTALL_COMMANDS). The generator specific variables
# usually begin with CPACK_<GENNAME>_xxxx.


set(CPACK_BUILD_SOURCE_DIRS "/home/tigerwuu-ncs/formation_ws/src/observer/gz_plugin/wind_effects;/home/tigerwuu-ncs/formation_ws/src/observer/gz_plugin/wind_effects/build")
set(CPACK_CMAKE_GENERATOR "Unix Makefiles")
set(CPACK_COMPONENTS_ALL "")
set(CPACK_COMPONENT_UNSPECIFIED_HIDDEN "TRUE")
set(CPACK_COMPONENT_UNSPECIFIED_REQUIRED "TRUE")
set(CPACK_DEFAULT_PACKAGE_DESCRIPTION_FILE "/usr/share/cmake-3.22/Templates/CPack.GenericDescription.txt")
set(CPACK_DEFAULT_PACKAGE_DESCRIPTION_SUMMARY "gz-sim7 built using CMake")
set(CPACK_GENERATOR "DEB")
set(CPACK_INSTALL_CMAKE_PROJECTS "/home/tigerwuu-ncs/formation_ws/src/observer/gz_plugin/wind_effects/build;gz-sim7;ALL;/")
set(CPACK_INSTALL_PREFIX "/usr/local")
set(CPACK_MODULE_PATH "/usr/share/cmake/gz-cmake3/cmake3")
set(CPACK_NSIS_DISPLAY_NAME "gz-sim7 7.9.0")
set(CPACK_NSIS_INSTALLER_ICON_CODE "")
set(CPACK_NSIS_INSTALLER_MUI_ICON_CODE "")
set(CPACK_NSIS_INSTALL_ROOT "$PROGRAMFILES")
set(CPACK_NSIS_PACKAGE_NAME "gz-sim7 7.9.0")
set(CPACK_NSIS_UNINSTALL_NAME "Uninstall")
set(CPACK_OUTPUT_CONFIG_FILE "/home/tigerwuu-ncs/formation_ws/src/observer/gz_plugin/wind_effects/build/CPackConfig.cmake")
set(CPACK_PACKAGE_DEFAULT_LOCATION "/")
set(CPACK_PACKAGE_DESCRIPTION_FILE "/usr/share/cmake-3.22/Templates/CPack.GenericDescription.txt")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "gz-sim7 built using CMake")
set(CPACK_PACKAGE_FILE_NAME "gz-sim7-7.9.0-Linux")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "gz-sim7 7.9.0")
set(CPACK_PACKAGE_INSTALL_REGISTRY_KEY "gz-sim7 7.9.0")
set(CPACK_PACKAGE_NAME "gz-sim7")
set(CPACK_PACKAGE_RELOCATABLE "true")
set(CPACK_PACKAGE_VENDOR "Humanity")
set(CPACK_PACKAGE_VERSION "7.9.0")
set(CPACK_PACKAGE_VERSION_MAJOR "7")
set(CPACK_PACKAGE_VERSION_MINOR "9")
set(CPACK_PACKAGE_VERSION_PATCH "0")
set(CPACK_PROJECT_CONFIG_FILE "/home/tigerwuu-ncs/formation_ws/src/observer/gz_plugin/wind_effects/build/cpack_options.cmake")
set(CPACK_RESOURCE_FILE_LICENSE "/usr/share/cmake-3.22/Templates/CPack.GenericLicense.txt")
set(CPACK_RESOURCE_FILE_README "/usr/share/cmake-3.22/Templates/CPack.GenericDescription.txt")
set(CPACK_RESOURCE_FILE_WELCOME "/usr/share/cmake-3.22/Templates/CPack.GenericWelcome.txt")
set(CPACK_SET_DESTDIR "OFF")
set(CPACK_SOURCE_GENERATOR "TBZ2;ZIP")
set(CPACK_SOURCE_IGNORE_FILES "TODO;.hg/;.sw.$;/build/;.hgtags;.hgignore;appveyor.yml;.travis.yml;codecov.yml")
set(CPACK_SOURCE_OUTPUT_CONFIG_FILE "/home/tigerwuu-ncs/formation_ws/src/observer/gz_plugin/wind_effects/build/CPackSourceConfig.cmake")
set(CPACK_SYSTEM_NAME "Linux")
set(CPACK_THREADS "1")
set(CPACK_TOPLEVEL_TAG "Linux")
set(CPACK_WIX_SIZEOF_VOID_P "8")

if(NOT CPACK_PROPERTIES_FILE)
  set(CPACK_PROPERTIES_FILE "/home/tigerwuu-ncs/formation_ws/src/observer/gz_plugin/wind_effects/build/CPackProperties.cmake")
endif()

if(EXISTS ${CPACK_PROPERTIES_FILE})
  include(${CPACK_PROPERTIES_FILE})
endif()
