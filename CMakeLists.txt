cmake_minimum_required(VERSION 3.5)

project(SDNS-GUI VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets PrintSupport qcustomplot)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets PrintSupport)

set(PROJECT_SOURCES
        main.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui
        qcustomplot.h
        qcustomplot.cpp
        cereal/archives/adapters.hpp cereal/archives/binary.hpp cereal/archives/json.hpp cereal/archives/portable_binary.hpp cereal/archives/xml.hpp cereal/details/helpers.hpp cereal/details/polymorphic_impl.hpp cereal/details/polymorphic_impl_fwd.hpp cereal/details/static_object.hpp cereal/details/traits.hpp cereal/details/util.hpp cereal/external/rapidjson/error/en.h cereal/external/rapidjson/error/error.h cereal/external/rapidjson/internal/biginteger.h cereal/external/rapidjson/internal/diyfp.h cereal/external/rapidjson/internal/dtoa.h cereal/external/rapidjson/internal/ieee754.h cereal/external/rapidjson/internal/itoa.h cereal/external/rapidjson/internal/meta.h cereal/external/rapidjson/internal/pow10.h cereal/external/rapidjson/internal/regex.h cereal/external/rapidjson/internal/stack.h cereal/external/rapidjson/internal/strfunc.h cereal/external/rapidjson/internal/strtod.h cereal/external/rapidjson/internal/swap.h cereal/external/rapidjson/msinttypes/inttypes.h cereal/external/rapidjson/msinttypes/LICENSE cereal/external/rapidjson/msinttypes/stdint.h cereal/external/rapidjson/allocators.h cereal/external/rapidjson/cursorstreamwrapper.h cereal/external/rapidjson/document.h cereal/external/rapidjson/encodedstream.h cereal/external/rapidjson/encodings.h cereal/external/rapidjson/filereadstream.h cereal/external/rapidjson/filewritestream.h cereal/external/rapidjson/fwd.h cereal/external/rapidjson/istreamwrapper.h cereal/external/rapidjson/LICENSE cereal/external/rapidjson/memorybuffer.h cereal/external/rapidjson/memorystream.h cereal/external/rapidjson/ostreamwrapper.h cereal/external/rapidjson/pointer.h cereal/external/rapidjson/prettywriter.h cereal/external/rapidjson/rapidjson.h cereal/external/rapidjson/reader.h cereal/external/rapidjson/schema.h cereal/external/rapidjson/stream.h cereal/external/rapidjson/stringbuffer.h cereal/external/rapidjson/writer.h cereal/external/rapidxml/license.txt cereal/external/rapidxml/manual.html cereal/external/rapidxml/rapidxml.hpp cereal/external/rapidxml/rapidxml_iterators.hpp cereal/external/rapidxml/rapidxml_print.hpp cereal/external/rapidxml/rapidxml_utils.hpp cereal/external/base64.hpp cereal/external/LICENSE cereal/types/concepts/pair_associative_container.hpp cereal/types/array.hpp cereal/types/atomic.hpp cereal/types/base_class.hpp cereal/types/bitset.hpp cereal/types/boost_variant.hpp cereal/types/chrono.hpp cereal/types/common.hpp cereal/types/complex.hpp cereal/types/deque.hpp cereal/types/forward_list.hpp cereal/types/functional.hpp cereal/types/list.hpp cereal/types/map.hpp cereal/types/memory.hpp cereal/types/optional.hpp cereal/types/polymorphic.hpp cereal/types/queue.hpp cereal/types/set.hpp cereal/types/stack.hpp cereal/types/string.hpp cereal/types/tuple.hpp cereal/types/unordered_map.hpp cereal/types/unordered_set.hpp cereal/types/utility.hpp cereal/types/valarray.hpp cereal/types/variant.hpp cereal/types/vector.hpp cereal/access.hpp cereal/cereal.hpp cereal/macros.hpp cereal/specialize.hpp cereal/version.hpp
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(SDNS-GUI
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
        graphwidget.h
        graphwidget.cpp
        GUISupport.h
        MapProcessing.h
        cereal/archives/adapters.hpp cereal/archives/binary.hpp cereal/archives/json.hpp cereal/archives/portable_binary.hpp cereal/archives/xml.hpp cereal/details/helpers.hpp cereal/details/polymorphic_impl.hpp cereal/details/polymorphic_impl_fwd.hpp cereal/details/static_object.hpp cereal/details/traits.hpp cereal/details/util.hpp cereal/external/rapidjson/error/en.h cereal/external/rapidjson/error/error.h cereal/external/rapidjson/internal/biginteger.h cereal/external/rapidjson/internal/diyfp.h cereal/external/rapidjson/internal/dtoa.h cereal/external/rapidjson/internal/ieee754.h cereal/external/rapidjson/internal/itoa.h cereal/external/rapidjson/internal/meta.h cereal/external/rapidjson/internal/pow10.h cereal/external/rapidjson/internal/regex.h cereal/external/rapidjson/internal/stack.h cereal/external/rapidjson/internal/strfunc.h cereal/external/rapidjson/internal/strtod.h cereal/external/rapidjson/internal/swap.h cereal/external/rapidjson/msinttypes/inttypes.h cereal/external/rapidjson/msinttypes/LICENSE cereal/external/rapidjson/msinttypes/stdint.h cereal/external/rapidjson/allocators.h cereal/external/rapidjson/cursorstreamwrapper.h cereal/external/rapidjson/document.h cereal/external/rapidjson/encodedstream.h cereal/external/rapidjson/encodings.h cereal/external/rapidjson/filereadstream.h cereal/external/rapidjson/filewritestream.h cereal/external/rapidjson/fwd.h cereal/external/rapidjson/istreamwrapper.h cereal/external/rapidjson/LICENSE cereal/external/rapidjson/memorybuffer.h cereal/external/rapidjson/memorystream.h cereal/external/rapidjson/ostreamwrapper.h cereal/external/rapidjson/pointer.h cereal/external/rapidjson/prettywriter.h cereal/external/rapidjson/rapidjson.h cereal/external/rapidjson/reader.h cereal/external/rapidjson/schema.h cereal/external/rapidjson/stream.h cereal/external/rapidjson/stringbuffer.h cereal/external/rapidjson/writer.h cereal/external/rapidxml/license.txt cereal/external/rapidxml/manual.html cereal/external/rapidxml/rapidxml.hpp cereal/external/rapidxml/rapidxml_iterators.hpp cereal/external/rapidxml/rapidxml_print.hpp cereal/external/rapidxml/rapidxml_utils.hpp cereal/external/base64.hpp cereal/external/LICENSE cereal/types/concepts/pair_associative_container.hpp cereal/types/array.hpp cereal/types/atomic.hpp cereal/types/base_class.hpp cereal/types/bitset.hpp cereal/types/boost_variant.hpp cereal/types/chrono.hpp cereal/types/common.hpp cereal/types/complex.hpp cereal/types/deque.hpp cereal/types/forward_list.hpp cereal/types/functional.hpp cereal/types/list.hpp cereal/types/map.hpp cereal/types/memory.hpp cereal/types/optional.hpp cereal/types/polymorphic.hpp cereal/types/queue.hpp cereal/types/set.hpp cereal/types/stack.hpp cereal/types/string.hpp cereal/types/tuple.hpp cereal/types/unordered_map.hpp cereal/types/unordered_set.hpp cereal/types/utility.hpp cereal/types/valarray.hpp cereal/types/variant.hpp cereal/types/vector.hpp cereal/access.hpp cereal/cereal.hpp cereal/macros.hpp cereal/specialize.hpp cereal/version.hpp
        Serial.h
        TCPSocket.h
        LightSerial.h
        Structs.h
        MapDataConversion.h
        WatchdogService.h
        ConsoleLogger.h


    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET SDNS-GUI APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
else()
    if(ANDROID)
        add_library(SDNS-GUI SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(SDNS-GUI
            ${PROJECT_SOURCES}
        )
    endif()
endif()

include_directories(/home/andrew/Documents/SDNS-GUI/)
link_directories(/home/andrew/Documents/SDNS-GUI/)
target_link_libraries(SDNS-GUI PRIVATE Qt${QT_VERSION_MAJOR}::Widgets Qt6::PrintSupport)

# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# If you are developing for iOS or macOS you should consider setting an
# explicit, fixed bundle identifier manually though.
if(${QT_VERSION} VERSION_LESS 6.1.0)
  set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.SDNS-GUI)
endif()
set_target_properties(SDNS-GUI PROPERTIES
    ${BUNDLE_ID_OPTION}
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

include(GNUInstallDirs)
install(TARGETS SDNS-GUI
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(SDNS-GUI)
endif()
