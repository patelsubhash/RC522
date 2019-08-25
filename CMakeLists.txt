if(IDF_VERSION_MAJOR GREATER_EQUAL 4)
    idf_component_register(SRC_DIRS ESP32IDF
        REQUIRES log
        INCLUDE_DIRS ESP32IDF)
else()
    set(COMPONENT_SRCDIRS ESP32IDF)
    set(COMPONENT_ADD_INCLUDEDIRS ESP32IDF)
    set(COMPONENT_REQUIRES log)
    register_component()
endif()

