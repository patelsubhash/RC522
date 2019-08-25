if(IDF_VERSION_MAJOR GREATER_EQUAL 4)
    idf_component_register(SRC_DIRS ESP32IDF
        REQUIRES log spi_flash
        INCLUDE_DIRS ESP32IDF)
else()
    set(COMPONENT_SRCDIRS ESP32IDF)
    set(COMPONENT_ADD_INCLUDEDIRS ESP32IDF)
    set(COMPONENT_REQUIRES log spi_flash)
    register_component()
endif()

