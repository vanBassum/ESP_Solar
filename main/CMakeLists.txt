


idf_component_register(
    SRCS 
        "main.cpp"  
        "st7735.cpp"
    INCLUDE_DIRS 
        "."
    REQUIRES
        ESP_Base
        ESP_Drivers
        ESP_Com
        ESP_LVGL
    )
