set(config_platform "${CMAKE_SYSTEM}")
set(config_processor "${CMAKE_SYSTEM_PROCESSOR}")
set(config_relative_output_dir "${config_platform}/${config_processor}")
message("- Config Output Directory: ${config_output_dir}")
add_definitions(
-D_UNICODE 
-DUNICODE
)
message("- Windows Project:  Using Unicode")