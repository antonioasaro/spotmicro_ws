idf_build_get_property(project_dir PROJECT_DIR)
idf_component_register(SRCS "main.cpp" "src/spot_micro_motion_cmd.cpp" "src/smfsm/spot_micro_idle.cpp" "src/smfsm/spot_micro_stand.cpp" "src/smfsm/spot_micro_state.cpp" "src/smfsm/spot_micro_transition_idle.cpp" "src/smfsm/spot_micro_transition_stand.cpp" "src/smfsm/spot_micro_walk.cpp" "src/i2cpwm_controller/i2cpwm_controller.cpp" "src/smfsm/spot_micro_cali.cpp"
                    INCLUDE_DIRS "include" "include/spot_micro_motion_cmd" "src/smfsm" "src/rate_limited_first_order_filter" "include/i2cpwm_controller"
                    EMBED_TXTFILES ${project_dir}/server_certs/ca_cert.pem)
component_compile_options(-std=c++14)
#### add_compile_options(-std=c++14)

