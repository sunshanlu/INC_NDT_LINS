add_executable(lo_example lo_example.cc)
target_link_libraries(lo_example inc_ndt_lins)

add_executable(loosely_lio_example loosely_lio_example.cc)
target_link_libraries(loosely_lio_example inc_ndt_lins)

add_executable(data_stamp_test data_stamp_test.cc)
target_link_libraries(data_stamp_test inc_ndt_lins)

add_executable(iekf_lio_example iekf_lio_example.cc)
target_link_libraries(iekf_lio_example inc_ndt_lins)

add_executable(preint_lio_example preint_lio_example.cc)
target_link_libraries(preint_lio_example inc_ndt_lins)