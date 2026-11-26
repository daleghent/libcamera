# SPDX-License-Identifier: CC0-1.0

subdir('libipa')

ipa_test = [
    {'name': 'ipa_module_test', 'sources': ['ipa_module_test.cpp']},
    {'name': 'ipa_interface_test', 'sources': ['ipa_interface_test.cpp']},
]

foreach test : ipa_test
    exe = executable(test['name'], test['sources'],
                     dependencies : [libcamera_private, libipa_dep],
                     link_with : [test_libraries],
                     include_directories : [test_includes_internal])

    test(test['name'], exe, suite : 'ipa')
endforeach

if 'rpi/pisp' in enabled_ipa_names or 'rpi/vc4' in enabled_ipa_names
    # Link the sensor's static registrar even though it exports no public symbols.
    exe = executable('imx585', 'imx585.cpp',
                     objects : rpi_ipa_cam_helper_lib.extract_objects('cam_helper_imx585.cpp'),
                     dependencies : libcamera_private,
                     link_with : [rpi_ipa_cam_helper_lib, rpi_ipa_controller_lib, test_libraries],
                     include_directories : [rpi_ipa_cam_helper_includes, test_includes_internal])
    test('imx585', exe, suite : 'ipa')
endif
