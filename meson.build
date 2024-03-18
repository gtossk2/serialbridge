project('serialbridge', 'cpp',
    meson_version: '>=1.1.1',
    default_options: [
        'buildtype=debugoptimized',
        'warning_level=3',
        'werror=true',
        'cpp_std=c++23',
    ],
    version: '1.0',
)

add_project_arguments('-Wno-psabi', language: 'cpp')

phosphor_logging_dep = dependency('phosphor-logging')
sdbusplus_dep = dependency('sdbusplus')
sdeventplus_dep = dependency('sdeventplus')
stdplus_dep = dependency('stdplus')
CLI11_dep = dependency('CLI11')

deps = [dependency('libsystemd'),
        dependency('systemd'),
        sdeventplus_dep,
        stdplus_dep,
        sdbusplus_dep,
        phosphor_logging_dep,
        CLI11_dep
]

serialbridged = executable('serialbridged','serialbridged.cpp',
  dependencies: deps,
  install: true,
  install_dir: get_option('libexecdir'))

# Configure and install systemd unit files
systemd = dependency('systemd')
if systemd.found()
  conf_data = configuration_data()
  conf_data.set('BIN', get_option('prefix') / get_option('libexecdir') / serialbridged.name())
  configure_file(
    input: 'serialbridge@.service.in',
    output: 'serialbridge@.service',
    configuration: conf_data,
    install: true,
    install_dir: systemd.get_variable(pkgconfig: 'systemdsystemunitdir'))
endif
